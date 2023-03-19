
/* Includes ------------------------------------------------------------------*/
#include "options.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_adc.h"

// #if defined(USE_FULL_ASSERT)
// #include "stm32_assert.h"
// #endif /* USE_FULL_ASSERT */
#include "gpio.h"
#include "pwm.h"
#include "audio.h"
#include "opto.h"

#include "uarts.h"
#include <stdio.h>	// pour snprintf

void SystemClock_Config(void);
void cmd_handler( char c );

// contexte global -----------------------------------------------------------

unsigned int cnt100Hz = 0;
unsigned int inhibition = 0;

// emission : par message
volatile int msg_request = 0;
char txbuf[64];
volatile int txindex;

// reception : fifo circulaire
#ifdef RX_FIFO
#define QRX 32		// a power of 2 !!!
char rxbuf[QRX];
volatile unsigned int rxwi=0;	// write index
volatile unsigned int rxri=0;	// read index
// exemple de lecture du fifo  :
// 	while	( rxwi - rxri )
//		{
//		int c = rxbuf[(rxri++)&(QRX-1)];
//		... }
#endif



// systick interrupt handler
void SysTick_Handler()
{
++cnt100Hz;
if	( etat.pos < 0 )
	{
	switch	( cnt100Hz % 100 )
		{
		case 0 :
			LED_ON();
			break;
		case 5 :
			LED_OFF();
			break;

		case 90 :
			snprintf( txbuf, sizeof(txbuf), "%d -> acc1 = %d, demod %d\n", adc_raw, acc1 >> LOG_TAU1, acc2 >> ( LOG_TAU2 - 8 ) );
			txindex = 0;
			UART2_TX_INT_enable();
			break;

		}
	}
else	LED_ON();
if	(
	// ( ( IS_PA12_SET() == 0 ) || ( IS_PB13_SET() ) ) &&
	( IS_PB13_SET() ) &&
	( etat.pos < 0 )
	)
	if	( cnt100Hz > inhibition )
		{
		inhibition = cnt100Hz + ( DUREE_INH * 100 );
		audio_start( frein );
		}
	}

// UART2 interrupt handler
void USART2_IRQHandler( void )
{
if	(
	( LL_USART_IsActiveFlag_TXE( USART2 ) ) &&
	( LL_USART_IsEnabledIT_TXE( USART2 ) )
	)
	{	// messages de taille variable
	if	( txbuf[txindex] == 0 )
		UART2_TX_INT_disable();
	else	LL_USART_TransmitData8( USART2, txbuf[txindex++] );
	}
if	(
	( LL_USART_IsActiveFlag_RXNE( USART2 ) ) &&
	( LL_USART_IsEnabledIT_RXNE( USART2 ) )
	)
	{
	#ifdef RX_FIFO
	rxbuf[(rxwi++)&(QRX-1)] = LL_USART_ReceiveData8( USART2 );
	#else
	cmd_handler( LL_USART_ReceiveData8( USART2 ) );
	#endif
	}
}

void cmd_handler( char c )
{
switch	( c )
	{
	case 't' :
		if	( etat.pos < 0 )
			audio_start( frein );
		break;
	case 'a' :
		snprintf( txbuf, sizeof(txbuf), "adc %d\n", adc_raw );
		txindex = 0;
		UART2_TX_INT_enable();
		break;
	case 'A' :
		opto_process();
		break;
	case 'd' :
		snprintf( txbuf, sizeof(txbuf), "acc1 = %d, demod %d (FS 524000)\n", acc1 >> LOG_TAU1, acc2 >> ( LOG_TAU2 - 8 ) );
		txindex = 0;
		UART2_TX_INT_enable();
		break;
	default :
		if	( c >= ' ' )
			snprintf( txbuf, sizeof(txbuf), "cmd \"%c\"\n", c );
		else	snprintf( txbuf, sizeof(txbuf), "cmd 0x%02x\n", c );
		txindex = 0;
		UART2_TX_INT_enable();
	}
}

int main(void)
{
// Configure the system clock to 64 or 72 MHz according to HSE_EXT
SystemClock_Config();

// config LED
gpio_init();


// config systick @ 100Hz

  // periode
  SysTick->LOAD  = (SystemCoreClock / 100) - 1;
  // priorite
  NVIC_SetPriority( SysTick_IRQn, 7 );
  // init counter
  SysTick->VAL = 0;
  // prescale (0 ===> %8)
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
  // enable timer, enable interrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

// config UART (interrupt handler doit etre pret!!)
gpio_uart2_init();
UART2_init( 9600 );
gpio_timer3_init();
TIM3_PWM_init( PWM_PERIOD );
adc_init();

 while (1)
 	{
	#ifdef GREEN_CPU
	SCB->SCR = 0;				// avoid deep sleep
	PWR->CR &= ~(PWR_CR_PDDS|PWR_CR_LPDS);	// avoid power down
	__WFI();				// Wait for Interrupt
	#endif
	if	( LL_ADC_IsActiveFlag_EOS(ADC1) ) PB12_PROFIL_0();
	else					  PB12_PROFIL_1();
 	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000 or 64000000
  *            HCLK(Hz)                       = 72000000 or 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

#ifdef HSE_EXT
#define HSE
#endif

#ifdef HSE
/* Enable HSE oscillator or bypass */
#ifdef HSE_EXT
LL_RCC_HSE_EnableBypass();	// pas de quartz ==> MCO du ST-Link
#endif
LL_RCC_HSE_Enable();
while(LL_RCC_HSE_IsReady() != 1)
  { }
#else
LL_RCC_HSI_Enable();
while(LL_RCC_HSI_IsReady() != 1)
  { }
#endif

/* Main PLL configuration and activation */
#ifdef HSE
LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
#else
// HSI est obligatoirement %2, donc avec MUL_16 qui est le max on a 64 MHz
LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
#endif

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
    { }

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    { }

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* Update SystemCoreClock variable */
  SystemCoreClockUpdate();
}
