/* test preliminaire passerelle pour multimetre AIMTTi 1604
 * PC -> uart2 -> uart1 -> 1604
 * bouton bleu -> 0x75 vers uart1 -> 1604 "connect"
 * 1604 -> uart1 -> bin2hex -> uart2 -> PC
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
// #if defined(USE_FULL_ASSERT)
// #include "stm32_assert.h"
// #endif /* USE_FULL_ASSERT */
#include "options.h"
#include "gpio.h"
#include "uarts.h"
#include "logfifo.h"
#include "spi.h"
#include <stdio.h>	// pour snprintf

void SystemClock_Config(void);
void cmd_handler( char c );


// HSE_EXT est pour utiliser une source d'horloge 8MHz externe
// si HSE_EXT : MCO de la sonde ST-LINK    8MHz -> PLL -> 72 MHz
// sinon      : oscillateur RC interne HSI 8MHz -> PLL -> 64 Mhz
// HSE_EXT ne marche pas sur une nucleo coupee ni sur Blue Pill

#define HSE_EXT
#define GREEN_CPU

// contexte global -----------------------------------------------------------

unsigned int cnt100Hz = 0;
volatile int blue=0, old_blue=0;

#ifdef RX_FIFO
// reception UART : fifo circulaire rudimentaire
#define QRX (1<<5)		// a power of 2 !!!
char urxbuf[QRX];
volatile unsigned int urxwi=0;	// write index
volatile unsigned int urxri=0;	// read index
// exemple de lecture du fifo  :
// 	while	( urxwi - urxri )
//		{
//		int c = urxbuf[(urxri++)&(QRX-1)];
//		... }
#endif

// systick interrupt handler
void SysTick_Handler()
{
++cnt100Hz;
switch	(cnt100Hz % 100)
	{
	case 0 :
	//case 10 :
		LED_ON();
		break;
	case 5 :
	//case 15 :
		LED_OFF();
		break;
	}
if	( ( blue ) && ( old_blue == 0 ) )
	{
	LL_USART_TransmitData8( USART1, 0x75 );	// AIMTTi "connect"
	}
old_blue = blue;
}

#ifdef USE_UART1
// UART1 interrupt handler, RX ONLY
void USART1_IRQHandler( void )
{
if	(
	( LL_USART_IsActiveFlag_RXNE( USART1 ) ) &&
	( LL_USART_IsEnabledIT_RXNE( USART1 ) )
	)
	{
	int c = LL_USART_ReceiveData8( USART1 );
	// LOGputc( c ); // echo
	LOGprint("0x%02x", c );
	}
}
#endif

#ifdef USE_UART2
// UART2 interrupt handler
void USART2_IRQHandler( void )
{
if	(
	( LL_USART_IsActiveFlag_TXE( USART2 ) ) &&
	( LL_USART_IsEnabledIT_TXE( USART2 ) )
	)
	{
	#ifdef USE_LOGFIFO
	if	( logfifo.rda == logfifo.wra )
		{			// rien a transmettre, attendre
		UART2_TX_INT_disable();
		}
	else	{
		int c;
		c = logfifo.circ[logfifo.rda++];
		if	( c == 0 )
			LL_USART_TransmitData8( USART2, '\n' );
		else	LL_USART_TransmitData8( USART2, c );
		logfifo.rda &= LFIFOMS;
		}
	#else
	LL_USART_TransmitData8( USART2, '?' );
	#endif
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
#endif

void cmd_handler( char c )
{
// reponse test pour CDC/USB
if	( c == '?' )
	{
	LOGprint("imposant" );
	return;
	}
else if	( c == '!' )
	{
	LOGflush(); return;
	}
else	{
	if	( c >= ' ' )
		{ LOGputc( c ); LOGputc(' '); }
	// else	LOGprint( " 0x%02x ", c );
	LL_USART_TransmitData8( USART1, c );
	}

}

#ifdef USE_LOGFIFO
// N.B. pour avoir la correspondance numero <--> perif , voir IRQn_Type
void report_interrupts(void)
{
int i;
unsigned int p;
p = __NVIC_GetPriorityGrouping();
LOGprint("priority grouping %d", p );
// special systick
i = -1;
if	(  SysTick->CTRL & SysTick_CTRL_TICKINT_Msk )
	{
	p = __NVIC_GetPriority((IRQn_Type)i);
	LOGprint("int #%2d, pri %d", i, p );
	}
// tous les autres
for	( i = 0; i <=  97; ++i )
	{
	if	( __NVIC_GetEnableIRQ((IRQn_Type)i) )
		{
		p = __NVIC_GetPriority((IRQn_Type)i);
		LOGprint("int #%2d, pri %d", i, p );
		}
	}
}
#endif

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
  NVIC_SetPriority( SysTick_IRQn, 11 );
  // init counter
  SysTick->VAL = 0;
  // prescale (0 ===> %8)
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
  // enable timer, enable interrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

#ifdef USE_UART2
// config UART (interrupt handler doit etre pret!!)
gpio_uart2_init();
UART2_init( 9600 );
logfifo_init();
report_interrupts();
#endif

#ifdef USE_UART2
// config UART (interrupt handler doit etre pret!!)
gpio_uart1_init();
UART1_init( 9600 );
#endif

while	(1)
 	{
	// taches de fond

	// gestion du sleep
	if	( BLUE_BUTTON() )
		{
		LED_ON(); blue = 1;
		}
	else	{
		blue = 0;
		#ifdef GREEN_CPU
		SCB->SCR = 0;				// avoid deep sleep
		PWR->CR &= ~(PWR_CR_PDDS|PWR_CR_LPDS);	// avoid power down
		__WFI();				// Wait for Interrupt
		#endif
		}
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
/* Enable HSE oscillator or bypass */
LL_RCC_HSE_EnableBypass();	// pas de quartz ==> MCO du ST-Link
LL_RCC_HSE_Enable();
while(LL_RCC_HSE_IsReady() != 1)
  {  };
#else
LL_RCC_HSI_Enable();
while(LL_RCC_HSI_IsReady() != 1)
  {  };
#endif

/* Main PLL configuration and activation */
#ifdef HSE_EXT
LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
#else
// HSI est obligatoirement %2, donc avec MUL_16 qui est le max on a 64 MHz
LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
#endif

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {  };

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* Update SystemCoreClock variable */
  SystemCoreClockUpdate();
}
