/* cette demo SPI slave - CDC fait la chose suivante :
	pour chaque char arrive du PC :
		echo vers le PC sauf '?' et '!'
		le char copie dans le fifo tx du SPI
	pour chaque bloc arrive en SPI
		le fifo tx rendu au master, complete avec des SPI_NO_DATA si necessaire
		le contenu du fifo rx mis dans un message pour le PC, dont l'emission
		est declenchee par le terminateur ';' (non emis) 
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
#include "gpio.h"
#include "uarts.h"
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

// emission UART : par message
char utxbuf[128];
volatile unsigned int utxwi=0;	// write index
volatile unsigned int utxri=0;	// read index

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

// emission UART : pour emettre et vider le contenu du buffer
void uflush(void)
{
utxri = 0;
UART2_TX_INT_enable();
}

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
}

// UART2 interrupt handler
void USART2_IRQHandler( void )
{
if	(
	( LL_USART_IsActiveFlag_TXE( USART2 ) ) &&
	( LL_USART_IsEnabledIT_TXE( USART2 ) )
	)
	{	// messages de taille variable utxwi
	if	( utxri >= utxwi )
		{ UART2_TX_INT_disable(); utxwi = 0; utxri = 0; }
	else	LL_USART_TransmitData8( USART2, utxbuf[utxri++] );
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

extern unsigned int spi2_rx_wi;	// index egaux <==> fifo vide
extern unsigned int spi2_rx_ri;

void cmd_handler( char c )
{
// reponse test pour CDC/USB
if	( c == '?' )
	{
	utxwi = snprintf( utxbuf, sizeof(utxbuf), "spi2 : rx_wi=%d rx_ri=%d\n", spi2_rx_wi, spi2_rx_ri );
	if	( utxwi > sizeof(utxbuf) )
		utxwi = sizeof(utxbuf);		// snprintf ne deborde pas mais il rend la taille avant trim...
	uflush(); return;
	}
else if	( c == '!' )
	{
	uflush(); return;
	}
else	{
	if	( c >= ' ' )
		utxwi = snprintf( utxbuf, sizeof(utxbuf), "%c", c );
	else	utxwi = snprintf( utxbuf, sizeof(utxbuf), " 0x%02x ", c );
	}
uflush();
// reponse test pour SPI/RASPI : le char est mis dans le fifo tel quel
spi2_put8( c );
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
// config SPI2 slave
gpio_spi2_slave_init();
spi2_fifo_init();
spi_slave_init( SPI2, 6 );


 while (1)
 	{
	// taches de fond
	if	( !( LL_USART_IsEnabledIT_TXE( USART2 ) ) )	// protection rudimentaire...
		{
		int c;
		c = spi2_get8();
		if	( c >= 0 )		// assembler les bytes venus de spi2
			{
			if	( c == ';' )
				uflush();	// les envoyer vers CDC/USB
			else if	( utxwi < sizeof(utxbuf) )
				utxbuf[utxwi++] = c;
			}
		}

	// gestion du sleep
	if	( BLUE_BUTTON() )
		{//LED_ON();
		}
	#ifdef GREEN_CPU
	else	{
		SCB->SCR = 0;				// avoid deep sleep
		PWR->CR &= ~(PWR_CR_PDDS|PWR_CR_LPDS);	// avoid power down
		__WFI();				// Wait for Interrupt
		}
	#endif
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
