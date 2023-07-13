
/* Includes ------------------------------------------------------------------*/
#include "options.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"

#include "gpio.h"
#include "flashy.h"
#include "uarts.h"
#include <stdio.h>	// pour snprintf

#ifdef USE_LCD2x16
#include "LCD2x16.h"
#endif

void SystemClock_Config(void);
void cmd_handler( char c );

// contexte global -----------------------------------------------------------

unsigned int cnt100Hz = 0;

// emission : par message
// volatile int msg_request = 0;
char txbuf[64];
volatile int txindex;

#ifdef USE_NOKIA
#include "nokia.h"
volatile int LCDcontrast = 59;	// 40-60 is usually a pretty good range.
volatile int LCDbias = 3;	// theoretical is 4
char LCDbuf[64];
#endif

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
#else
volatile char rxbyte;
#endif

volatile unsigned int Avar = 1;

// systick interrupt handler
void SysTick_Handler()
{
++cnt100Hz;
// LED blinks
switch	( cnt100Hz % 100 )
	{
	case 0 :
		LED_ON();
		break;
	case 5 :
		LED_OFF();
		break;
	}
// log periodique
if	( ( cnt100Hz % 200 ) == 90 )
	{
	char lcdbuf[16];
	snprintf( lcdbuf, sizeof(lcdbuf), "%6d ", Avar);
	set_cursor( 0, 0 ); lcd_print(lcdbuf);
	}
}

// temporisation base sur systick
// unites en periodes d'horloge du timer
// tickd doit etre inferieur a (LOAD+1)/2
void tickdelay( unsigned int tickd )
{
int tper = SysTick->LOAD + 1;
int nextVAL, diff;
nextVAL = SysTick->VAL - tickd;
if	( nextVAL < 0 )
	nextVAL += tper;
do	{				// diff c'est le temps restant a attendre
	diff = SysTick->VAL - nextVAL;
	if	( diff <= -(tper/2) )	// on maintient diff entre -(tper/2) et (tper/2)
		diff = 1;
	else if ( diff > (tper/2) )
		break;
	} while ( diff > 0 );
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
	rxbyte = LL_USART_ReceiveData8( USART2 );
	cmd_handler( rxbyte );
	#endif
	}
}

#ifdef USE_UART3
char txbuf3[24];
volatile int txindex3;
#define FM_MAGIC1 0xA5
#define FM_MAGIC2 0xe6
char rxbuf3[16];
volatile int rxindex3 = 0;
volatile int FM_status = 0;

// UART3 interrupt handler
void USART3_IRQHandler( void )
{
if	(
	( LL_USART_IsActiveFlag_TXE( USART3 ) ) &&
	( LL_USART_IsEnabledIT_TXE( USART3 ) )
	)
	{	// messages de taille variable
	if	( txbuf3[txindex3] == 0 )
		{ UART3_TX_INT_disable(); Tx_cmd(0); }
	else	{
		Tx_cmd(1);
		LL_USART_TransmitData8( USART3, txbuf3[txindex3++] );
		}
	}
if	(
	( LL_USART_IsActiveFlag_RXNE( USART3 ) ) &&
	( LL_USART_IsEnabledIT_RXNE( USART3 ) )
	)
	{
	// FM Rx FSM
	//	0 : idle
	//	1 : FF received, waiting fo magic
	//	2 : magic1 received, waiting for magic2
	//	3 : magic2 received, accepting data
	//	8 : terminator received, waiting for ack
	//	9 : buffer full, waiting for ack
	char c = LL_USART_ReceiveData8( USART3 );
	switch	( FM_status )
		{
		case 0: if	( c == 0xFF )
				{ FM_status = 1; rxindex3 = 0; }
			break;
		case 1:	if	( c == FM_MAGIC1 )
				FM_status = 2;
			else if	( c != 0xFF )
				FM_status = 0;
			break;
		case 2:	if	( c == FM_MAGIC2 )
				FM_status = 3;
			else	FM_status = 0;
			break;
		case 3: if	( rxindex3 < sizeof(rxbuf3) )
				{
				if	( c <= ' ' )
					{ c = 0; FM_status = 8; }	// fin normale
				rxbuf3[rxindex3++] = c;
				}
			else	FM_status = 9;				// buffer full
			break;
		case 8: // nothing to do, app will reset FM_status
		case 9: // nothing to do, app will reset FM_status
			break;
		default: FM_status = 0;
		}
	// echo vers UART2 pour test
	// snprintf( txbuf, sizeof(txbuf), "/%c/", LL_USART_ReceiveData8( USART3 ) );
	// txindex = 0; UART2_TX_INT_enable();
	}
}

// message formatage for FM 433MHz
#define QFF 4
int FM_send( int N )
{
int i;
for	( i = 0; i < QFF; i++ )
	txbuf3[i] = 0xFF;
txbuf3[i++] = FM_MAGIC1;
txbuf3[i++] = FM_MAGIC2;
i += snprintf( txbuf3 + i, sizeof(txbuf3)-i, "%04x  ", N );
txindex3 = 0;
UART3_TX_INT_enable();
return i;
}
#endif

void cmd_handler( char c )
{
#ifdef USE_NOKIA
static int x = 0, y = 0;
#endif
switch	( c )
	{
	#ifdef USE_ADC
	case 'a' :
		snprintf( txbuf, sizeof(txbuf), "adc %d\n", adc_raw );
		txindex = 0;
		UART2_TX_INT_enable();
		break;
	#endif
	#ifdef USE_NOKIA
	case 'R' : NOKIA_RST_LO(); break;
	case 'r' : LcdInitialize(); break;
	case 'x' : x += 1; if ( x > 83 ) x = 0; break;
	case 'y' : y += 1; if ( y > 5 )  y = 0; break;
	case 'g' : LcdGotoXY( x, y ); break;
	case 'c' : LcdClear( 0 ); break;
	case 'z' : LcdClear( 0x54 ); break;
	case '>' : LcdSetContrast( ++LCDcontrast ); break;
	case '<' : LcdSetContrast( --LCDcontrast ); break;
	#ifdef USE_FLASHY
	case 'W' : {
		unsigned short nokcfg1;
		nokcfg1 = LCDcontrast | ( LCDbias << 8 );
		flashy_unlock();
		flashy_page_erase( LAST_FLASH_PAGE );	// derniere page
		flashy_write_short( LAST_FLASH_PAGE,    nokcfg1 );
		flashy_write_short( LAST_FLASH_PAGE+2, ~nokcfg1 );
		flashy_relock();
		} break;
	case 'v' : {
		unsigned int nokcfg = *((unsigned int *)LAST_FLASH_PAGE);
		snprintf( txbuf, sizeof(txbuf), "nokcfg=%08x\n", nokcfg );
		txindex = 0; UART2_TX_INT_enable();
		} break;
	case 'V' : {
		unsigned short nokcfg1, nokcfg2;
		nokcfg1 = ((__IO uint16_t*)LAST_FLASH_PAGE)[0];
		nokcfg2 = ((__IO uint16_t*)LAST_FLASH_PAGE)[1];
		if	( nokcfg2 == ( ~nokcfg1 & 0xFFFF ) )
			snprintf( txbuf, sizeof(txbuf), "verif n=%d V=%03d\n",LCDbias, LCDcontrast );
		else	snprintf( txbuf, sizeof(txbuf), "err %04x %04x\n", ~nokcfg1, nokcfg2 );
		txindex = 0; UART2_TX_INT_enable();
		} break;
	#endif
	case 'n' : LcdNegativeImage(1); break;
	case 'p' : LcdNegativeImage(0); break;
	case '0' : LcdWrite( LCD_D, 0 ); break;
	case '|' : LcdWrite( LCD_D, 0xFF ); break;
	case '!' :
		snprintf( LCDbuf, sizeof(LCDbuf), "n=%d V=%03d   ", LCDbias, LCDcontrast );
		LcdString( LCDbuf, 1 );
		break;
	default :
	if	( ( c >= '1' ) && ( c <= '7' ) )
		{
		LCDbias = c - '0';
		LcdSetBias( LCDbias );
		}
	#endif
	}
if	( !LL_USART_IsEnabledIT_TXE( USART2 ) )
	{
	#ifdef USE_NOKIA
	snprintf( txbuf, sizeof(txbuf), "%c n=%d V=%03d [%2d:%d]\n", ((c>=' ')?(c):('?')), LCDbias, LCDcontrast, x, y );
	txindex = 0; UART2_TX_INT_enable();
	#else
	switch	( c )
		{
		case '1' :
			LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOC );
			lcd_init();
			break;
		case '2' :
			lcd_clear();
			break;
		case '3' :
			set_cursor( 1, 1 );
			lcd_print("hello");
			break;
		case 'T' : Rx_cmd(0); Tx_cmd(1); break;
		case 'R' : Tx_cmd(0); Rx_cmd(1); break;
		case 'S' : Rx_cmd(0); Tx_cmd(0); break;
		case 'A' : FM_send( Avar++ ); break;
		default:	// simple echo
			snprintf( txbuf, sizeof(txbuf), "%c\n", ((c>=' ')?(c):('?')) );
			txindex = 0; UART2_TX_INT_enable();
		}
	#endif
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

#ifdef USE_UART3
UART3_init( 9600 );
gpio_uart3_init();
#endif

#ifdef USE_PWM
#include "stm32f1xx_ll_tim.h"
// #include "pwm.h"
gpio_timer3_init();
TIM3_PWM_init( PWM_PERIOD );
#endif

#ifdef USE_ADC
#include "stm32f1xx_ll_adc.h"
adc_init();
#endif

#ifdef USE_NOKIA
gpio_nokia_init();
LcdInitialize();
LcdClear( 0x55 );
#ifdef USE_FLASHY
unsigned short nokcfg1, nokcfg2;
nokcfg1 = ((__IO uint16_t*)LAST_FLASH_PAGE)[0];
nokcfg2 = ((__IO uint16_t*)LAST_FLASH_PAGE)[1];
if	( nokcfg2 == ( ~nokcfg1 & 0xFFFF ) )
	{
	LCDcontrast = nokcfg1 & 0x7F;	// 40-60 is usually a pretty good range.
	LCDbias     = nokcfg1 >> 8;
	}
#else
LCDcontrast = 59;	// 40-60 is usually a pretty good range.
LCDbias = 3;
#endif
LcdSetContrast( LCDcontrast );
LcdSetBias( LCDbias );
LcdGotoXY( 0, 0 );		// 123456789abc123456789abc
snprintf( LCDbuf, sizeof(LCDbuf), "C'est ...   imposant !!!" );
LcdString( LCDbuf, 1 );		// 123456789abcde
snprintf( LCDbuf, sizeof(LCDbuf), "C'est imposant" );
LcdString( LCDbuf, 0 );
LcdGotoXY( 4 * 12, 5 ); LcdString( "1527", 1 );
LcdString2( 0, 4, "1527" );
#endif

#ifdef USE_LCD2x16
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOC );
lcd_init();	// init ne clear pas !
lcd_clear();
set_cursor( 0, 0 ); lcd_print("C'est IMPOSANT");
set_cursor( 6, 1 ); lcd_print("vrai!");
#endif

// LA GROSSE BOUCLE MAIN LOOP
while (1)
 	{
	#ifdef GREEN_CPU
	SCB->SCR = 0;				// avoid deep sleep
	PWR->CR &= ~(PWR_CR_PDDS|PWR_CR_LPDS);	// avoid power down
	__WFI();				// Wait for Interrupt
	#endif
	#ifdef prof_EOS
	if	( LL_ADC_IsActiveFlag_EOS(ADC1) ) PB12_PROFIL_0();
	else					  PB12_PROFIL_1();
	#endif
	#ifdef PROF_PB12
	PB12_PROFIL_1();
	tickdelay( Avar );
	PB12_PROFIL_0();
	tickdelay( Avar );
	#endif
	if	( ( FM_status == 8 ) || ( FM_status == 9 ) )
		{
		set_cursor( 0, 1 ); lcd_print("----------------");
		rxbuf3[sizeof(rxbuf3)-1] = 0;
		set_cursor( 0, 1 ); lcd_print( rxbuf3 );
		FM_status = 0;
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
