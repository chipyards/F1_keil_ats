/* prog pour emettre ou recevoir des messages en FM 433 MHz, taille variable, pas de delimiteur, contenu arbitraire
- emission :
	- payload de test : op 0x00 : 32 bit en binaire, op 0x10 : 32 bits hex en ascii (0 a 8 digits) ou texte <= 14 char
	- emission periodique (3s) si autoTx = 1, ou emission manuelle via CDC
	  compiler sans USE_LCD2x16 demarre avec autoTx = 1 pour tester un emetteur autonome
- reception :
	- reset avec bouton bleu ou compiler avec USE_LCD2x16 : reception permanente meme pendant emission
	  ==> relecture possible (works ok)
	- message transfere vers CDC (ascii) et LCD2x16 si existe
 */
/* Includes ------------------------------------------------------------------*/
#include "options.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_adc.h"

#include "sys.h"
#include "gpio.h"
#include "flashy.h"
#include "uarts.h"
#include <stdio.h>	// pour snprintf

#ifdef USE_LCD2x16
#include "LCD2x16.h"
#endif

#ifdef USE_ADC_4CH
#include "adc.h"
#endif

#ifdef USE_UART3_FM
#include "fm.h"
#endif

void SystemClock_Config(void);
void cmd_handler( char c );

// contexte global -----------------------------------------------------------

unsigned int cnt100Hz = 0;

#ifdef USE_NOKIA
#include "nokia.h"
volatile int LCDcontrast = 59;	// 40-60 is usually a pretty good range.
volatile int LCDbias = 3;	// theoretical is 4
char LCDbuf[64];
#endif

#ifdef USE_CDC
// emission sur CDC : par message
char txbuf2[64];
volatile int txindex2;
// reception CDC : fifo circulaire
#ifdef RX_FIFO
#define QRX 32		// a power of 2 !!!
char rxbuf2[QRX];
volatile unsigned int rxwi2=0;	// write index
volatile unsigned int rxri2=0;	// read index
// exemple de lecture du fifo  :
// 	while	( rxwi2 - rxri2 )
//		{
//		int c = rxbuf2[(rxri2++)&(QRX-1)];
//		... }
#else
volatile char rxbyte2;
#endif
#endif

volatile unsigned int Avar = 1;

#ifdef USE_LCD2x16
int autoTx = 0;
#else
int autoTx = 1;
#endif

// systick interrupt handler
void SysTick_Handler()
{
++cnt100Hz;
/* LED blinks
	{
	switch	( cnt100Hz % 100 )
		{
		case 0 :
			LED_ON();
			break;
		case 5 :
			LED_OFF();
			break;
		}
	}
*/
// log periodique
#ifdef USE_UART3_FM
if	( autoTx )
	{
	if	( ( cnt100Hz % 600 ) == 90 )
		{		// message binaire
		LED_ON();
		FM_send( OP_BIN | 4, (unsigned char *)&Avar );
		}
	if	( ( cnt100Hz % 600 ) == 390 )
		{		// message ascii (hex)
		char tbuf[12];
		LED_ON();
		int size = snprintf( tbuf, sizeof(tbuf), "%X", Avar );
		if	( Avar == 0 )
			size = 0;
		FM_send( OP_ASC | size, (unsigned char *)tbuf );
		if	( Avar <= 0x20 )
			Avar++;
		else	Avar <<= 1;
		}
	if	( ( cnt100Hz % 300 ) == 150 )
		LED_OFF();
	}

#endif
#ifdef USE_ADC_4CH
#ifdef USE_CDC
if	( ( autoTx ) && ( ( cnt100Hz % 100 ) == 10 ) )
	{
	if	( adc_res_ready == 2 )
		{
		snprintf( txbuf2, sizeof(txbuf2), "npvhdd %d %d %d %d %d %d\n", adc1_res0/CHANFIR, adc2_res0/CHANFIR, adc1_res1/CHANFIR, adc2_res1/CHANFIR,
			  ( (int)adc2_res0 - (int)adc1_res0 )/CHANFIR, ( (int)adc2_res1 - (int)adc2_res0 )/CHANFIR );
		adc_res_ready = 0;
		txindex2 = 0; UART2_TX_INT_enable();
		}
	else	{
		snprintf( txbuf2, sizeof(txbuf2), "adc res not ready\n");
		txindex2 = 0; UART2_TX_INT_enable();
		}
	}
#endif
#endif
}


#ifdef USE_CDC
// UART2 (CDC) interrupt handler
void USART2_IRQHandler( void )
{
if	(
	( LL_USART_IsActiveFlag_TXE( USART2 ) ) &&
	( LL_USART_IsEnabledIT_TXE( USART2 ) )
	)
	{	// messages de taille variable
	if	( txbuf2[txindex2] == 0 )
		UART2_TX_INT_disable();
	else	LL_USART_TransmitData8( USART2, txbuf2[txindex2++] );
	}
if	(
	( LL_USART_IsActiveFlag_RXNE( USART2 ) ) &&
	( LL_USART_IsEnabledIT_RXNE( USART2 ) )
	)
	{
	#ifdef RX_FIFO
	rxbuf2[(rxwi2++)&(QRX-1)] = LL_USART_ReceiveData8( USART2 );
	#else
	rxbyte2 = LL_USART_ReceiveData8( USART2 );
	cmd_handler( rxbyte2 );
	#endif
	}
}

void cmd_handler( char c )
{
#ifdef USE_NOKIA
static int x = 0, y = 0;
#endif
switch	( c )
	{
	#ifdef USE_ADC
	case 'a' :
		snprintf( txbuf2, sizeof(txbuf2), "adc %d\n", adc_raw );
		txindex2 = 0;
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
		snprintf( txbuf2, sizeof(txbuf2), "nokcfg=%08x\n", nokcfg );
		txindex2 = 0; UART2_TX_INT_enable();
		} break;
	case 'V' : {
		unsigned short nokcfg1, nokcfg2;
		nokcfg1 = ((__IO uint16_t*)LAST_FLASH_PAGE)[0];
		nokcfg2 = ((__IO uint16_t*)LAST_FLASH_PAGE)[1];
		if	( nokcfg2 == ( ~nokcfg1 & 0xFFFF ) )
			snprintf( txbuf2, sizeof(txbuf2), "verif n=%d V=%03d\n",LCDbias, LCDcontrast );
		else	snprintf( txbuf2, sizeof(txbuf2), "err %04x %04x\n", ~nokcfg1, nokcfg2 );
		txindex2 = 0; UART2_TX_INT_enable();
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
	snprintf( txbuf2, sizeof(txbuf2), "%c n=%d V=%03d [%2d:%d]\n", ((c>=' ')?(c):('?')), LCDbias, LCDcontrast, x, y );
	txindex2 = 0; UART2_TX_INT_enable();
	#else
	switch	( c )
		{
		#ifdef USE_LCD2x16
		case '1' :
			LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOC );
			lcd_init();
			break;
		case '2' :
			lcd_clear();
			break;
		case '3' :
			set_cursor( 1, 1 );
			lcd_print("hello ");
			break;
		#endif
		#ifdef USE_UART3_FM
		case 'T' : Rx_cmd(0); Tx_cmd(1); break;
		case 'R' : Tx_cmd(0); Rx_cmd(1); break;
		case 'S' : Rx_cmd(0); Tx_cmd(0); autoTx = 0; break;
		case 'e' : {
			   char tbuf[20];			//   123456789012345
			   int size = snprintf( tbuf, sizeof(tbuf), "C'est imposant" );
			   FM_send( OP_ASC | size, (unsigned char *)tbuf );
			   } break;
		case 'f' : {
			   char tbuf[20];			//   123456789012345
			   int size = snprintf( tbuf, sizeof(tbuf), "C'est imposant!" );
			   FM_send( OP_ASC | size, (unsigned char *)tbuf );
			   } break;
		case 'A' : autoTx = 1; break;
		#endif
		#ifdef USE_ADC_4CH
		case 'x' :
			snprintf( txbuf2, sizeof(txbuf2), "NPVHdd %d %d %d %d %d %d\n", adc1_res0, adc2_res0, adc1_res1, adc2_res1,
				  (int)adc2_res0 - (int)adc1_res0, (int)adc2_res1 - (int)adc2_res0 );
			txindex2 = 0; UART2_TX_INT_enable();
			break;
		case 'y' :
			snprintf( txbuf2, sizeof(txbuf2), "npvhdd %d %d %d %d %d %d\n", adc1_res0/CHANFIR, adc2_res0/CHANFIR, adc1_res1/CHANFIR, adc2_res1/CHANFIR,
				  ( (int)adc2_res0 - (int)adc1_res0 )/CHANFIR, ( (int)adc2_res1 - (int)adc2_res0 )/CHANFIR );
			txindex2 = 0; UART2_TX_INT_enable();
			break;
		case 'h' :
			adc_timer_stop();
			snprintf( txbuf2, sizeof(txbuf2), "ADC interrupt halted\n" );
			txindex2 = 0; UART2_TX_INT_enable();
			break;
		case 'c' :
			adc_calib();
			snprintf( txbuf2, sizeof(txbuf2), "calib. done\n" );
			txindex2 = 0; UART2_TX_INT_enable();
			break;
		case 'u' :
			adc_uncalib();
			snprintf( txbuf2, sizeof(txbuf2), "calib. erased\n" );
			txindex2 = 0; UART2_TX_INT_enable();
			break;
		case 'r' :
			adc_timer_init( SystemCoreClock / 2000 );	// 2 kHz ==> 1 ksamp/s pour chaque canal avant FIR );
			snprintf( txbuf2, sizeof(txbuf2), "ADC interrupt restarted\n" );
			txindex2 = 0; UART2_TX_INT_enable();
			break;
		case 't' :
			adc_start_conv();
			snprintf( txbuf2, sizeof(txbuf2), "test conversion started\n" );
			txindex2 = 0; UART2_TX_INT_enable();
			break;
		case 'd' :
			snprintf( txbuf2, sizeof(txbuf2), "test conversion %lu %lu\n", ADC1->DR, ADC2->DR );
			txindex2 = 0; UART2_TX_INT_enable();
			break;
		#endif
		case '$' :
			report_interrupts( txbuf2, sizeof(txbuf2) );
			txindex2 = 0; UART2_TX_INT_enable();
			break;
		default:	// simple echo
			snprintf( txbuf2, sizeof(txbuf2), "%c\n", ((c>=' ')?(c):('?')) );
			txindex2 = 0; UART2_TX_INT_enable();
		}
	#endif
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
systick_init( 100 );

#ifdef USE_CDC
// config UART (interrupt handler doit etre pret!!)
gpio_uart2_init();
UART2_init( 9600 );
#endif

#ifdef USE_UART3_FM
UART3_init( 9600 );
gpio_uart3_init();
#ifdef NUCLEO
if	( BLUE_PRESS() )
	Rx_cmd(1);
#endif
#ifdef USE_LCD2x16
Rx_cmd(1);
#endif
#endif

#ifdef USE_PWM
#include "stm32f1xx_ll_tim.h"
// #include "pwm.h"
gpio_timer3_init();
TIM3_PWM_init( PWM_PERIOD );
#endif

#ifdef USE_ADC_4CH
// 2 ADCs
adc_init();

  #ifdef PROF_PB12
  PB12_PROFIL_1();
  #endif

// tempo 10 us @ 72 MHz
tickdelay( 72 * 10 );

  #ifdef PROF_PB12
  PB12_PROFIL_0();
  #endif

// la calibration
adc_calib();

  #ifdef PROF_PB12
  PB12_PROFIL_1();
  #endif

// tempo min 2 cycles
tickdelay( 8 * 3 );	// 3 cycles ADC ne durent pas plus que 8 Tck puisque le diviseur max est 8

  #ifdef PROF_PB12
  PB12_PROFIL_0();
  #endif

// configurer le timer TIM3 en timebase (pour interrupts seulement)
adc_timer_init( SystemCoreClock / 2000 );	// 2 kHz ==> 1 ksamp/s pour chaque canal avant FIR
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
	if	( cnt100Hz > 3000 )
		{
		LED_OFF();
		SCB->SCR = 0;				// avoid deep sleep
		PWR->CR &= ~(PWR_CR_PDDS|PWR_CR_LPDS);	// avoid power down
		__WFI();	// Wait for Interrupt
		}
	else	LED_ON();
	#endif
	#ifdef PROF_PB12_EOS
	if	( LL_ADC_IsActiveFlag_EOS(ADC1) ) PB12_PROFIL_0();
	else					  PB12_PROFIL_1();
	#endif
	#ifdef USE_UART3_FM
	if	( rx_status == 10 )
		{
		int op = rxbuf3[0] & 0xF0;
		int sz = rxbuf3[0] & 0x0F;
		if	( op == OP_BIN )
			{
			if	( sz == 4 )
				{
				#ifdef USE_CDC
				snprintf( txbuf2, sizeof(txbuf2), "bin Ok %02X%02X%02X%02X ", rxbuf3[4], rxbuf3[3], rxbuf3[2], rxbuf3[1] );
				#endif
				#ifdef USE_LCD2x16
				  char lcdbuf[20];
				  snprintf( lcdbuf, sizeof(lcdbuf), "bin Ok %02X%02X%02X%02X", rxbuf3[4], rxbuf3[3], rxbuf3[2], rxbuf3[1] );
				  set_cursor( 0, 0 ); lcd_print("----------------");
				  set_cursor( 0, 0 ); lcd_print( lcdbuf );
				#endif
				}
			else	{
				#ifdef USE_CDC
				snprintf( txbuf2, sizeof(txbuf2), "opcode 0x%02X, ?", rxbuf3[0] );
				#endif
				}
			}
		else if	( op == OP_ASC )
			{
			if	( sz > 14 )	// taille limitee a 14 en ascii pour pouvoir AJOUTER le null terminator
				sz = 14;	// c'est juste pour la commodite de l'affichage
			rxbuf3[sz+1] = 0;	// rxbuf3 contient l'opcode suivi de la payload
			#ifdef USE_CDC
			snprintf( txbuf2, sizeof(txbuf2), "asc Ok %d %s\n", rxbuf3[0] & 0x0F, rxbuf3+1 ); // size puis payload, sans l'opcode
			#endif
			#ifdef USE_LCD2x16
			  set_cursor( 0, 1 ); lcd_print("----------------");
			  set_cursor( 0, 1 ); lcd_print( rxbuf3+1 );
			#endif
			}
		else	{
			#ifdef USE_CDC
			snprintf( txbuf2, sizeof(txbuf2), "opcode 0x%02X, ?", rxbuf3[0] );
			#endif
			}
			#ifdef USE_CDC
			txindex2 = 0; UART2_TX_INT_enable();
			#endif
		rx_status = 0;
		}
	else if	( ( rx_status == 20 ) || ( rx_status == 21 ) || ( rx_status == 22 ) )
		{
		#ifdef USE_CDC
		snprintf( txbuf2, sizeof(txbuf2), "Bad %d\n", rx_status );
		txindex2 = 0; UART2_TX_INT_enable();
		#endif
		#ifdef USE_LCD2x16
		  set_cursor( 0, 0 ); lcd_print("::::::::::::::::");
		  set_cursor( 2, 0 ); lcd_print( txbuf2 );
		#endif
		rx_status = 0;
		}
	#endif
 	}
}

