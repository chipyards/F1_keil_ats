/* prog pour evaluer Qfplib-M3 : soft float maths
 */

#include "options.h"
#ifdef MAIN_GENERIC
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"

#include "qfplib-m3.h"

#include "sys.h"
#include "gpio.h"
#include "flashy.h"
#include "uarts.h"
#include "CDC.h"
#include <stdio.h>	// pour snprintf
#include "skysplit.h"
#include "CC1101.h"

#ifdef USE_LCD2x16
#include "LCD2x16.h"
#endif

#ifdef USE_ADC_4CH
#include "stm32f1xx_ll_adc.h"
#include "adc.h"
#endif

#ifdef USE_UART3_FM
#include "fm.h"
#endif

void cmd_handler( char c );

// contexte global -----------------------------------------------------------

volatile unsigned int cnt100Hz = 0;
volatile unsigned int cnt1Hz = 0;
volatile unsigned int cntblinks = 1;

#ifdef USE_NOKIA
#include "nokia.h"
volatile int LCDcontrast = 59;	// 40-60 is usually a pretty good range.
volatile int LCDbias = 3;	// theoretical is 4
char LCDbuf[64];
#endif

#ifdef USE_UART3_FM
volatile unsigned int Avar = 1;
#endif

#ifdef USE_LCD2x16
int autoTx = 0;
#else
int autoTx = 1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
// systick interrupt handler
void SysTick_Handler()
{
++cnt100Hz;
#ifndef NUCLEO	// sur nucleo la LED est masquee par SCK de SPI1
	{	// 1 to 5 LED blinks per second
	switch	( cnt100Hz % 100 )
		{
		case 0 : ++cnt1Hz; if ( cntblinks ) LED_ON();
			break;
		case 10 : if ( cntblinks > 1 ) LED_ON();
			break;
		case 20 : if ( cntblinks > 2 ) LED_ON();
			break;
		case 30 : if ( cntblinks > 3 ) LED_ON();
			break;
		case 40 : if ( cntblinks > 4 ) LED_ON();
			break;
		case 4 :
		case 14 :
		case 24 :
		case 34 :
		case 44 : LED_OFF();
			break;
		}
	}
#else
if	( ( cnt100Hz % 100 ) == 0 )
	++cnt1Hz;
#endif
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
		CDC_printf("npvhdd %d %d %d %d %d %d\n", adc1_res0/CHANFIR, adc2_res0/CHANFIR, adc1_res1/CHANFIR, adc2_res1/CHANFIR,
			  ( (int)adc2_res0 - (int)adc1_res0 )/CHANFIR, ( (int)adc2_res1 - (int)adc2_res0 )/CHANFIR );
		adc_res_ready = 0;
		}
	else	{
		CDC_printf("adc res not ready\n");
		}
	}
#endif
#endif
}
#ifdef __cplusplus
}
#endif


#ifdef USE_CDC
int CDC_debug_mode = 1;
// attention cette fonction ne doit pas etre appelee depuis une interruption (CDC_printf n'est pas thread-safe !)
void cmd_handler( char c )
{
#ifdef USE_NOKIA
static int x = 0, y = 0;
#endif
switch	( c )
	{
	#ifdef USE_ADC
	case 'a' :
		CDC_printf( "adc %d\n", adc_raw );
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
		CDC_printf("nokcfg=%08x\n", nokcfg );
		} break;
	case 'V' : {
		unsigned short nokcfg1, nokcfg2;
		nokcfg1 = ((__IO uint16_t*)LAST_FLASH_PAGE)[0];
		nokcfg2 = ((__IO uint16_t*)LAST_FLASH_PAGE)[1];
		if	( nokcfg2 == ( ~nokcfg1 & 0xFFFF ) )
			CDC_printf("verif n=%d V=%03d\n",LCDbias, LCDcontrast );
		else	CDC_printf("err %04x %04x\n", ~nokcfg1, nokcfg2 );

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
	case '?' :
		CDC_printf("%c n=%d V=%03d [%2d:%d]\n", ((c>=' ')?(c):('?')), LCDbias, LCDcontrast, x, y );
		break;
	default :
	if	( ( c >= '1' ) && ( c <= '7' ) )
		{
		LCDbias = c - '0';
		LcdSetBias( LCDbias );
		}
	#endif
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
		CDC_printf("NPVHdd %d %d %d %d %d %d\n", adc1_res0, adc2_res0, adc1_res1, adc2_res1,
			  (int)adc2_res0 - (int)adc1_res0, (int)adc2_res1 - (int)adc2_res0 );
		break;
	case 'y' :
		CDC_printf("npvhdd %d %d %d %d %d %d\n", adc1_res0/CHANFIR, adc2_res0/CHANFIR, adc1_res1/CHANFIR, adc2_res1/CHANFIR,
			  ( (int)adc2_res0 - (int)adc1_res0 )/CHANFIR, ( (int)adc2_res1 - (int)adc2_res0 )/CHANFIR );
		break;
	case 'h' :
		adc_timer_stop();
		CDC_printf("ADC interrupt halted\n" );
		break;
	case 'c' :
		adc_calib();
		CDC_printf("calib. done\n" );
		break;
	case 'u' :
		adc_uncalib();
		CDC_printf("calib. erased\n" );
		break;
	case 'r' :
		adc_timer_init( SystemCoreClock / 2000 );	// 2 kHz ==> 1 ksamp/s pour chaque canal avant FIR );
		CDC_printf("ADC interrupt restarted\n" );
		break;
	case 't' :
		adc_start_conv();
		CDC_printf("test conversion started\n" );
		break;
	case 'd' :
		CDC_printf("test conversion %lu %lu\n", ADC1->DR, ADC2->DR );
		break;
	#endif
	case '$' :
		report_interrupts();
		break;
	default:
		#ifdef USE_CC1101
		CC.demo(c);
		#else	// simple echo
		CDC_printf("%c\n", ((c>=' ')?(c):('?')) );
		#endif
	}
}
#endif

int main(void)
{
// Configure the system clock to 8 or 64 or 72 MHz according to options.h
SystemClock_Config();

// mode CW avec jumper A2-A3 pour Blue Pill, peu compatible avec NUCLEO
#ifndef USE_NUCLEO
CC.CW_tx_enable = ( gpio_test_jmpA23() );
#endif

gpio_init();

// config systick @ 100Hz
systick_init( 100 );

#ifdef USE_CDC
// config UART (interrupt handler doit etre pret!!)
gpio_uart2_init();
UART2_init( 9600 );
CDC_init();
#endif

#ifdef USE_CC1101
gpio_spi1_init();
SPI1_init();
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
 	static unsigned int old1Hz = 0;
 	if	(  ( old1Hz != cnt1Hz ) )
 		{
 		old1Hz = cnt1Hz;
		// do something exactly once per second
		#ifdef SIMPLE_BEACON
		if	( cnt1Hz == 11 )
			{
			cntblinks = 5;	// pour le cas ou SPI planterait dans while ( IS_MISO_SET() ), i.e. si transceiver absent
			if	( CC.CW_tx_enable )
				cntblinks = 3 + CC.simple_CW_init(); // 3 blink si Ok, sinon 4
			else	{
				cntblinks = 1 + CC.simple_radio_init(); // 1 blink si Ok, sinon 2
				CC.AAR_tx_enable = 1;
				}
			}
		else if	( ( cnt1Hz > 11 ) && ( CC.AAR_tx_enable ) )
			{
			// CC.simple_beacon_tx( cnt1Hz );
			lepilot.AAR_tx();
			}
		#endif
		}
	if	( IS_GDO0_SET() )
		CC.handle_rx_to_CDC();
	#ifdef GREEN_CPU
	if	( cnt100Hz < (10*100) )
		LED_ON();	// continuous light indicating safe to debug
	else	{
		// 		// LED blinks (SysTick_Handler() driven)
		SCB->SCR = 0;				// avoid deep sleep
		PWR->CR &= ~(PWR_CR_PDDS|PWR_CR_LPDS);	// avoid power down
		__WFI();	// Wait for Interrupt
		}
	#endif
	#ifdef USE_CDC
	int c;
	if	( ( c = CDC_getcmd() ) > 0 )
		{
		if	( c == '#' )
			{
			CDC_debug_mode = !CDC_debug_mode;
			CDC_printf("debug_mode %d\n", CDC_debug_mode );
			}
		if	( CDC_debug_mode )
			cmd_handler( c );
		else	lepilot.cmd_handler(c);
		}
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
				CDC_printf("bin Ok %02X%02X%02X%02X ", rxbuf3[4], rxbuf3[3], rxbuf3[2], rxbuf3[1] );
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
				CDC_printf("opcode 0x%02X, ?", rxbuf3[0] );
				#endif
				}
			}
		else if	( op == OP_ASC )
			{
			if	( sz > 14 )	// taille limitee a 14 en ascii pour pouvoir AJOUTER le null terminator
				sz = 14;	// c'est juste pour la commodite de l'affichage
			rxbuf3[sz+1] = 0;	// rxbuf3 contient l'opcode suivi de la payload
			#ifdef USE_CDC
			CDC_printf("asc Ok %d %s\n", rxbuf3[0] & 0x0F, rxbuf3+1 ); // size puis payload, sans l'opcode
			#endif
			#ifdef USE_LCD2x16
			  set_cursor( 0, 1 ); lcd_print("----------------");
			  set_cursor( 0, 1 ); lcd_print( rxbuf3+1 );
			#endif
			}
		else	{
			#ifdef USE_CDC
			CDC_printf("opcode 0x%02X, ?", rxbuf3[0] );
			#endif
			}
			#ifdef USE_CDC

			#endif
		rx_status = 0;
		}
	else if	( ( rx_status == 20 ) || ( rx_status == 21 ) || ( rx_status == 22 ) )
		{
		#ifdef USE_CDC
		CDC_printf("Bad %d\n", rx_status );
		#endif
		#ifdef USE_LCD2x16
		  set_cursor( 0, 0 ); lcd_print("::::::::::::::::");
		  set_cursor( 2, 0 ); lcd_print( txbuf2 );
		#endif
		rx_status = 0;
		}
	#endif
 	} // while (1)
}	// main
#endif
