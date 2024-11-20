/* prog pour afficher des reports recus en FM 433 MHz,
	donnees transferees vers CDC (ascii) et LCD2x16 si existe
 */
#include "options.h"
#ifdef MAIN_TERMINAL
/* Includes ------------------------------------------------------------------*/
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

#ifdef USE_UART3_FM
#include "fm.h"
#endif

void SystemClock_Config(void);
void cmd_handler( char c );

// contexte global -----------------------------------------------------------

unsigned int cnt100Hz = 0;
unsigned int amp_timout100Hz;


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

#ifdef USE_UART1
#define QRX1 1024		// a power of 2 !!!
char rxbuf1[QRX1];
volatile unsigned int rxwi1=0;	// write index
volatile unsigned int rxri1=0;	// read index
// exemple de lecture du fifo  :
// 	while	( rxwi1 - rxri1 )
//		{
//		int c = rxbuf1[(rxri1++)&(QRX-1)];
//		... }
#endif

#ifdef USE_GPS
#include "nmea.h"
nmea_ctx gps_ctx;		// we allocate the context here, it will have a pointer to the variables
#define NMEA_CTX (&gps_ctx)
#endif

// systick interrupt handler
void SysTick_Handler()
{
++cnt100Hz;
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
amp_timout100Hz++;
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
		case 'S' : Rx_cmd(0); Tx_cmd(0); break;
		case 'e' : {
			   char tbuf[20];			//   123456789012345
			   int size = snprintf( tbuf, sizeof(tbuf), "C'est imposant" );
			   FM_send( OP_TEST | size, (unsigned char *)tbuf );
			   } break;
		case 'f' : {
			   char tbuf[20];			//   123456789012345
			   int size = snprintf( tbuf, sizeof(tbuf), "C'est imposant!" );
			   FM_send( OP_TEST | size, (unsigned char *)tbuf );
			   } break;
		case 'A' : break;
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

#ifdef USE_UART1
void USART1_IRQHandler( void )
{
if	(
	( LL_USART_IsActiveFlag_TXE( USART1 ) ) &&
	( LL_USART_IsEnabledIT_TXE( USART1 ) )
	)
	UART1_TX_INT_disable();
if	(
	( LL_USART_IsActiveFlag_RXNE( USART1 ) ) &&
	( LL_USART_IsEnabledIT_RXNE( USART1 ) )
	)
	{
	rxbuf1[(rxwi1++)&(QRX1-1)] = LL_USART_ReceiveData8( USART1 );
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
Rx_cmd(1);
#endif

#ifdef USE_GPS
nmea_ctx gps_ctx;		// we allocate the context here, it has a pointer to the variables
#define NMEA_CTX (&gps_ctx)
//gps_var gps_vars[QVAR];	// we allocate the variables here
//NMEA_CTX->data = gps_vars; 	// then we make the context aware of it
gps_var_init( NMEA_CTX );	// we initialize the variables we are interested in
new_sentence( NMEA_CTX);
#endif

#ifdef USE_UART1
UART1_init( 9600 );
gpio_uart1_init();
#endif

#ifdef USE_PWM
#include "stm32f1xx_ll_tim.h"
// #include "pwm.h"
gpio_timer3_init();
TIM3_PWM_init( PWM_PERIOD );
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
set_cursor( 0, 0 ); lcd_print("Ready");
#endif

/* test statique *
#include <string.h>
const char * testbuf = "$GNGGA,184941.00,4332.71767,N,00129.26376,E,1,05,5.02,135.7,M,48.6,M,,*41";
new_sentence(NMEA_CTX);
for	( unsigned int i = 0; i < strlen(testbuf); i++ )
	{
	nmea_proc( NMEA_CTX, testbuf[i] );
	if	( NMEA_CTX->status == 42 )
		{
		int h,mn,ss;
		if	( ( NMEA_CTX->data[0].stat == 0 ) && ( NMEA_CTX->data[1].stat == 0 ) && ( NMEA_CTX->data[2].stat == 0 ) )
			{
			h = NMEA_CTX->data[0].val; mn = NMEA_CTX->data[1].val; ss = NMEA_CTX->data[2].val;
			}
		else	h = mn = ss = 0;
		int gps_cnt;
		if	( NMEA_CTX->data[16].stat == 0 )
			gps_cnt = NMEA_CTX->data[16].val & 63;
		else	gps_cnt = 0;
		#ifdef USE_LCD2x16
		char lcdbuf[16];
		snprintf( lcdbuf, sizeof(lcdbuf), "%02uh%02umn%d, %02d", h, mn, ss, gps_cnt  );
		// snprintf( lcdbuf, sizeof(lcdbuf), "%8d  %02d", raw_knots, gps_cnt );	// number of sats
		set_cursor( 0, 1 ); lcd_print( lcdbuf );
		#endif
		}
	}
//*/
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
		switch	(op)
			{
			case OP_REPORT :
				if	( sz >= 5 )
					{
					int vals, valh;
					static unsigned int cnt = 0;
					switch	( rxbuf3[1] )
						{
						case VAR_AMP :
							vals = rxbuf3[2] | ( rxbuf3[3] << 8 ) | ( rxbuf3[4] << 16 ) | ( rxbuf3[5] << 24 );
							valh = rxbuf3[6] | ( rxbuf3[7] << 8 ) | ( rxbuf3[8] << 16 ) | ( rxbuf3[9] << 24 );
							// arrondi de valh a 100 mA
							int valhabs = ((valh<0)?(-valh):(valh));
							valhabs +=50;
							valhabs /= 100;
							int valhA = valhabs / 10;	// amperes
							int valhD = valhabs % 10;	// dixiemes
							if ( valh < 0 ) valhA = -valhA;
							#ifdef USE_CDC
							snprintf( txbuf2, sizeof(txbuf2), "%d %d.%d [%u]\n", vals, valhA, valhD, cnt );
							txindex2 = 0; UART2_TX_INT_enable();
							#endif
							#ifdef USE_LCD2x16
				  			char lcdbuf[20];
				  			snprintf( lcdbuf, sizeof(lcdbuf), "------------- %02d", cnt % 100 );
				  			set_cursor( 0, 0 ); lcd_print( lcdbuf );
				  			snprintf( lcdbuf, sizeof(lcdbuf), "%5d %d.%d ", vals, valhA, valhD );
				 			set_cursor( 0, 0 ); lcd_print( lcdbuf );
							#endif
							cnt++; amp_timout100Hz = 0;
						break;
						}
					}
				break;
			default:
				#ifdef USE_CDC
				snprintf( txbuf2, sizeof(txbuf2), "opcode 0x%02X, ?", rxbuf3[0] );
				#endif
				break;
			}
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
	if	( amp_timout100Hz > 250 )
		{
		#ifdef USE_LCD2x16
		char lcdbuf[4];
		snprintf( lcdbuf, sizeof(lcdbuf), "NS" );	// no signal !
		set_cursor( 14, 0 ); lcd_print( lcdbuf );
		#endif
		amp_timout100Hz = 0;
		}
	if	( K1_PRESS() )
		{
		#ifdef USE_LCD2x16
		  set_cursor( 13, 1 ); lcd_print(" K1");
		#endif
		if	( ( rx_status < 3 ) && ( tx_status == 0 ) )
			{
			unsigned char auto_tx_1 = 0;
			FM_send2( OP_SET | 2, VAR_AUTO1, &auto_tx_1 );
			#ifdef USE_LCD2x16
			set_cursor( 13, 1 ); lcd_print("-K1");
			#endif
			}
		}
	if	( BLUE_PRESS() )
		{
		#ifdef USE_LCD2x16
		  set_cursor( 13, 1 ); lcd_print(" K2");
		#endif
		if	( ( rx_status < 3 ) && ( tx_status == 0 ) )
			{
			unsigned char auto_tx_1 = 1;
			FM_send2( OP_SET | 2, VAR_AUTO1, &auto_tx_1 );
			#ifdef USE_LCD2x16
			set_cursor( 13, 1 ); lcd_print("+K2");
			#endif
			}
		}
	#endif		// UART3_FM
	#ifdef USE_GPS
	while	( rxwi1 - rxri1 )
		{
		nmea_proc( NMEA_CTX, rxbuf1[(rxri1++)&(QRX1-1)] );
		if	( NMEA_CTX->status == 43 )
			{
			invalidate_sentence( NMEA_CTX );
			new_sentence( NMEA_CTX );
			}
		else if	( NMEA_CTX->status == 42 )
			{
			new_sentence( NMEA_CTX );	// remet a zero le parseur mais les variables sont persistantes
			if	( NMEA_CTX->fourcc == NGGA )
				{
				int gps_cnt;
				if	( NMEA_CTX->data[16].stat == 0 )
					gps_cnt = NMEA_CTX->data[16].val & 63;
				else	gps_cnt = 0;
				int raw_knots;
				if	( NMEA_CTX->data[10].stat == 0 )
					raw_knots = NMEA_CTX->data[10].val;
					else	raw_knots = 0;
				int h,mn,ss;
				if	( ( NMEA_CTX->data[0].stat == 0 ) && ( NMEA_CTX->data[1].stat == 0 ) && ( NMEA_CTX->data[2].stat == 0 ) )
					{
					h = NMEA_CTX->data[0].val; mn = NMEA_CTX->data[1].val; ss = NMEA_CTX->data[2].val;
					}
				else	h = mn = ss = 0;
				#ifdef USE_LCD2x16
				char lcdbuf[16];
				if	( gps_cnt == 0 )
					snprintf( lcdbuf, sizeof(lcdbuf), "%02uh%02u\"%04d ", h, mn, ss  );
				else	snprintf( lcdbuf, sizeof(lcdbuf), "%05dk %02dsat", raw_knots, gps_cnt );	// number of sats
				set_cursor( 0, 1 ); lcd_print( lcdbuf );
				#endif
				}
			}
		}
	#endif		// GPS
 	}
}
#endif	// main
