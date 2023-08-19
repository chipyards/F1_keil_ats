/* prog pour emettre ou recevoir des messages en FM 433 MHz, taille variable, pas de delimiteur, contenu arbitraire
- emission :
	- quelques FF, 2 magic numbers, 1 opcode, 1 payload, 2 bytes de CRC
	  N.B. les 4 LSBs de l'opcode fournissent la taille de la payload (pas de delimiteur),
	  l'opcode n'est pas lui-meme compte dans cette taille
	- payload de test : op 0x00 : 32 bit en binaire, op 0x10 : 32 bits hex en ascii (0 a 8 digits) ou texte <= 14 char
	- emission periodique (3s) si autoTx = 1, ou emission manuelle via CDC
	  compiler sans USE_LCD2x16 demarre avec autoTx = 1 pour tester un emetteur autonome
- reception :
	- reset avec bouton bleu ou compiler avec USE_LCD2x16 : reception permanente meme pendant emission
	  ==> relecture possible (works ok)
	- la FSM de reception detecte au moins un FF, les magic numbers, lit l'opcode, deduit la taille de la payload
	  stocke opcode et payload, et verifie le crc
	- message transfere vers CDC (ascii)
- CRC :
	- polynome CCIT-16, little endian, mask 0x8408 (Koopman 0x8810), CRC-16/KERMIT chez reveng.sourceforge.io
	- init 0, simuler avec CRC16_jln -p
	- couvre l'opcode et la payload
 */
/* Includes ------------------------------------------------------------------*/
#include "options.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_adc.h"

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

// emission sur CDC : par message
char txbuf[64];
volatile int txindex;

// reception CDC : fifo circulaire
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
#ifdef USE_UART3
#ifdef USE_LCD2x16
int autoTx = 0;
#else
int autoTx = 1;
#endif
// les 4 MSBs pour l'opcode au debut du message
#define OP_BIN 0x00
#define OP_ASC 0x10
void FM_send( unsigned int opcode, const unsigned char * payload );
#endif

// systick interrupt handler
void SysTick_Handler()
{
++cnt100Hz;
// LED blinks
#ifdef USE_UART3
if	( autoTx == 0 )
#endif
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
// log periodique
#ifdef USE_UART3
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
}

// temporisation base sur systick
// unites en periodes d'horloge du timer ( HCLK ou HCLK/8 )
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

// UART2 (CDC) interrupt handler
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

// N.B. pour avoir la correspondance numero <--> perif , voir IRQn_Type
// F103 : UARTS 1,2,3 : 37, 38, 39; TIM 2, 3, 4 : 28, 29, 30
void report_interrupts(void)
{
int i, p, space, j;
p = __NVIC_GetPriorityGrouping();
j = snprintf( txbuf, sizeof(txbuf), "P.G. %d\n", p );
// special systick (#-1)
i = -1;
if	(  SysTick->CTRL & SysTick_CTRL_TICKINT_Msk )
	{
	p = __NVIC_GetPriority((IRQn_Type)i);
	space = sizeof(txbuf) - j;
	if	( space > 0 )
		j += snprintf( txbuf+j, space, "i #%2d, p %d\n", i, p );
	}
// tous les autres
for	( i = 0; i <=  97; ++i )
	{
	if	( __NVIC_GetEnableIRQ((IRQn_Type)i) )
		{
		p = __NVIC_GetPriority((IRQn_Type)i);
		space = sizeof(txbuf) - j;
		if	( space > 0 )
			j += snprintf( txbuf+j, space, "i #%2d, p %d\n", i, p );
		}
	}
txindex = 0; UART2_TX_INT_enable();
}

#ifdef USE_UART3
// systemes de communication FM 433 MHz
#define FM_MAGIC1 0xA5
#define FM_MAGIC2 0xe6
// Tx zone
char txbuf3[16];	// opcode plus 0 to 15 bytes payload
volatile int txindex3;
volatile unsigned int tx_paycnt;
volatile unsigned int tx_crc;
volatile int tx_status = 0;
// Rx zone
char rxbuf3[16];	// opcode plus 0 to 15 bytes payload
volatile int rxindex3 = 0;
volatile unsigned int rx_paycnt;
volatile unsigned int rx_crc;
volatile int rx_status = 0;

// CRC CCITT 16 little endian
unsigned int crc_1byte( unsigned int crc, unsigned int b )
{
for	( unsigned int i = 0; i < 8; i++ )
	{
	if	( ( crc ^ b ) & 1 )
		crc = ( crc >> 1 ) ^ 0x8408;
	else	crc >>= 1;
	b >>= 1;
	}
return crc;
}

// UART3 interrupt handler
void USART3_IRQHandler( void )
{
char c;
if	(
	( LL_USART_IsActiveFlag_TXE( USART3 ) ) &&
	( LL_USART_IsEnabledIT_TXE( USART3 ) )
	)
	{
	// FM Rx FSM status
	//	0 : idle
	//	1..7 : FF to send
	//	8 : magic1 to send
	//	9 : magic2 to send
	//	10 : payload to begin, opcode to send
	//	11 : sending payload data
	//	12 : 2nd CRC byte to send
	//	13 : sacrified byte to send
	//	14 : transmitter to be switched off
	switch	( tx_status )
		{
		case 1:	Tx_cmd(1);
			LL_USART_TransmitData8( USART3, 0xFF ); tx_status++; break;
		case 2:	LL_USART_TransmitData8( USART3, 0xFF ); tx_status++; break;
		case 3:	LL_USART_TransmitData8( USART3, 0xFF ); tx_status++; break;
		case 4:	LL_USART_TransmitData8( USART3, 0xFF ); tx_status = 8; break;
		case 8:	LL_USART_TransmitData8( USART3, FM_MAGIC1 ); tx_status++; break;
		case 9:	LL_USART_TransmitData8( USART3, FM_MAGIC2 ); tx_status++; break;
		case 10: txindex3 = 0;
			c = txbuf3[txindex3++];
			LL_USART_TransmitData8( USART3, c );
			tx_crc = crc_1byte( 0, c );		// init crc = 0
			tx_paycnt = c & 0x0F;			// taille = 4 LSBs
			tx_status++;
			break;
		case 11: if	( tx_paycnt )
				{
				c = txbuf3[txindex3++];
				LL_USART_TransmitData8( USART3, c );
				tx_crc = crc_1byte( tx_crc, c );
				tx_paycnt--;
				}
			else	{
				LL_USART_TransmitData8( USART3, tx_crc & 0xFF );
				tx_status++;
				}
			break;
		case 12: LL_USART_TransmitData8( USART3, ( tx_crc >> 8 ) & 0xFF );
			tx_status++;
			break;
		case 13: LL_USART_TransmitData8( USART3, ' ' );
			tx_status++;
			break;
		case 14: UART3_TX_INT_disable(); Tx_cmd(0);
			tx_status = 0;
			break;
		}
	}
if	(
	( LL_USART_IsActiveFlag_RXNE( USART3 ) ) &&
	( LL_USART_IsEnabledIT_RXNE( USART3 ) )
	)
	{
	// FM Rx FSM status
	//	0 : idle
	//	1 : FF received, waiting fo magic
	//	2 : magic1 received, waiting for magic2
	//	3 : magic2 received, waiting for opcode
	//	4 : opcode received, crc started, accepting data or 1st CRC byte
	//	5 : 1st CRC byte Ok, waiting for 2nd CRC byte
	//	10 : CRC ok, waiting for ack
	//	20 : buffer full, waiting for ack
	//	21 : bad crc 1st, waiting for ack
	//	22 : bad crc 2nd, waiting for ack
	c = LL_USART_ReceiveData8( USART3 );
	switch	( rx_status )
		{
		case 0: if	( c == 0xFF )
				{ rx_status = 1; rxindex3 = 0; }
			break;
		case 1:	if	( c == FM_MAGIC1 )
				rx_status = 2;
			else if	( c != 0xFF )
				rx_status = 0;
			break;
		case 2:	if	( c == FM_MAGIC2 )
				rx_status = 3;
			else	rx_status = 0;
			break;
		case 3: if	( rxindex3 < sizeof(rxbuf3) )
				{
				rxbuf3[rxindex3++] = c;			// opcode
				rx_paycnt = c & 0x0F;			// taille = 4 LSBs
				rx_crc = crc_1byte( 0, c );		// init crc = 0
				rx_status = 4;
				}
			else	rx_status = 20;				// buffer full
			break;
		case 4:	if	( rxindex3 <= rx_paycnt )
				{
				if	( rxindex3 < sizeof(rxbuf3) )
					{
					rxbuf3[rxindex3++] = c;
					rx_crc = crc_1byte( rx_crc, c );
					}
				else	rx_status = 20;			// buffer full
				}
			else	{
				if	( c == ( rx_crc & 0xFF ) )
					rx_status = 5;
				else	rx_status = 21;			// bad CRC
				}
			break;
		case 5:
			if	( c == ( ( rx_crc >> 8 ) & 0xFF ) )
				rx_status = 10;
			else	rx_status = 22;			// bad CRC
			break;
		case 10: // nothing to do, app will reset rx_status
		case 20: // nothing to do, app will reset rx_status
		case 21: // nothing to do, app will reset rx_status
		case 22: // nothing to do, app will reset rx_status
			break;
		default: rx_status = 0;
		}
	}
}


// message formatage for FM 433MHz
void FM_send( unsigned int opcode, const unsigned char * payload )
{
unsigned int i=0, paycnt;
txbuf3[i++] = opcode;
paycnt = opcode;	// provisoire
while	( paycnt )
	{
	txbuf3[i++] = *(payload++);
	paycnt--;
	if	( i >= sizeof(txbuf3) )
		break;
	}
tx_status = 1;
UART3_TX_INT_enable();
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
		#ifdef USE_UART3
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
			snprintf( txbuf, sizeof(txbuf), "NPVHdd %d %d %d %d %d %d\n", adc1_res0, adc2_res0, adc1_res1, adc2_res1,
				  (int)adc2_res0 - (int)adc1_res0, (int)adc2_res1 - (int)adc2_res0 );
			txindex = 0; UART2_TX_INT_enable();
			break;
		case 'h' :
			adc_timer_stop();
			snprintf( txbuf, sizeof(txbuf), "ADC interrupt halted\n" );
			txindex = 0; UART2_TX_INT_enable();
			break;
		case 'c' :
			adc_calib();
			snprintf( txbuf, sizeof(txbuf), "calib. done\n" );
			txindex = 0; UART2_TX_INT_enable();
			break;
		case 'u' :
			adc_uncalib();
			snprintf( txbuf, sizeof(txbuf), "calib. erased\n" );
			txindex = 0; UART2_TX_INT_enable();
			break;
		case 'r' :
			adc_timer_init( SystemCoreClock / 2000 );	// 2 kHz ==> 1 ksamp/s pour chaque canal avant FIR );
			snprintf( txbuf, sizeof(txbuf), "ADC interrupt restarted\n" );
			txindex = 0; UART2_TX_INT_enable();
			break;
		case 't' :
			adc_start_conv();
			snprintf( txbuf, sizeof(txbuf), "test conversion started\n" );
			txindex = 0; UART2_TX_INT_enable();
			break;
		case 'd' :
			snprintf( txbuf, sizeof(txbuf), "test conversion %lu %lu\n", ADC1->DR, ADC2->DR );
			txindex = 0; UART2_TX_INT_enable();
			break;
		#endif
		case '$' : report_interrupts();
			break;
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
if	( BLUE_PRESS() )
	Rx_cmd(1);
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
	SCB->SCR = 0;				// avoid deep sleep
	PWR->CR &= ~(PWR_CR_PDDS|PWR_CR_LPDS);	// avoid power down
	__WFI();				// Wait for Interrupt
	#endif
	#ifdef PROF_PB12_EOS
	if	( LL_ADC_IsActiveFlag_EOS(ADC1) ) PB12_PROFIL_0();
	else					  PB12_PROFIL_1();
	#endif
	#ifdef PROF_PB12_DLY
	PB12_PROFIL_1();
	tickdelay( Avar );
	PB12_PROFIL_0();
	tickdelay( Avar );
	#endif
	#ifdef USE_UART3
	if	( rx_status == 10 )
		{
		int op = rxbuf3[0] & 0xF0;
		int sz = rxbuf3[0] & 0x0F;
		if	( op == OP_BIN )
			{
			if	( sz == 4 )
				{
				snprintf( txbuf, sizeof(txbuf), "bin Ok %02X%02X%02X%02X ", rxbuf3[4], rxbuf3[3], rxbuf3[2], rxbuf3[1] );
				#ifdef USE_LCD2x16
				  char lcdbuf[20];
				  snprintf( lcdbuf, sizeof(lcdbuf), "bin Ok %02X%02X%02X%02X", rxbuf3[4], rxbuf3[3], rxbuf3[2], rxbuf3[1] );
				  set_cursor( 0, 0 ); lcd_print("----------------");
				  set_cursor( 0, 0 ); lcd_print( lcdbuf );
				#endif
				}
			else	snprintf( txbuf, sizeof(txbuf), "opcode 0x%02X, ?", rxbuf3[0] );
			}
		else if	( op == OP_ASC )
			{
			if	( sz > 14 )	// taille limitee a 14 en ascii pour pouvoir AJOUTER le null terminator
				sz = 14;	// c'est juste pour la commodite de l'affichage
			rxbuf3[sz+1] = 0;	// rxbuf3 contient l'opcode suivi de la payload
			snprintf( txbuf, sizeof(txbuf), "asc Ok %d %s\n", rxbuf3[0] & 0x0F, rxbuf3+1 ); // size puis payload, sans l'opcode
			#ifdef USE_LCD2x16
			  set_cursor( 0, 1 ); lcd_print("----------------");
			  set_cursor( 0, 1 ); lcd_print( rxbuf3+1 );
			#endif
			}
		else	snprintf( txbuf, sizeof(txbuf), "opcode 0x%02X, ?", rxbuf3[0] );
		txindex = 0; UART2_TX_INT_enable();
		rx_status = 0;
		}
	else if	( ( rx_status == 20 ) || ( rx_status == 21 ) || ( rx_status == 22 ) )
		{
		snprintf( txbuf, sizeof(txbuf), "Bad %d\n", rx_status );
		txindex = 0; UART2_TX_INT_enable();
		#ifdef USE_LCD2x16
		  set_cursor( 0, 0 ); lcd_print("::::::::::::::::");
		  set_cursor( 2, 0 ); lcd_print( txbuf );
		#endif
		rx_status = 0;
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
