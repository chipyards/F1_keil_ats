/* Ce prog est pour une station de mesure

- il acquiert et integre des mesures chaque ms
- il est capable d'emettre des resultats en FM 433 MHz, periodiquement ou sur requete
- opcodes supportes en Rx:
	OP_TEST (demande d'echo)
	OP_SET  (pour les variables supportees)
- opcodes supportes en Tx:
	OP_ECHO
	OP_REPORT
- resultats :
	VAR_AMP : milliamperes instantanes (bin 24 bits signes)
	VAR_COUL : coulombs integres (futur)

 */
#include "options.h"
#ifdef MAIN_COULOMB
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

#ifdef USE_ADC_4CH
#include "adc.h"
#endif

#ifdef USE_UART3_FM
#include "fm.h"
#endif

void SystemClock_Config(void);
void cmd_handler( char c );

// contexte global -----------------------------------------------------------

volatile unsigned int cnt100Hz = 0;

#ifdef USE_CDC
// emission sur CDC : par message
char txbuf2[64];
volatile int txindex2;
volatile char rxbyte2;
#endif

volatile int auto_tx_1 = 0;

// systick interrupt handler
void SysTick_Handler()
{
++cnt100Hz;
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
	rxbyte2 = LL_USART_ReceiveData8( USART2 );
	cmd_handler( rxbyte2 );
	}
}

void cmd_handler( char c )
{
if	( !LL_USART_IsEnabledIT_TXE( USART2 ) )
	{
	switch	( c )
		{
		#ifdef USE_UART3_FM
		case 'T' : Rx_cmd(0); Tx_cmd(1); break;
		case 'R' : Tx_cmd(0); Rx_cmd(1); break;
		case 'S' : Rx_cmd(0); Tx_cmd(0); auto_tx_1 = 0; break;
		case 'e' : {
			   char tbuf[20];			//   123456789012345
			   int size = snprintf( tbuf, sizeof(tbuf), "C'est imposant" );
			   FM_send( OP_TEST | size, (unsigned char *)tbuf );
			   } break;
		case 'A' : auto_tx_1 = 1; break;
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
	}
}
#endif	// CDC

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
#endif	// ADC

// LA GROSSE BOUCLE MAIN LOOP
while	(1)
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
	#ifdef USE_ADC_4CH
	// emission periodique
	if	( ( auto_tx_1 ) && ( adc_res_ready == 2 ) )
		{
		#ifdef USE_CDC
		snprintf( txbuf2, sizeof(txbuf2), "npvhdd %d %d %d %d %d %d\n", adc1_res0/CHANFIR, adc2_res0/CHANFIR, adc1_res1/CHANFIR, adc2_res1/CHANFIR,
			  ( (int)adc2_res0 - (int)adc1_res0 )/CHANFIR, ( (int)adc2_res1 - (int)adc2_res0 )/CHANFIR );
		txindex2 = 0; UART2_TX_INT_enable();
		#endif
		#ifdef USE_UART3_FM
		unsigned int val = adc2_res1/CHANFIR;
		FM_send2( OP_REPORT | 5, VAR_AMP, (unsigned char *)&val );
		#endif
		adc_res_ready = 0;	// acknowledge
		}
	// traitement des requetes
	if	( rx_status == 10 )
		{
		int op = rxbuf3[0] & 0xF0;
		int sz = rxbuf3[0] & 0x0F;
		switch	(op)
			{
			case OP_SET:
				if	( sz >= 2 )
					{
					switch	( rxbuf3[1] )
						{
						case VAR_AUTO1 :
							auto_tx_1 = rxbuf3[2];
							#ifdef USE_CDC
							snprintf( txbuf2, sizeof(txbuf2), "auto_tx_1 = %d\n", auto_tx_1 );
							txindex2 = 0; UART2_TX_INT_enable();
							#endif
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
		rx_status = 0;	// acknowledge
		}
	else if	( ( rx_status == 20 ) || ( rx_status == 21 ) || ( rx_status == 22 ) )
		{
		#ifdef USE_CDC
		snprintf( txbuf2, sizeof(txbuf2), "Bad %d\n", rx_status );
		txindex2 = 0; UART2_TX_INT_enable();
		#endif
		rx_status = 0;	// acknowledge
		}
	#endif
 	}
}
#endif
