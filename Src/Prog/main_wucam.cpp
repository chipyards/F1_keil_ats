/* prog ENAC's WUCAM
 */

#include "options.h"
#ifdef MAIN_WUCAM
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

void cmd_handler( char c );

// contexte global -----------------------------------------------------------

volatile unsigned int cnt100Hz = 0;
volatile unsigned int cnt1Hz = 0;
volatile unsigned int cntblinks = 1;

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
		case 15 : if ( cntblinks > 1 ) LED_ON();
			break;
		case 30 : if ( cntblinks > 2 ) LED_ON();
			break;
		case 45 : if ( cntblinks > 3 ) LED_ON();
			break;
		case 60 : if ( cntblinks > 4 ) LED_ON();
			break;
		case 75 : if ( cntblinks > 5 ) LED_ON();
			break;
		case 4 :
		case 19 :
		case 34 :
		case 49 :
		case 64 :
		case 79 : LED_OFF();
			break;
		}
	}
#else	// pour nucleo c'est deja fait ci-dessus
if	( ( cnt100Hz % 100 ) == 0 )
	++cnt1Hz;
#endif
} // SysTick_Handler()

#ifdef __cplusplus
}
#endif


// calcul CRC32, mis ici provisoirement
unsigned int crc_aixm( const unsigned char *buf, unsigned int len )
{
unsigned int crc = 0, i, msbin, msbreg, bit;
unsigned char lebyte;
do	{
	lebyte = *(buf++);
	for	( i = 0; i < 8; i++ )
		{
		msbin  = lebyte >> 7;
		msbreg = crc >> 31;
		bit = ( msbin ^ msbreg ) & 1;
		crc <<= 1;
		if	( bit )
			crc ^= 0x814141ABL;
		lebyte <<= 1;
        	}
	} while (--len);
return crc;
}

unsigned int echo_cnt = 0;
// periodic report sent in ECHO mode
void tx_echo_report() {
unsigned char data[4];
data[0] = FLIGHT+128;
data[1] = 0;
data[2] = echo_cnt;
data[3] = echo_cnt >> 8;
echo_cnt += 4;
CC.tx_if_can( (char *)data, 4 );
}


int main(void)
{
// Configure the system clock to 8 or 64 or 72 MHz according to options.h
SystemClock_Config();
// config systick @ 100Hz
systick_init( 100 );

gpio_init();

#ifdef USE_CC1101
gpio_spi1_init();
SPI1_init();
#endif

// LA GROSSE BOUCLE MAIN LOOP
// pendant les 10 premieres secondes, le service est configure par etapes,
// et la LED indique que le sleep n'est pas actif (facilite acces debug sur blue pill)
while (1)
 	{
 	static unsigned int old1Hz = 0;
 	static unsigned int oldGDO0 = 0;
 	if	(  ( old1Hz != cnt1Hz ) )	// une fois par seconde
 		{
 		old1Hz = cnt1Hz;
 		if	( cnt1Hz == 1 )		// do this once
 			{
 			// mode ECHO avec jumper A2-A3 pour Blue Pill, avant activation UART CDC
			if	( gpio_test_jmpA23() )
				CC.mode = ECHO;
			else	CC.mode = PILOT;
			}
 		if	( cnt1Hz == 10 )	// do this once
 			{
 			if	( gpio_test_jmpA23() )	// si le jumper y est encore, passer en CW
 				{			// alors pas de CDC !
 				if	( CC.mode == ECHO )
 					CC.mode = CW;
 				}
 			else	{
	 			#ifdef USE_CDC
				// config UART (interrupt handler doit etre pret!!)
				gpio_uart2_init();
				UART2_init( 9600 );
				CDC_init();
				CDC_printf("CC.mode = %s\n", ((CC.mode == ECHO)?("ECHO"):("PILOT")) );
				#endif
 				}
  			}
		else if	( cnt1Hz == 11 )	// do this once
			{
			cntblinks = 6;	// pour le cas ou SPI planterait dans while ( IS_MISO_SET() ), i.e. si transceiver absent
			if	( CC.mode == CW )
				cntblinks = 3 + CC.cw_radio_init();	// 3 blink si Ok, sinon 4
			else	{
				cntblinks = 1 + CC.GFSK_radio_init();	// 1 blink si Ok, sinon 2
				if	( CC.mode == ECHO )
					cntblinks += 2;			// 5 si Ok, sinon 6
				}
			}
		else if	( ( cnt1Hz > 11 ) && ( CC.AAR_tx_enable ) )
			{			// do something exactly once per second
			if	( CC.mode == PILOT )
				lepilot.AAR_tx();
			else if ( ( cnt1Hz & 3 ) == 0 )
				tx_echo_report();
			}
		}
	if	( ( IS_GDO0_SET() ) && ( oldGDO0 == 0 ) )
		{
		unsigned char * data = CC.extract_rx();
		if	( ( data[0] == 4 ) && ( data[1] == FLIGHT ) && ( data[2] == 0 ) )
			{
			echo_cnt = ( data[3] & 0xff ) | ( data[4] << 8 );
			CDC_printf("echo _cnt <- %u (no CRC)\n", echo_cnt );
			}
		else if	( ( data[0] == 8 ) && ( data[1] == FLIGHT ) && ( data[2] == 0 ) )
			{
			unsigned int remoteCRC = ( data[5] & 0xff ) | ( data[6] << 8 ) | ( data[7] << 16 ) | ( data[8] << 24 );
			unsigned int localCRC = crc_aixm( data+1, 4 );
			if	( remoteCRC == localCRC )
				{
				echo_cnt = ( data[3] & 0xff ) | ( data[4] << 8 );
				CDC_printf("echo_cnt <- to %u, CRC ok\n", echo_cnt );
				}
			else	CDC_printf("echo_cnt rejected, bad CRC %08x vs %08x\n", localCRC, remoteCRC );
			}
		else	CC.format_rx_to_CDC( data );
		oldGDO0 = 1;
		}
	else	oldGDO0 = 0;
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
		CC.demo( c );
		// lepilot.cmd_handler(c);
		}
	#endif
 	} // while (1)
}	// main
#endif
