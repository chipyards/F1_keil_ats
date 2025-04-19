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
#include "apilot.h"
#include "CC1101.h"

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


unsigned int echo_cnt = 0;
// periodic report sent in ECHO mode
void tx_echo_report() {
unsigned char data[4];
data[0] = FLIGHT+128;
data[1] = 0;
data[2] = echo_cnt;
data[3] = echo_cnt >> 8;
echo_cnt += 4;
int resu = CC.tx_if_can( data, 4 );
CDC_printf("tx e=%u -> %d\n", echo_cnt, resu );
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

CDC_init();	// on doit faire cela avant d'entrer dans la main loop,
		// pour que CDC_getcmd() ne capte pas de garbage tant qu'on n'a pas fait UART2_init()

// Cette version est destinee a un espionnage passif des messages WUCAM, qui sont simplement
// reportes sur l'UART CDC. Le simulateur de vol Apilot est inactif.

gpio_uart2_init();
UART2_init( 38400 );
CDC.verbose = 1;
CDC_printf("CC.mode = RX_ONLY\n");
// experience pour detecter le piege: "char is unsigned  ?"
// unsigned char ccc = 0xFF;
// int iss = (int)((signed char)ccc);
// int icc = (int)((       char)ccc);
// CDC_printf("%02x -> %d = %d\n", ccc, iss, icc );

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
 		if	( cnt1Hz == 1 )	// do this once
 			{
			CC.GFSK_radio_init();
			}
		}
	if	( ( IS_GDO0_SET() ) && ( oldGDO0 == 0 ) )
		{
		unsigned char * data = CC.extract_rx();
		int flight = data[1] & 0x7F;
		opcode_t opcode = (opcode_t)data[2];
		if	( opcode == VAAR )
			{
			unsigned int len = data[0];
			unsigned int lo_time = (unsigned int)data[len-1];	// timestamp modulo 256
			int hrssi = ((int)((signed char)data[len+1])) - (2*74);	// RSSI in half-dB
			float rssi = qfp_fmul( 0.5, (float)hrssi );		// in dB
			unsigned char LQI = data[len+2];
			CDC_printf("%3u: %3u %4.1fdBm, LQI=%u\n", lo_time, flight, rssi, LQI & 0x7F );
			}
		else	CC.formatb_rx_to_CDC( data );
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
		if	( CDC.verbose <= 0 )
			{	// si, au reset, verbose = -4, la reception d'un '4' va le faire passer a -3
			if	( ( '0' - c ) == CDC.verbose )	// le code "43210" va le faire passer a +1
				CDC.verbose += 1;		// et le dialogue deviendra possible
			}
		else	CC.demo( c );
		}
	#endif
 	} // while (1)
}	// main
#endif
