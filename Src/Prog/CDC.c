#include "options.h"

#ifdef USE_CDC
#include "stm32f1xx_ll_usart.h"
#include "uarts.h"
#include <stdarg.h>
#include <stdio.h>	// pour snprintf
#include "CDC.h"

// objet CDC ------------------------------------------------------------------------------

CDCtype CDC;	// pour communiquer avec PC via ST-Link

// UART2 (CDC) interrupt handler
void USART2_IRQHandler( void )
{
if	(
	( LL_USART_IsActiveFlag_TXE( USART2 ) ) &&
	( LL_USART_IsEnabledIT_TXE( USART2 ) )
	)
	{	// messages de taille variable
	if	( CDC.TXbuf[CDC.TXindex] == 0 )
		UART2_TX_INT_disable();
	else	LL_USART_TransmitData8( USART2, CDC.TXbuf[CDC.TXindex++] );
	}
if	(
	( LL_USART_IsActiveFlag_RXNE( USART2 ) ) &&
	( LL_USART_IsEnabledIT_RXNE( USART2 ) )
	)
	{
	#ifdef RX_FIFO
	CDC.RXbuf[(CDC.RXwi++)&(QRX-1)] = LL_USART_ReceiveData8( USART2 );
	#else
	CDC.RXbyte = LL_USART_ReceiveData8( USART2 );
	#endif
	}
}

void UART2_wait_TX_complete()
{
while	( LL_USART_IsEnabledIT_TXE( USART2 ) )
	{}
}

// constructeur
void CDC_init()
{
CDC.TXindex = 0;
CDC.RXbyte = -1;	// empty
CDC.verbose = 0;
}

// envoyer une ligne de texte formattee
// retourne 1 si renoncement pour cause de transmission en cours
void CDC_printf( const char *fmt, ... )
{
if	( CDC.verbose <= 0 )
	return;
va_list  argptr;
va_start( argptr, fmt );
UART2_wait_TX_complete();
vsnprintf( (char *)CDC.TXbuf, sizeof(CDC.TXbuf), fmt, argptr );
va_end( argptr );
CDC.TXindex = 0;
UART2_TX_INT_enable();
}

// lire une commande, rendre -1 s'il n'y a rien de nouveau
int CDC_getcmd()
{
int c = CDC.RXbyte;
CDC.RXbyte = -1;	// consommer le byte
return c;
}

#endif
