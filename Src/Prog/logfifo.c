/* fifo pour log vers uart ou transcript scrollable */

#include "options.h"
#ifdef USE_LOGFIFO

#include <stdio.h>
#include <stdarg.h>
#include "logfifo.h"
#ifdef USE_UART2
#include "uarts.h"
#endif

// contexte global (singleton)
LOGtype logfifo;

// constructeur
void logfifo_init(void)
{
logfifo.wra = 0;
logfifo.rda = 0;
LOGprint( "LOG FIFO %d bytes", LFIFOQB );
}

// ajouter un caractere dans la queue
void LOGputc( char c )
{
logfifo.circ[logfifo.wra++] = c;
logfifo.wra &= LFIFOMS;
}

// ajouter une ligne de texte au transcript - sera tronquee si elle est trop longue
void LOGline( const char *txt )
{
int i;
char c;
for	( i = 0; i < LFIFOLL; ++i )
	{
	c = txt[i];
	if	( ( c >= ' ' ) || ( c == 0 ) )
		LOGputc( c );
	if	( c == 0 )
		break;
	}
if	( c != 0 )
	LOGputc( 0 );
#ifdef USE_UART2
UART2_TX_INT_enable();
#endif
}

// ajouter une ligne de texte formattee - sera tronquee si elle est trop longue
void LOGprint( const char *fmt, ... )
{
char lbuf[LFIFOLL];
va_list  argptr;
va_start( argptr, fmt );
vsnprintf( lbuf, sizeof(lbuf), fmt, argptr );
va_end( argptr );
LOGline( lbuf );
}

// a utiliser apres une serie de LOGputc()
// (pas necessaire apres LOGprint() ou LOGline() ) 
void LOGflush(void)
{
LOGputc( 0 );
#ifdef USE_UART2
UART2_TX_INT_enable();
#endif
}

#endif
