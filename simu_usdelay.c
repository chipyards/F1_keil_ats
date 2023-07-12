#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
// prog pour simuler une fonction de delay "a la microseconde"
// base sur un timer qui overflowe avec une periode arbitraire LOAD+1

static unsigned long long Xdrand48 = 0x330E;

unsigned long long my_drand48( void )
{
Xdrand48 *= 0x5DEECE66DLL;
Xdrand48 += 0xB;
Xdrand48 &= 0xFFFFFFFFFFFFLL;
// return( (double)Xdrand48 / (double)(0x1000000000000LL) );
return Xdrand48;
}

void my_srand48( long seedval )
{
Xdrand48 = (unsigned long )seedval;
Xdrand48 <<= 16;
Xdrand48 |= 0x330E;
Xdrand48 &= 0xFFFFFFFFFFFFLL;
}

// le timer :
int LOAD = 720000;	// 10ms sur F103
int VAL = 0;
unsigned long long ABS = 0;

/* hypothese A : systick est incremente (ARRGH c'est pas le cas du STM32)
void inctick( int inc )
{
VAL += inc; ABS += inc;
if	( VAL > LOAD )
	VAL -= ( LOAD + 1 );
}


// la tempo ( attention : il faut d < (LOAD+1)/2 )
void usdelay( int d )
{
int diff, nextVAL = VAL + d;
if	( nextVAL > LOAD )
	nextVAL -= (LOAD+1);
do	{
	diff = nextVAL - VAL;
	if	( diff <= -((LOAD+1)/2) )
		diff = 1;
	else if ( diff > ((LOAD+1)/2) )
		break;
	inctick( 6 + ( ( my_drand48() >> 7 ) & 15 ) );
	} while ( diff > 0 );
}
*/

/* hypothese B : systick est decremente, et maintenu positif ou nul */
void dectick( int inc )
{
VAL -= inc; ABS += inc;
if	( VAL < 0 )
	VAL += ( LOAD + 1 );
}


// la tempo ( attention : il faut d < (LOAD+1)/2 )
void usdelay( int d )
{
int diff, nextVAL;
nextVAL = VAL - d;
if	( nextVAL < 0 )
	nextVAL += (LOAD+1);
do	{			// diff c'est toujours le temps restant a attendre
	diff = VAL - nextVAL;	// on maintient diff entre -((LOAD+1)/2) et ((LOAD+1)/2)
	if	( diff <= -((LOAD+1)/2) )
		diff = 1;
	else if ( diff > ((LOAD+1)/2) )
		break;
	dectick( 6 + ( ( my_drand48() >> 7 ) & 15 ) );
	} while ( diff > 0 );
}


// le test
int main( int argc, char ** argv )
{
int d = 723;
int cnt = 1000;
if	( argc > 1 )
	d = atoi( argv[1] );
if	( argc > 2 )
	cnt = atoi( argv[2] );
printf("d = %d, %d fois\n", d, cnt );
int dmin = INT_MAX;
int dmax = 0;
int dvu;
unsigned long long ABS0;
for	( int i = 0; i < cnt; i++ )
	{
	ABS0 = ABS;
	usdelay( d );
	dvu = (int)(ABS - ABS0);
	if	( dvu > dmax ) dmax = dvu;
	if	( dvu < dmin ) dmin = dvu;
	}
printf("%d < d < %d\n", dmin, dmax );
}
 