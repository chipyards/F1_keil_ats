/** compression predictive avec perte */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "codec.h"


// parametres derives

// les limites de plages cote diff source, t.q. ( bnd[n-1] <= x < bnd[n] ) ==> y = QLIN + n  
// cas limite inferieur				( QLIN-0.5 <= x < bnd[0] ) ==> y = QLIN
// cas limite superieur : bnd[QLOG-1] n'est pas applique vu qu'il n'y a rien apres
float bnd[QLOG-1];
// les valeurs de restitution, qui tombent "au milieu" des plages, t.q. y --> ddi[y-QLIN]
int ddi[QLOG];
// la table de dequantification
int dequant[1<<QBIT];

// tmp static storage
int oldsig_d;	// predicteur du decodeur
float oldsig_e;	// predicteur de l'encodeur 

//int dequant( unsigned short code );
unsigned short quant( float diff );

// initialisation
void codec_init()
{
oldsig_d = 0;
oldsig_e = 0.0;
// calcul des plages en progression exponentielle soit une serie constituee de :
// QLIN-0.5, ddi[0], bnd[0], ddi[1], bnd[1], ... ddi[QLOG-1] soit 2*QLOG elements,
// ddi[QLOG-1] etant le plafond, determine par kplaf.
// le facteur de progression est donc :
double kexp = pow( ( ((double)target_p) * kplaf ) / ((double)(QLIN-0.5)), 1.0 / ((double)((2*QLOG)-1)) );
double _ddi, _bnd = ((double)(QLIN-0.5));
int i;
for	( i = 0; i < ( QLOG - 1 ); ++i )
	{
	_ddi = _bnd * kexp;
	_bnd = _ddi * kexp;
	ddi[i] = (int)round(_ddi);
	bnd[i] = _bnd;
	}
_ddi = _bnd * kexp;
ddi[QLOG-1] = (int)round(_ddi);
// table de dequantification
unsigned int mag, sgn, code;
for	( code = 0; code < ( 1 << QBIT ); ++code )
	{
	mag = code & codemagmask;
	sgn = code & codesgnmask;
	if	( mag >= QLIN )
		mag = ddi[mag-QLIN];
	dequant[code] = (sgn?(-((int)mag)):((int)mag));
	}
}

void codec_dump( FILE * dfil )
{
fprintf( dfil, "/* compression predictive avec perte\n" );
fprintf( dfil, "amplitude nominale target_p = %u (soit (Fck/fsamp)/4 )\n", target_p );
fprintf( dfil, "kplaf = %g, %u bits de code soit QLIN = %u, QLOG = %u\n", kplaf, QBIT, QLIN, QLOG );
fprintf( dfil, "serie exponentielle :\n");
fprintf( dfil, "\t%g\n", ((double)(QLIN-0.5)) );
int i;
for	( i = 0; i < ( QLOG - 1 ); ++i )
	{
	fprintf( dfil, "\t%u\n", ddi[i] ); 
	fprintf( dfil, "\t%g\n", bnd[i] );
	}
fprintf( dfil, "\t%u\n", ddi[QLOG-1] );
float v; unsigned short u;
fprintf( dfil, "tests :\n");
v = 2.0; u = quant( v );
fprintf( dfil, "\t%g -> %u -> %d\n", v, u, dequant[u] );
v = -2.0; u = quant( v );
fprintf( dfil, "\t%g -> %u -> %d\n", v, u, dequant[u] );
v = 100.0; u = quant( v );
fprintf( dfil, "\t%g -> %u -> %d\n", v, u, dequant[u] );
v = -200.0; u = quant( v );
fprintf( dfil, "\t%g -> %u -> %d\n", v, u, dequant[u] );
v = 666.0; u = quant( v );
fprintf( dfil, "\t%g -> %u -> %d\n", v, u, dequant[u] );
v = 1111.0; u = quant( v );
fprintf( dfil, "\t%g -> %u -> %d\n", v, u, dequant[u] );
v = 0.0; u = quant( v );
fprintf( dfil, "\t%g -> %u -> %d\n", v, u, dequant[u] );
v = 2700.0; u = quant( v );
fprintf( dfil, "\t%g -> %u -> %d\n", v, u, dequant[u] );
v = -7000.0; u = quant( v );
fprintf( dfil, "\t%g -> %u -> %d\n", v, u, dequant[u] );
fprintf( dfil, " */\n");
fprintf( dfil, "// definitions a reporter dans audio.h :\n" );
fprintf( dfil, "// 72000000 / %u = %g Hz\n", target_p * 4, 72000000.0 / ( target_p * 4.0 ) );
fprintf( dfil, "#define QBIT %d\n", QBIT );
fprintf( dfil, "#define SAMP_PERIOD  %u\n", target_p * 4 );
fprintf( dfil, "#define PWM_PERIOD   (SAMP_PERIOD/2)\n" );
fprintf( dfil, "#define PWM_SILENCE  (PWM_PERIOD/2)\n\n" );
fprintf( dfil, "const short dequant[] = {\n" ); 
for	( i = 0; i < ( ( 1 << QBIT ) - 1 ); ++i )
	{
	fprintf( dfil, "%d, ", dequant[i] );
	if	( ( i % 8 ) == 7 )
		fprintf( dfil, "\n" );
	}
// dernier element sans la virgule
fprintf( dfil, "%d };\n", dequant[i] );
}

// decodeur

int decode( unsigned short code )
{
int retval = oldsig_d + dequant[code];
oldsig_d = retval;
return retval;
}

// encodeur

// quantificateur 1, prend diff a l'echelle target
unsigned short quant( float diff )
{
int sgn = (diff >= 0.0)?(0):(1);
float mag = fabs( diff );
// zone lin : simple round
unsigned short retval = (unsigned short)round(mag);
if	( retval >= QLIN )
	{			// zone LOG : comparer a chaque bnd[]
	int i = 0;
	do	{
		if	( mag < bnd[i] )
			break;
		++i;
		} while ( i < (QLOG-1) );
	retval = QLIN + i;
	}
// on quantifie
retval &= codemagmask;	// precaution
if	( sgn )
	retval |= codesgnmask;
return retval;
}

// encodeur : prend signal a l'echelle target
unsigned short encode( float signal )
{
unsigned short retval = quant( signal - oldsig_e );
oldsig_e = (float)decode( retval );
return retval;
}

// niveau buffer entier

// #define DUMP_ZECODE

// comprimer et packer l'audio fourni dans mbuf (float normalise -1.0;1.0)
// alloue la memoire pour wbuf, rend la taille de wbuf en unsigned int
int compress2w32( unsigned int qsamp, float * mbuf, unsigned int ** pwbuf )
{
// preparation buffer pour packing des codes comprimes
unsigned qw32 = qsamp * QBIT;
qw32 = ( ( qw32 - 1 ) / 32 ) + 1;
unsigned int * wbuf = (unsigned int *)malloc( qw32 * sizeof(int) );
if	( wbuf == NULL )
	return -1;
// variables pour packing
unsigned int ic;	// indice du code
unsigned int iw;	// indice du word
unsigned int pb0;	// position du lsb du code courant dans le word courant
unsigned int zecode;	// code courant
unsigned int zew;	// word courant
// compression precedee de mise a l'echelle target_p
// packing des codes de QBIT bits dans des mots de 32 bits
mbuf[0] = 0.0;	// precaution pour eviter offset au depart
codec_init();
iw = 0; zew = 0; pb0 = 0;
for	( ic = 0; ic < qsamp; ++ic )
	{
	zecode = (unsigned int)encode( mbuf[ic] * ((float)target_p) );
	#ifdef DUMP_ZECODE
	printf("%d\n", zecode );
	#endif
	zew |= ( zecode << pb0 );
	pb0 += QBIT;
	if	( pb0 >= 32 )
		{			// passer au word suivant
		if	( iw >= qw32 )	// precaution
			break;
		wbuf[iw++] = zew;
		pb0 -= 32;
		if	( pb0 > 0 )	// traiter residu
			zew = ( zecode >> ( QBIT - pb0 ) );
		else	zew = 0;
		}
	}
// sauver dernier mot inacheve s'il y en a
if	( ( pb0 > 0 ) && ( iw < qw32 ) )
	wbuf[iw++] = zew;
if	( ( iw != qw32 ) || ( ic != qsamp ) )
	{
	printf("erreur packing iw=%u vs %u, ic=%u vs %u\n", iw, qw32, ic, qsamp );
	return -2;
	}
*pwbuf = wbuf;
return (int)qw32;
}

// depacker et decomprimer l'audio fourni dans wbuf (unsigned int)
// alloue la memoire pour mbuf (float normalise -1.0;1.0)
int uncompress2float( unsigned int qsamp, float ** pmbuf, unsigned int * wbuf, unsigned int qwbuf )
{
// preparation buffer pour audio decomprime
float * mbuf = (float *)malloc( qsamp * sizeof(float) );
if	( mbuf == NULL )
	return -1;
// calcul taille buffer w32 pour verif
unsigned qw32 = qsamp * QBIT;
qw32 = ( ( qw32 - 1 ) / 32 ) + 1;
if	( qw32 != qwbuf )
	return -3;
// variables pour unpacking
unsigned int ic;	// indice du code
unsigned int iw;	// indice du word
unsigned int pb0;	// position du lsb du code courant dans le word courant
unsigned int zecode;	// code courant
unsigned int zew;	// word courant
// extraction et decompression des codes de QBIT bits des mots de 32 bits
codec_init();
iw = 0; zew = 0; pb0 = 0;
zew = wbuf[iw++];
for	( ic = 0; ic < qsamp; ++ic )
	{
	zecode = ( zew >> pb0 );
	pb0 += QBIT;
	if	( pb0 >= 32 )
		{			// passer au word suivant
		if	( iw >= qw32 )	// precaution
			break;
		zew = wbuf[iw++];
		pb0 -= 32;
		if	( pb0 > 0 )	// traiter residu
			zecode |= ( zew << ( QBIT - pb0 ) ); 
		}
	zecode &= ( ( 1 << QBIT ) - 1 );
	mbuf[ic] = ((float)decode( zecode )) / ((float)target_p);
	}
if	( ( iw != qw32 ) || ( ic != qsamp ) )
	{
	printf("erreur depacking iw=%u vs %u, ic=%u vs %u\n", iw, qw32, ic, qsamp );
	return -2;
	}
*pmbuf = mbuf;
return 0;
}
