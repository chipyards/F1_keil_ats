/** compression predictive avec perte */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "codec.h"

// profils de codage
profile_t profils[] = {
	{		// original CHTI 11k oversamp 2
	72000000,	// FCK
	3266,		// PWM range reso 11.7 bits
	2,		// oversamp
	5,		// nombre de bits du code
	4,		// nombre de valeurs de la partie lineaire du code
	0.4,		// (max mag) / (PWMdiv/2) = coeff de limitation de la mag du code
	0, 0, 0, 0, 0
	},
	{		// 22k sans oversamp (PWM 22k)
	72000000,	// FCK
	3266,		// PWM range reso 11.7 bits
	1,		// oversamp
	5,		// nombre de bits du code
	4,		// nombre de valeurs de la partie lineaire du code
	0.4,		// (max mag) / (PWMdiv/2) = coeff de limitation de la mag du code
	0, 0, 0, 0, 0
	},
	{		// 22k oversamp 2 (PWM 44k)
	72000000,	// FCK
	1633,		// PWM range reso 10.7 bits
	2,		// oversamp
	5,		// nombre de bits du code
	4,		// nombre de valeurs de la partie lineaire du code
	0.4,		// (max mag) / (PWMdiv/2) = coeff de limitation de la mag du code
	0, 0, 0, 0, 0
	}
};

profile_t * P = NULL;

#define QMAX 256	// QBIT <= 8 ==> (1<<QBIT) <= 256 ==> QCODE < 256 ==> QLOG < 256
// les limites de plages cote diff source, t.q. ( bnd[n-1] <= x < bnd[n] ) ==> y = QLIN + n  
// cas limite inferieur				( QLIN-0.5 <= x < bnd[0] ) ==> y = QLIN
// cas limite superieur : bnd[QLOG-1] n'est pas applique vu qu'il n'y a rien apres
float bnd[QMAX];		// float bnd[QLOG-1];
// les valeurs de restitution, qui tombent "au milieu" des plages, t.q. y --> ddi[y-QLIN]
int ddi[QMAX];			// int ddi[QLOG];
// la table de dequantification
int dequant[QMAX];		// int dequant[1<<QBIT];

// tmp static storage
int oldsig_d;	// predicteur du decodeur
float oldsig_e;	// predicteur de l'encodeur 

//int dequant( unsigned short code );
unsigned short quant( float diff );

// initialisation
void codec_init( int ip )
{
oldsig_d = 0;
oldsig_e = 0.0;
// profil
P = &profils[ip];
// parametres derives
if	( P->QBIT > 8 )
	printf("WARNING P->QBIT > 8 unsupported\n");
P->target_pk = P->PWMdivider / 2;	// amplitude peak sur target
P->QCODE     = 1 << (P->QBIT-1);	// nombre de valeurs de la magnitude du code
P->QLOG      = P->QCODE - P->QLIN;	// nombre de bits de la partie "log" du code
P->codemagmask = P->QCODE-1;		// mask pour la magnitude du code
P->codesgnmask = P->QCODE;		// mask pour le bit de signe du code
// calcul des plages en progression exponentielle soit une serie constituee de :
// QLIN-0.5, ddi[0], bnd[0], ddi[1], bnd[1], ... ddi[QLOG-1] soit 2*QLOG elements,
// ddi[QLOG-1] etant le plafond, determine par kplaf.
// le facteur de progression est donc :
double kexp = pow( ( ((double)P->target_pk) * P->kplaf ) / ((double)(P->QLIN-0.5)), 1.0 / ((double)((2*P->QLOG)-1)) );
double _ddi, _bnd = ((double)(P->QLIN-0.5));
int i;
for	( i = 0; i < ( P->QLOG - 1 ); ++i )
	{
	_ddi = _bnd * kexp;
	_bnd = _ddi * kexp;
	ddi[i] = (int)round(_ddi);
	bnd[i] = _bnd;
	}
_ddi = _bnd * kexp;
ddi[P->QLOG-1] = (int)round(_ddi);
// table de dequantification
unsigned int mag, sgn, code;
for	( code = 0; code < ( 1 << P->QBIT ); ++code )
	{
	mag = code & P->codemagmask;
	sgn = code & P->codesgnmask;
	if	( mag >= P->QLIN )
		mag = ddi[mag-P->QLIN];
	dequant[code] = (sgn?(-((int)mag)):((int)mag));
	}
}

void codec_dump( FILE * dfil )
{
fprintf( dfil, "/* compression predictive avec perte\n" );
fprintf( dfil, "PWM resolution = %d\n", P->PWMdivider );
fprintf( dfil, "amplitude nominale target_pk = %d\n", P->target_pk );
fprintf( dfil, "frequence horloge = %d Hz\n", P->FCK );
fprintf( dfil, "frequence PWM = %g Hz\n", (double)P->FCK / (double)P->PWMdivider );
fprintf( dfil, "frequence audio = %g Hz\n", ( (double)P->FCK / (double)P->PWMdivider ) / (double)P->Koversamp );
fprintf( dfil, "kplaf = %g, %u bits de code soit QLIN = %u, QLOG = %u\n", P->kplaf, P->QBIT, P->QLIN, P->QLOG );
fprintf( dfil, "serie exponentielle :\n");
fprintf( dfil, "\t%g\n", ((double)(P->QLIN-0.5)) );
int i;
for	( i = 0; i < ( P->QLOG - 1 ); ++i )
	{
	fprintf( dfil, "\t%u\n", ddi[i] ); 
	fprintf( dfil, "\t%g\n", bnd[i] );
	}
fprintf( dfil, "\t%u\n", ddi[P->QLOG-1] );
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
fprintf( dfil, "// frequence audio = %g Hz\n", ( (double)P->FCK / (double)P->PWMdivider ) / (double)P->Koversamp );
fprintf( dfil, "#define QBIT %d\n", P->QBIT );
fprintf( dfil, "#define PWM_SILENCE %d\n", P->target_pk );
fprintf( dfil, "#define PWM_PERIOD  %d\n", P->PWMdivider );
fprintf( dfil, "#define SAMP_PERIOD %d\n", P->PWMdivider * P->Koversamp );
fprintf( dfil, "#define KOVER %d // <-- OVERSAMPLING\n", P->Koversamp );
fprintf( dfil, "const short dequant[] = {\n" ); 
for	( i = 0; i < ( ( 1 << P->QBIT ) - 1 ); ++i )
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
if	( retval >= P->QLIN )
	{			// zone LOG : comparer a chaque bnd[]
	int i = 0;
	do	{
		if	( mag < bnd[i] )
			break;
		++i;
		} while ( i < (P->QLOG-1) );
	retval = P->QLIN + i;
	}
// on quantifie
retval &= P->codemagmask;	// precaution
if	( sgn )
	retval |= P->codesgnmask;
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
unsigned qw32 = qsamp * P->QBIT;
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
// compression precedee de mise a l'echelle target_pk
// packing des codes de QBIT bits dans des mots de 32 bits
mbuf[0] = 0.0;	// precaution pour eviter offset au depart
iw = 0; zew = 0; pb0 = 0;
for	( ic = 0; ic < qsamp; ++ic )
	{
	zecode = (unsigned int)encode( mbuf[ic] * ((float)P->target_pk) );
	#ifdef DUMP_ZECODE
	printf("%d\n", zecode );
	#endif
	zew |= ( zecode << pb0 );
	pb0 += P->QBIT;
	if	( pb0 >= 32 )
		{			// passer au word suivant
		if	( iw >= qw32 )	// precaution
			break;
		wbuf[iw++] = zew;
		pb0 -= 32;
		if	( pb0 > 0 )	// traiter residu
			zew = ( zecode >> ( P->QBIT - pb0 ) );
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
unsigned qw32 = qsamp * P->QBIT;
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
iw = 0; zew = 0; pb0 = 0;
zew = wbuf[iw++];
for	( ic = 0; ic < qsamp; ++ic )
	{
	zecode = ( zew >> pb0 );
	pb0 += P->QBIT;
	if	( pb0 >= 32 )
		{			// passer au word suivant
		if	( iw >= qw32 )	// precaution
			break;
		zew = wbuf[iw++];
		pb0 -= 32;
		if	( pb0 > 0 )	// traiter residu
			zecode |= ( zew << ( P->QBIT - pb0 ) ); 
		}
	zecode &= ( ( 1 << P->QBIT ) - 1 );
	mbuf[ic] = ((float)decode( zecode )) / ((float)P->target_pk);
	}
if	( ( iw != qw32 ) || ( ic != qsamp ) )
	{
	printf("erreur depacking iw=%u vs %u, ic=%u vs %u\n", iw, qw32, ic, qsamp );
	return -2;
	}
*pmbuf = mbuf;
return 0;
}
