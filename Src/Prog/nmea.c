/* petite lib pour le decodage des sentences NMEA du GPS NEO 8 (et autres) 2023
   compiler avec -DSTANDALONE pour test sur PC (MinGW)

   ce parseur presente de nombreuses vulnerabilites aux donnees incorrectes :
	ivar peut prendre la valeur -1 mais etre quand meme utilise comme index
	les fields de longueur fixe  '2' ou '3' peuvent avoir une longueur zero (bug de la spec NMEA)
	ce prog sait en sauter 1 mais pas 2
   proposition d'amelioration : traiter les fields de longueur fixe comme des sous-fields
	- separer les fields seulements par les ',',
	- stocker en texte temporaire les fields de type "complexes" contenant des sous-fields
	- utiliser un sous-parseur specifique par type "complexe" pour decoder les fields "complexes"
	- mettre les types dans des templates en ROM plutot que dans les variables
*/

#include "nmea.h"
// #include <stdio.h>

// initialisation des variables
void gps_var_init( nmea_ctx * ctx )
{
int iva;

ctx->data[0].type = '2';	// hour
ctx->data[1].type = '2';	// mn
ctx->data[2].type = 'f';	// sec
ctx->data[3].type = 'a';	// receiver status (A=ok?)
ctx->data[4].type = '2';	// latitude degres
ctx->data[5].type = 'f'; 	// latitude minutes
ctx->data[6].type = 'a'; 	// lat. sign
ctx->data[7].type = '3'; 	// longitude degres
ctx->data[8].type = 'f'; 	// longitude minutes
ctx->data[9].type = 'a'; 	// long. sign
ctx->data[10].type = 'f';	// knots
ctx->data[11].type = 'f';	// track made good degrees
ctx->data[12].type = '2';	// day
ctx->data[13].type = '2';	// month
ctx->data[14].type = 'd';	// year
ctx->data[15].type = 'd';	// GPS qualite (1=good)
ctx->data[16].type = 'd';	// number of satellites
ctx->data[17].type = 'f' ;	// HDOP
ctx->data[18].type = 'f' ;	// VDOP
ctx->data[19].type = 'f';	// Altitude (over sea level)

//ctx->data[].type = '';	//

ctx->qdata = 20;
for	( iva = 0; iva < ctx->qdata; ++iva )
	ctx->data[iva].stat = 1;	// 1 <==> undefined
}

// template des fields de chaque sentence supportee : un index de variable pour chaque field,
// ou -1 s'il n'est pas destine a etre stocke dans une variable
// N.B. le -1 n'est permis que pour les variables avec delimiteur : a, d, f
const char fields_NGGA[] = {
0,	// hour
1,	// mn
2,	// sec
4,	// latitude degres
5,	// latitude minutes
6,	// lat. sign
7,	// longitude degres
8,	// longitude minutes
9,	// long. sign
15,	// GPS qualite (1=good)(2=?)
16,	// number of satellites
17,	// HDOP
19,	// Altitude (over sea level)
};

const char fields_NRMC[] = {
0,	// hour
1,	// mn
2,	// sec
3,	// status
4,	// latitude degres
5,	// latitude minutes
6,	// lat. sign
7,	// longitude degres
8,	// longitude minutes
9,	// long. sign
10,	// knots
11,	// track made good degrees
12,	// day
13,	// month
14,	// year
-1,	// magnetic var
-1,	// magnetic sign 
};


void new_sentence( nmea_ctx * ctx )
{
ctx->qfields = 0;
ctx->ifield = -1;	// le preambule style $GNRMC ne compte pas pour un field
ctx->ichar = 0;
ctx->status = 0;	// 0 = empty, 1 = in progress, 42 = checksum Ok
}

static void next_field( nmea_ctx * ctx )
{
++ctx->ifield;
if	( ctx->ifield < ctx->qfields )
	{
	ctx->ivar = ctx->fields[ctx->ifield];
	if	( ctx->ivar < ctx->qdata )
		ctx->tfield = ctx->data[ctx->ivar].type;
	else	ctx->tfield = 0;	// not to be stored
	if	( ctx->tfield == 'f' )
		ctx->data[ctx->ivar].frac = (char)-99;
	}
else	{
	ctx->ivar = -1;				// an extra field
	ctx->tfield = 0;
	}
ctx->ichar = 0;
}

void invalidate_sentence( nmea_ctx * ctx )
{
unsigned int ifi, iva;
for	( ifi = 0; ifi < ctx->qfields; ++ifi )
	{
	iva = ctx->fields[ifi];
	if	( iva >= 0 )
		ctx->data[iva].stat = 2;
	}
}

// traitement d'un char
// description de la FSM
// 	ifield	index of the current/next field
//		-1 attente $ (si ichar == 0 ), ou preambule en cours
//		0 .. (qfields-1) regular field
//	ichar	index of current char in field
//	ivar	index of current variable in data[]
//	status	0 = empty, 1 = in progress, 42 = checksum Ok, 43 = checksum Bad
// ATTENTION: quand on arrive a 42 ou 43 il faut reinitialiser avant d'appeler nmea_proc a nouveau
// N.B. une sentence non supportee va etre parcourue en laissant le systeme en attente d'un autre '$'
// i.e. ifield = -1 avec ichar = 0 et status = 0
void nmea_proc( nmea_ctx * ctx, char c )
{
if	( ctx->ifield == -1 )	// preambule ou attente preambule
	{
	if	( ctx->ichar == 0 )
		{ if ( c == '$' ) ++ctx->ichar; ctx->checksum = 0; ctx->status = 1; }
	else if ( ctx->ichar == 1 )
		{ if ( c == 'G' ) { ++ctx->ichar; ctx->fourcc = 0; } else ctx->ichar = 0; }
	else if ( ctx->ichar <= 5 )
		{ ctx->fourcc |= (int)c << ( 8 * ( ctx->ichar - 2 ) ); ++ctx->ichar; }
	else if ( ctx->ichar == 6 )
		{
		if	( c == ',' )
			{
			switch	( ctx->fourcc )
				{
				case NRMC :
				case PRMC :	ctx->fields = fields_NRMC;
						ctx->qfields = sizeof(fields_NRMC);
						next_field( ctx ); break;
				case NGGA :
				case PGGA :	ctx->fields = fields_NGGA;
						ctx->qfields = sizeof(fields_NGGA);
						next_field( ctx ); break;
				default   :	ctx->status = 0;
				}
			}
		ctx->ichar = 0;
		}
	if	( c != '$' )
		ctx->checksum ^= c;
	}
else if	( ctx->ifield >= 0 )	// data field
	{
	if	( ( c == ',' ) || ( c == '*' ) )		// normal delimited end
		{
		if	( ctx->tfield )
			{
			if	( ctx->ichar == 0 )		// cas d'un field vide
				{
				ctx->data[ctx->ivar].stat = 1;
				if	( ( ctx->data[ctx->ivar].type == '2' ) || ( ctx->data[ctx->ivar].type == '3' ) )
					{			// cas vicieux d'un field sans delimiteur qu'il faut sauter
					next_field( ctx );
					}
				}
			else	{ ctx->data[ctx->ivar].stat = 0; ctx->data[ctx->ivar].val = ctx->tmp; } 
			}
		if	( c == '*' )
			{ ctx->ifield = -2; ctx->ichar = 0; }
		else	next_field( ctx );
		}
	else if	(						// normal counted end
			( ( ctx->tfield == '2' ) && ( ctx->ichar == 1 ) ) ||
			( ( ctx->tfield == '3' ) && ( ctx->ichar == 2 ) )
		)
		{
		ctx->tmp = ctx->tmp * 10 + ( c - '0');
		if	( ctx->tfield )
			{ ctx->data[ctx->ivar].stat = 0; ctx->data[ctx->ivar].val = ctx->tmp; } 
		next_field( ctx );
		}
	else if ( c < ' ' )
		{						// premature end of line
		invalidate_sentence( ctx );
		new_sentence( ctx );
		}
	else if	( c == '.' )					// decimal point
		{ 
		if	( ctx->tfield )
			ctx->data[ctx->ivar].frac = 0;
		++ctx->ichar;
		}
	else	{						// normal data digit
		if	( ctx->tfield == 'a' )
			ctx->tmp = c;
		else if	( ctx->ichar == 0 )
			ctx->tmp = c - '0';
		else	ctx->tmp = ctx->tmp * 10 + ( c - '0');
		++ctx->ichar;
		if	( ctx->tfield )
			++ctx->data[ctx->ivar].frac;
		}
	if	( c != '*' )
		ctx->checksum ^= c;
	}
else if	( ctx->ifield == -2 )
	{				// checksum
	c -= '0';
	if ( c > 9 ) c -= ('A' - '9' - 1 );
	if	( ctx->ichar == 0 )
		{ ctx->tmp = c; ++ctx->ichar; }
	else if	( ctx->ichar == 1 )
		{
		ctx->tmp = ( ctx->tmp << 4 ) + c;
		if	( ctx->tmp == ctx->checksum )
			ctx->status = 42;	// normal end of sentence
		else	ctx->status = 43;	// bad checksum end of sentence
		}
	}
}

// conversion des minutes decimales en microdegres
// fmin : representation entiere, frac : nombre de digits apres la virgule
int min2microdeg( int fmin, int frac )
{
if	( frac == 5 )
	return( fmin / 6 );
else if	( frac == 4 )
	return( ( fmin * 10 ) / 6 );
else	return 0;			// erreur
}

#ifdef STANDALONE
/* ---------- standalone demo : give a sentence as CLI 'argument' ----------- */
// gcc -Wall -o nmea.exe -Wno-char-subscripts -DSTANDALONE nmea.c

#include <string.h>
#include <stdio.h>

int vardump( nmea_ctx * ctx, int ivar )
{
double d;
printf("var %d: ", ivar );
gps_var * v = &ctx->data[ivar];
if	( v->stat == 1 )
	{ printf("undef\n"); return 1; }
else if ( v->stat != 0 )
	{ printf("invalid\n"); return 2; }
printf("'%c' ", v->type );
switch	( v->type )
	{
	case 'a' : // ascii, single char, delim ',' or '*'
		printf("%c", (char)v->val );
		break;
	case 'd' : // decimal, variable length, delim ',' or '*'
	case '2' : // decimal, 2-digits, no delim
	case '3' : // decimal, 3-digits, no delim
		printf("%d", (int)v->val );
		break;
	case 'f' : // floating point,  delim ',' or '*'
		d = (double)v->val;
		for	( int p = 0; p < v->frac; p++ )
			d /= 10.0;
		printf("%g", d );
		break;
	}
printf("\n");
return 0;
}

void fourccdump( unsigned int fourcc )
{
fourcc >>= 8;
printf("%s\n", (char *)&fourcc );
}


int main( int argc, char ** argv )
{
nmea_ctx lectx;		// we allocate the context here, it has a pointer to the variables
#define CTX (&lectx)	// and only scalar variables
gps_var lesvars[QVAR];	// we allocate the variables here
CTX->data = lesvars; 	// then we make the context aware of it
gps_var_init( CTX );	// we initialize the variables we are interested in

if	( argc < 2 )
	return 1;

char * lasentence = argv[1];

// traiter la sentence caractere par caractere
// c'est au programme appelant de detecter le terminateur t.q. CRLF
// et alors de tester le status et appeler new_sentence(CTX);
// Mais ici dans cette demo on utilise strlen
new_sentence(CTX);
for	( unsigned int i = 0; i < strlen(lasentence); i++ )
	{
	nmea_proc( CTX, lasentence[i] );
	printf("char %d (%c) ==> field %d char %d -> var %d (tmp=%d)\n", i, lasentence[i], CTX->ifield , CTX->ichar , CTX->ivar, CTX->tmp ); fflush(stdout);
	if	( CTX->status == 42 )
		{
		fourccdump( CTX->fourcc );
		printf("<<checksum ok>>\n");
		// a ce point les variables repertoriees pour cette sentence sont a jour
		for	( int i = 0; i < CTX->qdata; i++ )
			{
			vardump( CTX, i ); fflush(stdout);
			}
		if	( CTX->data[16].stat == 0 )
			printf("number of sats : %u\n", CTX->data[16].val );
		if	( ( CTX->data[4].stat == 0 ) && ( CTX->data[5].stat == 0 ) && ( CTX->data[6].stat == 0 ) )
			{
			int microlat = CTX->data[4].val * 1000000 + min2microdeg( CTX->data[5].val, CTX->data[5].frac );
			if	( CTX->data[6].val == 'S' )
				microlat = -microlat;
			printf("latitude %d ud\n", microlat );
			}
		if	( ( CTX->data[7].stat == 0 ) && ( CTX->data[8].stat == 0 ) && ( CTX->data[9].stat == 0 ) )
			{
			int microlat = CTX->data[7].val * 1000000 + min2microdeg( CTX->data[8].val, CTX->data[8].frac );
			if	( CTX->data[9].val == 'W' )
				microlat = -microlat;
			printf("longitude %d ud\n", microlat );
			}
		if	( CTX->data[10].stat == 0 )
			{
			double s = (double)CTX->data[10].val;
			for	( int p = 0; p < CTX->data[10].frac; p++ )
				s /= 10.0;
			printf("%.3f knots\n", s );
			}
		if	( ( CTX->data[0].stat == 0 ) && ( CTX->data[1].stat == 0 ) && ( CTX->data[2].stat == 0 ) )
			{
			double s = (double)CTX->data[2].val;
			for	( int p = 0; p < CTX->data[2].frac; p++ )
				s /= 10.0;
			printf("%02uh%02umn%g\n", CTX->data[0].val, CTX->data[1].val, s );
			}
		return 0;
		}
	else if	( CTX->status == 43 )
		{
		fourccdump( CTX->fourcc );
		printf("<<bad checksum>>\n");
		return 1;
		}
	}
if	( CTX->ifield != -1 )
	fourccdump( CTX->fourcc );
printf("<<incomplete or corrupted sentence>>\n");
return 2;
}
#endif

