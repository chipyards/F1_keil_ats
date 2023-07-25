#include <stdio.h>
#include <stdlib.h>
#include "cli_parse.h"
/*
g++ -Wall -o CRC16_jln -O2 CRC16_jln.cpp
   ici on va faire des experiences autour du CRC CCITT-16, polynome X16 + X12 + X5 + X0
   pour lequel on a 3 codes de reference :
	- spec et middleware SDCard
	- code ppp
	- code mifare selon Adam Laurie (rfidiot)
   et tenter de clarifier les notations, le lien entre le LFSR et le code, et l'implementation acceleree par table.

- schema hardware classique : (+) = porte XOR a 2 entrees

	      0  __   1  __   2  __   3  __   4  __       5  __   6  __   7  __   8  __   9  __   10 __   11 __       12 __   13 __   14 __   15 __   16 
	     X  |  | X  |  | X  |  | X  |  | X  |  |     X  |  | X  |  | X  |  | X  |  | X  |  | X  |  | X  |  |     X  |  | X  |  | X  |  | X  |  | X
       0-(+)--->|__|--->|__|--->|__|--->|__|--->|__|-(+)--->|__|--->|__|--->|__|--->|__|--->|__|--->|__|--->|__|-(+)--->|__|--->|__|--->|__|--->|__|---|
          |                                           |                                                           |                                    |
          ^-----------------------<-------------------^--------------------------<--------------------------------^--<--(+)--<-------------------------|
                                                                                                                         |
                                                                                                                 input---^

- schema software : decalage puis bitwise XOR conditionnel

	  0  ___   1  ___   2  ___   3  ___   4  ___   5  ___   6  ___   7  ___   8  ___   9  ___   10 ___   11 ___   12 ___   13 ___   14 ___   15 ___   16 
	 X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X  |   | X
	0-->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--->|___|--|
              |                                            |                                                              |                               |
	      b0                                           b5                                                             b12                             |
              |                                            |                                                              |                               |
              ^-----------------------<--------------------^-----------------------------<--------------------------------^--<--(+)------<----------------|
                                                                                                                                 |
                                                                                                                         input---^

	mask: 1        0        0        0        0        1        0        0        0        0        0        0        1        0        0        0  

	1) le registre contenant le CRC en cours est decale vers les exposants croissants
		- le bit ejecte cote fort exposant est recupere
		- un zero est insere cote faible exposant
	2) le XOR du bit recupere avec le bit d'entree est calcule
	3) si le resultat est 1, un bitwise xor est applique au registre sur les bits qui auraient reçu la sortie d'un XOR dans la version hardware
  Alors on doit introduire une convention de direction :
	- BIG : si les indices des bits sont dans l'ordre des exposants du polynome (ce qui semble evident), alors le decalage est LEFT
	  (et la figure est à l'envers, dommage) et le mask est 0b_0001_0000_0010_0001 = 0x1021
		- le LSB du mask est toujours 1 (correspond au terme 1 (degre 0) qui est toujours present dans le polynome)
		- on doit capturer le MSB du CRC avant le shift (si le mot machine est rempli, il faut sauver ce bit avant le shift)
	  Cette direction peut etre qualifiee de big-endian, car dans le CRC, le bit le plus "ancien" est le MSB, et le premier bit du message
	  est le MSB du message (correspondant au terme de plus fort exposant du polynome associe au message)
	  Si le message est traite par bytes, ceux-ci doivent etre traites en big-endian i.e. shiftes a gauche
	- LITTLE : si les indices des bits sont dans l'ordre inverse des exposants du polynome, alors le decalage est RIGHT
	  (ok avec l'orientation de la figure) et le mask est 0b_1000_0100_0000_1000 = 0x8408
		- le MSB du mask est toujours 1 (correspond au terme 1 qui est toujours present dans le polynome)
		- on doit capturer le LSB du CRC avant le shift (ce qui est facile, d'ou la popularite de cette implementation) 
	  Cette direction peut etre qualifiee de little-endian, car dans le CRC, le premier bit du message est le LSB du message
	Compatibilite :
		si :
			- on change la direction de l'algo
			- on renverse les bits de chaque byte du message (sans changer l'ordre des bytes)
		alors
			- les 16 bits du CRC sont renverses

- reinjection :
	si on append le CRC du corps du message apres celui (avec la bonne endianness) on obtient zero !!! (le "residue" chez reveng)
	les "codewords" de reveng ont ce CRC appendu.
	En plus c'est vrai quelle que soit la valeur du CRC init (evident si on considere que ce crc resulte d'une portion de message deja traitee)

- Notation Koopman : utilise un mot binaire fait en associant a chaque terme du polynome un bit d'indice egal a l'exposant
  soit 0b1_0001_0000_0010_0001 ensuite il enleve le LSB (terme 1 (degre 0) toujours present) soit 0b1000_1000_0001_0000 = 0x8810
  N.B. les notations koopman et little endian mask ont le MSB a 1 ce qui indique aussi la taille du CRC, le mask little endian ne l'indique pas.

- links
	https://en.wikipedia.org/wiki/Cyclic_redundancy_check
	https://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
	https://reveng.sourceforge.io/crc-catalogue/all.htm
	koopman :
	https://users.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
	http://users.ece.cmu.edu/~koopman/crc/index.html
	https://www.youtube.com/watch?v=qRqvdOAfxcA

- commentaires sur les links
	koopman designe les polynomes par un mask avec 1 bit par terme sauf le terme 1 (degre zero) big-endian e.g. 8810
	koopman analyse la performance de nombreux polynomes pour des messages courts ou moyens, mais seulement pour
	des BERs (Bit Error Rate) faibles t.q. 1e-6, pour des BERs superieurs: "you need to understand more before using these polynomials"
	reveng notations :
		polynom : big endian mask, right justified "highest-order term is omitted" (e.g. 1021)
		refin : true <==> little endian process (right shift)
		refout : idem, pour le traitement du resultat (a verifier) 
	reveng cree plusieurs entrees pour le meme polynome, si l'initialisation, l'ordre ou le xor final different
	reveng donne :
		un check "check" qui est le CRC de la string ascii "123456789",
		des codewords trouves dans la litterature, qui sont des data suivis du crc appendu sous forme de bytes,
		un enorme soft pour tout simuler

*/
// utilitaires
// conversion texte hexa (byte sequence) en binaire
unsigned int hex2buf( unsigned char * buf, unsigned int size, const char * hex )
{
unsigned int i = 0, j = 0, k = 0;
char hexbyte[3]; char c;
while	( j < size )
	{ 
	if	( ( c = hex[i++] ) == 0 ) break;
	else if (		// skip naughty unicode shit in reveng's page
		( ( c >= '0' ) && ( c <= '9' ) ) ||
		( ( c >= 'a' ) && ( c <= 'f' ) ) ||
		( ( c >= 'A' ) && ( c <= 'F' ) )
		)
		hexbyte[k++] = c;
	if	( k >= 2 )
		{
		hexbyte[2] = 0;
		buf[j++] = (unsigned char)strtoul( hexbyte, NULL, 16 );
		k = 0;
		}
	}
return j;
}
// dump binaire en texte hexa (byte sequence)
void dumpbuf( const unsigned char * buf, unsigned int size )
{
for	( unsigned int i = 0; i < size; i++ )
	printf("%02X", buf[i] );
printf("\n"); fflush(stdout);
}


// classe de base 
class crc_base {
public:
unsigned int crc;
unsigned int initcrc;

crc_base() : crc( 0 ) {};

virtual void init() { crc = initcrc; }

// traiter 1 bit
virtual void crc_step( unsigned int bit ) { };

// traiter 1 byte
virtual void crc_8steps( unsigned int b ) = 0;

// traiter un buffer
unsigned int crc_Nbytes( const unsigned char *data, unsigned int len )
	{
        for	( unsigned int i = 0; i < len ; ++i )
                crc_8steps( data[i] );
	return crc;
	};

// reinjecter le CRC 
virtual unsigned int crc_reinj() = 0;
unsigned int crc_reinj( bool little )
	{
	int little_end = crc & 0xFF;
	int big_end    = ( crc >> 8 ) & 0xFF;
	if	( little )
		{ crc_8steps( little_end ); crc_8steps( big_end ); }
	else	{ crc_8steps( big_end ); crc_8steps( little_end ); }
	return crc;
	};

}; // class

// little endian (right shift) implementation
// from some forgotten ppp code
class pppR : public crc_base {
public:

pppR() : crc_base() { initcrc = 0; };

void crc_step( unsigned int bit )
	{
	if	( ( crc ^ bit ) & 1 )
		crc = ( crc >> 1 ) ^ 0x8408;
	else	crc >>= 1;
	};

void crc_8steps( unsigned int b )
	{
	for	( unsigned int i = 0; i < 8; i++ )
		{
		crc_step( b & 1 );
		b >>= 1;
		}
	};

unsigned int crc_reinj() { return crc_base::crc_reinj( true ); };

}; // class

// big endian (left shift) translation of pppR
class pppL : public crc_base {
public:

pppL() : crc_base() { initcrc = 0; };

void crc_step( unsigned int bit )
	{
	if	( ( ( crc >> 15 ) ^ bit ) & 1 )
		crc = ( crc << 1 ) ^ 0x1021;
	else	crc <<= 1;
	crc &= 0xffff;
	};

void crc_8steps( unsigned int b )
	{
	for	( unsigned int i = 0; i < 8; i++ )
		{
		crc_step( ( b >> 7 ) & 1 );
		b <<= 1;
		}
	};

unsigned int crc_reinj() { return crc_base::crc_reinj( false ); };

}; // class

// code from Adam Laurie http://rfidiot.org/crc16.c for mifare (ISO-IEC-14443-3-A)
// CRC for AABBCCDDEE001122 is 7B09 in 2 bytes or 097B in one 16-bit word
// attention : valeur initiale 0x6363 ! (chez reveng, init=0xc6c6, same, left shifted)
class adam : public crc_base {
public:

adam() : crc_base() { initcrc = 0x6363; };

void crc_8steps( unsigned int c )   
	{
        unsigned int v, tcrc = 0;
        v = (crc ^ c) & 0xff;
        for	( unsigned int i = 0; i < 8; i++ ) 
                {
                tcrc = ( (tcrc ^ v) & 1 ) ? ( ( tcrc >> 1 ) ^ 0x8408 ) : ( tcrc >> 1 );
                v >>= 1;
                }
        crc = ( (crc >> 8) ^ tcrc ) & 0xffff;
	};

unsigned int crc_reinj() { return crc_base::crc_reinj( true ); };

}; // class

// code SDCard pour microcontroleur, par Neil Thiessen
class SDCard : public crc_base {
public:

const unsigned short m_CRC16Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

SDCard() : crc_base() { initcrc = 0; };

void crc_8steps( unsigned int b )
	{
        crc = ( crc << 8 ) ^ m_CRC16Table[ ( ( crc >> 8 ) ^ b ) & 0xFF ];
	crc &= 0xFFFF;
	};

unsigned int crc_reinj() { return crc_base::crc_reinj( false ); };

}; // class


int main( int argc, char ** argv )
{
// singletons
adam ladam;
pppR lepppR;
pppL lepppL;
SDCard laSD;
// pointeur sur le singleton courant
crc_base * C = NULL;
// message
unsigned char mbuf[512];
unsigned int msize = 0;

// 	parsage CLI
const char * valkeys = "iH";
cli_parse * lepar = new cli_parse( argc, (const char **)argv, valkeys ); 
// le parsage est fait, on recupere les args !
const char * hexbuf = NULL;
const char * val;
bool opt_reinj = false;
unsigned int opt_force_val = 0;
bool opt_force = false;

// choix algo
if	( ( val = lepar->get( 'a' ) ) ) C = &ladam;
else if	( ( val = lepar->get( 'p' ) ) ) C = &lepppR;
else if	( ( val = lepar->get( 'q' ) ) ) C = &lepppL;
else if	( ( val = lepar->get( 's' ) ) ) C = &laSD;
// test message

if	( ( val = lepar->get( 'S' ) ) )
	{					// SD Card idiot test
	for	( int i = 0; i < 512; i++ )
		mbuf[i] = 0xFF;
	msize = 512;
	}
else if	( ( val = lepar->get( 'A' ) ) )
	{					// Laurie's test
	// char x[] = {0xAA,0xBB,0xCC,0xDD,0xEE,0x00,0x11,0x22};
	msize = hex2buf( mbuf, sizeof(mbuf), "AABBCCDDEE001122" );
	dumpbuf( mbuf, msize );
	}
else if	( ( val = lepar->get( 'H' ) ) )
	{
	msize = hex2buf( mbuf, sizeof(mbuf), val );
	dumpbuf( mbuf, msize );
	}
else	{		// default		// reveng's test aka "check"
	snprintf( (char *)mbuf, sizeof(mbuf), "123456789" );
	msize = 9;
	}
// options
if	( ( val = lepar->get( 'i' ) ) )
	{
	opt_force_val = (unsigned int)strtoul( val, NULL, 16 );
	opt_force = true;
	}
if	( ( val = lepar->get( 'r' ) ) ) opt_reinj = true;
if	( ( val = lepar->get( 'h' ) ) )
	{
	printf( "options :\n"
	"-a	  : Adam Laurie's algo\n"
	"-p	  : ppp algo, little endian\n"
	"-q	  : ppp algo, big endian\n"
	"-A	  : Laurie's test AABBCCDDEE001122\n"
	"-S	  : SD Card idiot test 512 FF's\n"
	"-H <hex> : arbitrary message\n"
	"-i <hex> : initial CRC value override\n" 
	"-r	  : reinjection du CRC en fin de message\n"
	"-h	  : this help\n");
	return 0;
	}
if	( hexbuf == NULL )
	hexbuf = lepar->get( '@' );	// get avec la clef '@' rend la chaine nue 

if	( C )
	{
	C->init();
	if	( opt_force )
		{
		C->crc = opt_force_val;
		printf("init CRC = %04X\n", C->crc ); 		
		}
	C->crc_Nbytes( mbuf, msize );
	printf("%04X\n", C->crc );
	if	( opt_reinj )
		{
		C->crc_reinj();
		printf("reinject. CRC -> %04X\n", C->crc );
		}
	}
} // main

/* resume des verifications :
#### classe pppR, CRC-16/KERMIT chez reveng ####
$ ./CRC16_jln -p
	2189	Ok, check selon reveng
$ ./CRC16_jln -p -r -H 43AED6C8​ADD65143​1551B031​02D332B9​C1D65131​3732B583​
	03F3
	reinject. CRC -> 0000	Ok, codeword de reveng
$ ./CRC16_jln -p    -H 43AED6C8​ADD65143​1551B031​02D332B9​C1D65131​3732B583​F303
	0000	Ok, codeword residue selon reveng's
$ ./CRC16_jln -p -S
	85FE	note : bit-reversed result given for SDCard which is 7FA1

#### classe pppL, like CRC-16/IBM-3740 chez reveng, but crc init 0 ####
$ ./CRC16_jln -q -i FFFF
	init CRC = FFFF
	29B1	Ok, check selon reveng
$ ./CRC16_jln -q -i FFFF -H F20183D3​74
	init CRC = FFFF
	0000	Ok, codeword residue selon reveng
		Note : JL a invente cet algo en inversant le shift de pppR,
		sans savoir que c'etait celui du floppy disk IBM.
$ ./CRC16_jln -q -S
	7FA1	note : result given for SDCard
		Note : a comparer avec le code SDCard (pas encore inclus ici)

#### classe adam, CRC-16/ISO-IEC-14443-3-A chez reveng ####
$ ./CRC16_jln -a
	BF05	Ok, check selon reveng
		Note : crc init = 6363 (celui de adam) vs c6c6 chez reveng
$ ./CRC16_jln -a -A
	097B	ok, check suggere par le post d'adam
$ ./CRC16_jln -a -H AABBCCDD​EE001122​7B09
	0000	ok, meme check sous forme de codeword chez reveng
$ ./CRC16_jln -a -i 0000
	init CRC = 0000
	2189	ok, idem pppR quand on enleve le crc init specifique
$ ./CRC16_jln -a -i 0000 -H 43AED6C8​ADD65143​1551B031​02D332B9​C1D65131​3732B583​F303
	init CRC = 0000
	0000	ok, idem pppR confirme avec un codeword de CRC-16/KERMIT chez reveng
		Note : ce qui est interessant c'est que le code est different, adam
		calcule le CRC de chaque byte puis l'integre dans le CRC du message, comme
		dans les solutions tabulaires

#### classe SDCard, like CRC-16/IBM-3740 chez reveng, but crc init 0 ####
$ ./CRC16_jln -s -S
	7FA1	Ok, test result for SDCard in official spec, which does not check data bit order
$ ./CRC16_jln -s -i FFFF
	init CRC = FFFF
	29B1	Ok, check selon reveng pour IBM-3740
$ ./CRC16_jln -s -i FFFF -H F20183D3​74
	init CRC = FFFF
	0000	Ok, codeword residue selon reveng pour IBM-3740
$ ./CRC16_jln -q
	31C3	exemple pppL
$ ./CRC16_jln -s
	31C3	Ok, same as pppL
		Note : ce qui est interessant, c'est que les implementations sont differentes :
			- pppL reproduit le hardware qui traite bit-par-bit,
			- SDCard utilise une table de 256 CRCs codes en dur
$ ./CRC16_jln -s -H 01
	1021	ok, second element m_CRC16Table[1] of the table
$ ./CRC16_jln -s -H 80
	9188	ok, element m_CRC16Table[128] au milieu de la table
$ ./CRC16_jln -s -H FF
	1EF0	ok, last element m_CRC16Table[255] of the table

#### dualite
$  ./CRC16_jln -p -H 1248365709FD
	2218	un test arbitraire
on teste le meme polynome avec la direction inverse, et
on a renverse les bits de chaque byte du message :
$  ./CRC16_jln -q -H 48126CEA90BF
	1844	Ok, les 16 bits du CRC sont renverses

*/


