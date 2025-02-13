/* 
gcc -Wall -c -O3 from_bentham.c
gcc -Wall -O3 -o appli.exe appli.c from_bentham.o
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

unsigned int crc_ben( unsigned int init, unsigned int poly, unsigned int xorout, const unsigned char *buf, int len );
unsigned int crc_mm( unsigned int init, unsigned int poly, unsigned int xorout, const unsigned char *buf, int len );

int main( int argc, char ** argv )
{
unsigned int crc;
const unsigned char * lebuf;
unsigned int init, poly, xorout;

// ascii text
if	( argc > 1 )
	lebuf = (const unsigned char *)argv[1];
else	lebuf = (const unsigned char *)"123456789";	// reveng's "check"

// zone CRC32/ISO-HDLC aka PKZIP
init = 0xFFFFFFFF;
poly = 0xEDB88320;
xorout = 0xFFFFFFFF;
// selon reveng.sourceforge.io :
// poly=0x04c11db7 init=0xffffffff refin=true refout=true xorout=0xffffffff check=0xcbf43926 residue=0xdebb20e3 
// quelques codewords (les 4 derniers bytes du codeword sont le CRC des n-4 premiers)
unsigned char zero[] = { 0, 0, 0, 0 ,0x1C ,0xDF ,0x44 ,0x21 };
unsigned char ffff[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
unsigned char f201[] = { 0xF2, 0x01, 0x83, 0x77, 0x9D, 0xAB, 0x24 }; // F20183779DAB24

crc = crc_ben( init, poly, xorout, zero, 4 );
printf("CRC32/ISO-HDLC 0 0 0 0 -> %08X\n", crc );
crc = crc_ben( init, poly, xorout, zero, 8 );
printf("CRC32/ISO-HDLC 0 0 0 0 1CDF4421 -> %08X (%08X)\n", crc, ~crc );

crc = crc_ben( init, poly, xorout, ffff, 4 );
printf("CRC32/ISO-HDLC ff ff ff ff -> %08X\n", crc );
crc = crc_ben( init, poly, xorout, ffff, 8 );
printf("CRC32/ISO-HDLC ff ff ff ff ff ff ff ff -> %08X (%08X)\n", crc, ~crc );

crc = crc_ben( init, poly, xorout, f201, 3 );
printf("CRC32/ISO-HDLC F20183 -> %08X\n", crc );
crc = crc_ben( init, poly, xorout, f201, 7 );
printf("CRC32/ISO-HDLC F20183779DAB24 -> %08X (%08X)\n", crc, ~crc );

crc = crc_ben( init, poly, xorout, lebuf, strlen((const char *)lebuf) );
printf("CRC32/ISO-HDLC %s -> %08X\n", lebuf, crc );
/* conclusion pour CRC32/ISO-HDLC aka PKZIP :
	- nous utilisons le polynome 0xEDB88320 qui est le renversement de celui de reveng 0x04c11db7
	  Note : refin=true ==> message lu LSB first (comme crc_ben), register shifted left (pas comme crc_ben,
		 ce qui explique le renversement du polynome mais renverse aussi le CRC)
		 refout=true ==> CRC renverse a la fin (finalement comme nous)
	- le CRC obtenu pour textes arbitraires correspond exactement a ceux de zip et 7z
	- le CRC des codewords de reveng debarrasses des 4 derniers bytes donne bien ces
	  ces 4 bytes, mais dans l'ordre inverse, ce qui est naturel car nous sommes little endian
	- idem pour le "check" obtenu avec "123456789"
	- le CRC des codewords de reveng complets est constant, egal a celui obtenu pour 4 bytes nuls,
	  c'est le complement du "residue", normal car celui-ci est defini avant le XOR final
*/
printf("\n");

// zone CRC-32/AIXM "Recognised by the ICAO"
init = 0;
poly = 0x814141ab;
xorout = 0;
// selon reveng.sourceforge.io :
// poly=0x814141ab init=0x00000000 refin=false refout=false xorout=0x00000000 check=0x3010bf7f residue=0x00000000

crc = crc_mm( init, poly, xorout, zero, 4 );
printf("CRC32/AIXM 0 0 0 0 -> %08X\n", crc );

crc = crc_mm( init, poly, xorout, lebuf, strlen((const char *)lebuf) );
printf("CRC32/AIXM %s -> %08X\n", lebuf, crc );

// AIXM's tests
const char * tests[] = {
	"480637N", 
	"0163411E", 
	"480637N0163411E",
	"782", 
	"480637N0163411E782", 
	"46.7",
	"480637N0163411E46.7", 
	"480637N0163411E78246.7" }; 
for	( int i = 0; i < 8; i++ )
	{
	lebuf = (const unsigned char *)tests[i];
	crc = crc_mm( init, poly, xorout, lebuf, strlen((const char *)lebuf) );
	printf("CRC32/AIXM %s -> %08X\n", lebuf, crc );
	}
// reveng's codewords (appended CRC is big-endian) 
unsigned char c782[] = { 0x37, 0x38, 0x32, 0x6C, 0x29, 0x71, 0x00 };
unsigned char c467[] = { 0x34, 0x36, 0x2E, 0x37, 0x26, 0x6D, 0x25, 0xC1 };

crc = crc_mm( init, poly, xorout, c782, 7 );
printf("CRC32/AIXM 3738326C297100 -> %08X\n", crc );
crc = crc_mm( init, poly, xorout, c467, 8 );
printf("CRC32/AIXM 34362E37266D25C1 -> %08X\n", crc );

/* conclusion pour CRC32/AIXM "Recognised by the ICAO"
	- nous utilisons le polynome 0x814141ab donne par reveng, qui est le renversement du binaire de AIXM
	  dit CRC32Q par ICAO, soit 1101 0101 1000 0010 1000 0010 1000 0001
	- notre code crc_mm est conforme a refin=false ==> message lu MSB first, register shifted right
	  et refout=false (pas de renversement final)
	- le "check" obtenu avec "123456789" est Ok avec reveng
	- le CRC obtenu pour exemples d'AIXM (traites comme ascii sans delimiteurs) correspond exactement
	- le CRC obtenu pour 2 codewords de reveng donne bien le residu 00000000

*/

return 0;
}

