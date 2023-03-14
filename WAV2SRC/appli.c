/* programme pour experimenter le decoupage en bandes sur fichier WAVE */
/*   */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <math.h>

#include "wav_head.h"
#include "codec.h"


void usage()
{
fprintf( stderr,
"\nUsage : wav2src source.wav dest.s -a   # asm pour Keil (style CHTI)"
"\n        wav2src source.wav dest.c -c   # C non comprime compatible CHTI"
"\n        wav2src source.wav dest.c -n   # C non comprime , struct nommee"
"\n        wav2src source.wav dest.wav -w # test compression"
"\n        wav2src source.wav dest.c -z   # C comprime, 11025 Hz"
"\n        wav2src -       dequant.c -t   # table de dequantification en C , 11025 Hz\n\n");
exit(1);
}

// lecture WAV entier en memoire, fichier doit etre ouvert et header lu dans un wavpars
// donnees mono 32 bits stockées dans un buffers de float alloue par cette fonction
void wave_read_body_mono( wavpars * s, float ** pbuf )
{
unsigned int rambytes;

// allocation RAM pour le son entier
rambytes = s->wavsize * sizeof(float);
*pbuf = (float *)malloc( rambytes );

if	( *pbuf == NULL )
	gasp("echec malloc %d bytes", rambytes );
// printf("malloc Ok %d bytes = %d samples\n", rambytes, rambytes/sizeof(float) );

// lecture bufferisee pour pouvoir traiter stereo ou pouvoir convertir en float ou les deux
unsigned int rdbytes;			// retour de read()
unsigned int remsamples;		// samples restant a lire
unsigned int rdsamples;			// samples juste lus
unsigned int rdframes;			// frames juste lues (1 frame = 2 samples si on est en stereo)
unsigned int totalframes;		// total des frames lues
unsigned int sizeofsamp=1;		// taille d'un sample en bytes
#define QRAW 4096			// taille du buffer en bytes

char rawsamples[QRAW];	// buffer pour read()
#define rawsamples16 ((short int *)rawsamples)
#define rawsamples32 ((float *)rawsamples)

if	( ( s->type == 1 ) && ( s->resol == 16 ) )
	sizeofsamp = 2;
else if	( ( s->type == 3 ) && ( s->resol == 32 ) )
	sizeofsamp = 4;
else	gasp("type %d with resolution %d unsupported", s->type, s->resol );

unsigned int i, j;
j = 0; totalframes = 0;
while	( totalframes < s->wavsize )
	{
	remsamples = ( s->wavsize - totalframes ) * s->chan;
	if	( remsamples > ( QRAW / sizeofsamp ) )
		remsamples = ( QRAW / sizeofsamp );
	rdbytes = read( s->hand, rawsamples, ( remsamples * sizeofsamp ) );
	rdsamples = rdbytes / sizeofsamp;
	if	( rdsamples != remsamples )
		gasp("truncated WAV data");
	rdframes = rdsamples / s->chan;
	totalframes += rdframes;
	if	( s->chan == 2 )
		{
		if	( s->type == 3 )
			for	( i = 0; i < rdsamples; i += 2 )
				(*pbuf)[j++] = 0.5 * ( rawsamples32[i] + rawsamples32[i+1] );
		else	for	( i = 0; i < rdsamples; i += 2 )
				(*pbuf)[j++] = (0.5/32768.0) * (float)( rawsamples16[i] + rawsamples16[i+1] );
		}
	else if	( s->chan == 1 )
		{
		if	( s->type == 3 )
			for	( i = 0; i < rdsamples; ++i )
				(*pbuf)[j++] = rawsamples32[i];
		else	for	( i = 0; i < rdsamples; ++i )
				(*pbuf)[j++] = (1.0/32768.0) * (float)rawsamples16[i];
		}
	}
if	( totalframes != s->wavsize )	// cela ne peut pas arriver, cette verif serait parano
	gasp("WAV size error %u vs %d", totalframes , (int)s->wavsize );
close( s->hand );
// printf("lu %d samples Ok\n", totalframes );
}

// Keil style ASM source code style CHTI
void write_keil_asm( FILE * dfil, wavpars * s, float * mbuf )
{
fprintf( dfil, "\tAREA DonneeSon, DATA, READONLY\n");
fprintf( dfil, "\texport LongueurSon\n");
fprintf( dfil, "\texport PeriodeSonMicroSec\n");
fprintf( dfil, "\texport Son\n");

fprintf( dfil, "LongueurSon\tDCD\t%u\n", s->wavsize );
fprintf( dfil, "PeriodeSonMicroSec\tDCD\t%d\n", (int)( round( 1000000.0/(double)s->freq ) ) );

fprintf( dfil, "Son "); 
// on a le son entier en mono dans mbuf[], convertissons le en ASM "Keil"
for	( int i = 0; i < s->wavsize; ++i )
	{
	fprintf( dfil, "\tdcw\t%d\n", (int)floor(32768.0*mbuf[i]) );
	}
fprintf( dfil, "\tend\n"); 
}

// C source code style CHTI
void write_sl16_c( FILE * dfil, wavpars * s, float * mbuf )
{
fprintf( dfil, "const unsigned int LongueurSon = %u;\n", s->wavsize );
fprintf( dfil, "const unsigned int PeriodeSonMicroSec = %d;\n", (int)( round( 1000000.0/(double)s->freq ) ) );

fprintf( dfil, "const short Son[] = {\n"); 
// on a le son entier en mono dans mbuf[], convertissons le en C
int i;
for	( i = 0; i < ( s->wavsize - 1 ); ++i )
	{
	fprintf( dfil, "%d, ", (int)floor(32768.0*mbuf[i]) );
	if	( ( i % 8 ) == 7 )
		fprintf( dfil, "\n" );
	}
// dernier element sans la virgule
fprintf( dfil, "%d };\n", (int)floor(32768.0*mbuf[i]) );
}

// C source code avec struct nommee pour multi-son
void write_sl16_nom_c( FILE * dfil, wavpars * s, float * mbuf, char * nom )
{

fprintf( dfil, "#include \"../audio.h\"\n" );
fprintf( dfil, "#ifdef USE_%s\n", nom );
fprintf( dfil, "/* pour utiliser ce son, declarer :\n" );
fprintf( dfil, "#define USE_%s\n", nom );
fprintf( dfil, "extern const type_son %s;\n", nom );
fprintf( dfil, "*/\n" );
fprintf( dfil, "static const short leson[] = {\n"); 
// on a le son entier en mono dans mbuf[], convertissons le en C
int i;
for	( i = 0; i < ( s->wavsize - 1 ); ++i )
	{
	fprintf( dfil, "%d, ", (int)floor(32768.0*mbuf[i]) );
	if	( ( i % 8 ) == 7 )
		fprintf( dfil, "\n" );
	}
// dernier element sans la virgule
fprintf( dfil, "%d };\n", (int)floor(32768.0*mbuf[i]) );
// la structure, enfin
fprintf( dfil, "const type_son %s = { .longson=%u, .periodus=%u, .son=leson };	// style C99\n",
		nom, s->wavsize, (int)( round( 1000000.0/(double)s->freq ) ) );
fprintf( dfil, "#endif\n");
}

// production WAV file pour test auditif compression-decompression
// le fichier source est deja lu dans mbuf
// le fichier dest est a creer
void write_codec_wav( wavpars * s, float * mbuf, const char * fnam )
{
printf("source %u ech. @ %u Hz, duree %g s\n", s->wavsize, s->freq, (double)s->wavsize / (double)s->freq );
unsigned int qsamp = s->wavsize;
unsigned int * wbuf;	// buffer pour audio comprime
int retval;
// comprimer et packer l'audio fourni dans mbuf (float normalise -1.0;1.0)
// alloue la memoire pour wbuf, rend la taille de wbuf en unsigned int
retval = compress2w32( qsamp, mbuf, &wbuf );
if	( retval <= 0 )
	gasp("echec compress2w32 : %d", retval );
unsigned int qw32 = (unsigned int)retval;
printf("compression effectuee : %u words de 32 bits\n", qw32 );
unsigned int qbytes = ((s->wavsize*QBIT)/8)+1;
printf("codage %d bits soit %u bytes = %g kbytes\n", QBIT, qbytes, ((double)qbytes)/1024.0 );
// depacker et decomprimer l'audio fourni dans wbuf (unsigned int)
// alloue la memoire pour mbuf (float normalise -1.0;1.0)
float * mbuf2;
retval = uncompress2float( qsamp, &mbuf2, wbuf, qw32 );
if	( retval < 0 )
	gasp("echec uncompress2float : %d", retval );
// ecriture fichier
int resol = 32;
s->type = (resol==32)?(3):(1);
s->chan = 1;
// s->freq // no change 
s->resol = resol;
// s->wavsize // no change
// block et bpsec seront calcules par WAVwriteHeader
s->hand = open( fnam, O_RDWR | O_BINARY | O_CREAT | O_TRUNC, 0666 );
if	( s->hand == -1 )
	gasp("echec ouverture ecriture %s", fnam );
WAVwriteHeader( s );
int bytecnt, writecnt;
bytecnt = s->wavsize * ((resol==32)?(sizeof(float)):(sizeof(short)));
writecnt = write( s->hand, mbuf2, bytecnt );
if	( writecnt != bytecnt )
	gasp("erreur ecriture %s", fnam );
close( s->hand );
printf("fini ecriture %s\n", fnam );
}

// C source code avec audio comprime, tableau nomme de u32
void write_comp_nom_c( FILE * dfil, wavpars * s, float * mbuf, char * nom )
{
fprintf( dfil, "#include \"../audio.h\"\n" );
fprintf( dfil, "#ifdef USE_%s\n", nom );
fprintf( dfil, "/* pour utiliser ce son, declarer :\n" );
fprintf( dfil, "#define USE_%s\n", nom );
fprintf( dfil, "extern const unsigned int %s[];\n", nom );
fprintf( dfil, "*/\n" );
fprintf( dfil, "const unsigned int %s[] = {\n", nom ); 
// on a le son entier en mono dans mbuf[], comprimons-le dans un buffer wbuf
printf("source %u ech. @ %u Hz, duree %g s\n", s->wavsize, s->freq, (double)s->wavsize / (double)s->freq );
unsigned int qsamp = s->wavsize;
unsigned int * wbuf;	// buffer pour audio comprime
int retval;
// comprimer et packer l'audio fourni dans mbuf (float normalise -1.0;1.0)
// alloue la memoire pour wbuf, rend la taille de wbuf en unsigned int
retval = compress2w32( qsamp, mbuf, &wbuf );
if	( retval <= 0 )
	gasp("echec compress2w32 : %d", retval );
unsigned int qw32 = (unsigned int)retval;
printf("compression effectuee : %u words de 32 bits\n", qw32 );
unsigned int qbytes = ((s->wavsize*QBIT)/8)+1;
printf("codage %d bits soit %u bytes = %g kbytes\n", QBIT, qbytes, ((double)qbytes)/1024.0 );
// convertissons le binaire 32 bits en texte C
fprintf( dfil, "%u,   // le nombre d'echantillons\n", qsamp );
int i;
for	( i = 0; i < ( qw32 - 1 ); ++i )
	{
	fprintf( dfil, "0x%08x, ", wbuf[i] );
	if	( ( i % 4 ) == 3 )
		fprintf( dfil, "\n" );
	}
// dernier element sans la virgule
fprintf( dfil, "0x%08x };\n", wbuf[i] );
fprintf( dfil, "#endif\n");
}

void write_comp_table( FILE * dfil )
{
codec_init();
codec_dump( dfil );
}

int main( int argc, char ** argv )
{
wavpars s;
char snam[256];
char dnam[256];
FILE * dfil;
float * mbuf;

if ( argc != 4 ) usage();

sprintf( snam, "%s", argv[1] );
sprintf( dnam, "%s", argv[2] );

if	( snam[0] != '-' )
	{
	printf("ouverture %s en lecture\n", snam );
	s.hand = open( snam, O_RDONLY | O_BINARY );
	if ( s.hand == -1 ) gasp("not found");
		WAVreadHeader( &s );
	wave_read_body_mono( &s, &mbuf );
	close( s.hand );
	printf("%u ech. @ %u Hz, duree %g s\n", s.wavsize, s.freq, (double)s.wavsize / (double)s.freq );
	if	( s.freq != NOM_FSAMP )
		printf("ATTENTION : freq d'echantillonnage %u au lieu de %u\n", s.freq, NOM_FSAMP );
	}

if	( argv[3][1] != 'w' )
	{
	printf("ouverture %s en ecriture\n", dnam );
	dfil = fopen( dnam, "w" );
	if ( dfil == NULL ) gasp("pb ouverture pour ecrire");
	}
else	dfil = NULL;

// ici on choisit le format de sortie
switch	( argv[3][1] )
	{
	case 'a' : write_keil_asm( dfil, &s, mbuf ); break;
	case 'c' : write_sl16_c( dfil, &s, mbuf ); break;
	case 'n' :
		{	// extraire le prenom du son
		for	( int i = 1; i < strlen(dnam); ++i )
			if	( dnam[i] == '.' )
				dnam[i] = 0;
		write_sl16_nom_c( dfil, &s, mbuf, dnam );
		} break;
	case 'w' : write_codec_wav( &s, mbuf, dnam ); break;
	case 'z' :
		{	// extraire le prenom du son
		for	( int i = 1; i < strlen(dnam); ++i )
			if	( dnam[i] == '.' )
				dnam[i] = 0;
		write_comp_nom_c( dfil, &s, mbuf, dnam );
		} break;
	case 't' : write_comp_table( dfil ); break;
	default : usage();
	}

if	( dfil )
	fclose( dfil );
return 0;
}

