/** compression predictive avec perte */

// parametres de design
#define NOM_FSAMP	11025		// ==> periode en ticks = 72000000 / 11025 = 6530.61, arrondi a 6532
#define target_p 	(6532/4)	// amplitude peak sur target (i.e. 1/2 resolution PWM)
#define QBIT		5		// nombre de bits du code
#define QCODE		(1<<(QBIT-1))	// nombre de valeurs de la magnitude du code
#define QLIN		4		// nombre de valeurs de la partie lineaire du code
#define kplaf		0.4		// coeff de limitation de la mag de la diff, 2.0 <==> no limit

// parametres derives
#define QLOG		(QCODE-QLIN)	// nombre de bits de la partie "log" du code
#define codemagmask	(QCODE-1)	// mask pour la magnitude du code
#define codesgnmask	QCODE		// mask pour le bit de signe du code

// initialisation
void codec_init();
void codec_dump( FILE * dfil );

// decodeur
int decode( unsigned short code );

// encodeur
unsigned short encode( float signal );

// comprimer et packer l'audio fourni dans mbuf (float normalise -1.0;1.0)
// alloue la memoire pour wbuf, rend la taille de wbuf en unsigned int
int compress2w32( unsigned int qsamp, float * mbuf, unsigned int ** pwbuf );

// depacker et decomprimer l'audio fourni dans wbuf (unsigned int)
// alloue la memoire pour mbuf (float normalise -1.0;1.0)
int uncompress2float( unsigned int qsamp, float ** pmbuf, unsigned int * wbuf, unsigned int qwbuf );

