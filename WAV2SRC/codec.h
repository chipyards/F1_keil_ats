/** compression predictive avec perte */
/* resume du decodage :
	- les codes de QBIT bits sont extraits du binaire brut
	- ils sont utilises comme index dans la table de dequantification
	- cette table donne des differences signees (s16) a appliquer au predicteur
   details de realisation :
	- autour du zero la table donne des valeurs lineaires "1 pour 1" => sans perte sur les petites amplitudes
	  ( concerne QLIN valeurs de part et d'autre du zero)
	- l'amplitude du signal restitue, limitee par la PWM, est target_pk t.q. -target_pk <= sig <= target_pk
	  avant codage, le signal original a ete mis a cette echelle
	- theoriquement la difference peut atteindre 2 * target_pk, ce cas ne se rencontre pas dans l'audio reel
	  ==> on plafonne la difference a kplaf * target_pk ( 0 < kplaf < 2.0 ) avec perte
	- la position DC du signal restitue sur la target est determinee par la valeur initiale du predicteur
	  PWM_SILENCE = target_pk ==> valeur non signee t.q. 0 <= pw <= 2*target_pk
	  ( on peut avoir un minuscule clip a (2*target_pk)-1 si PWMdivider est pair )
   emission :
	- l'oversampling permet de faciliter le filtrage le la porteuse PWM, au detriment de la resolution PWM
	- a chaque interrupt PWM, le firmware va decider s'il doit decoder un nouveau sample ou interpoler
	  l'interpolation la plus rudimentaire consiste a repeter le sample anterieur (nearest neighbor)
*/

/*
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
*/

typedef struct {
// parametres de design
int FCK;		// 72000000
int PWMdivider;		// 3266			// PWM range = Tpwm en ticks = ( FCK / Fsamp ) / Koversamp
int Koversamp;		// 2			// Tsamp / Tpwm	
int QBIT;		// 5			// nombre de bits du code
int QLIN;		// 4			// nombre de valeurs de la partie lineaire du code
double kplaf;		// 0.4			// (max mag) / (PWMdiv/2) = coeff de limitation de la mag du code (<= 2.0)
// parametres derives
int target_pk; 		// (PWMdivider/2)	// amplitude peak sur target
int QCODE;		// (1<<(QBIT-1))	// nombre de valeurs de la magnitude du code
int QLOG;		// QCODE-QLIN		// nombre de bits de la partie "log" du code
int codemagmask;	// QCODE-1		// mask pour la magnitude du code
int codesgnmask;	// QCODE		// mask pour le bit de signe du code
} profile_t;

extern profile_t * P;

// initialisation
void codec_init( int ip );
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

