// 72000000 / 6532 = 11022.7 Hz
#define QBIT 5
#define SAMP_PERIOD  6532
#define PWM_PERIOD   (SAMP_PERIOD/2)	// oversampling x2
#define PWM_SILENCE  (PWM_PERIOD/2)

// liste des sons a compiler
#define USE_troca
extern const unsigned int troca[];

// etat du systeme audio
typedef struct {
int tai;			// nombre de samples du son
const unsigned int * wbuf1;	// pack de codes
int pos;			// position en samples (-1 = stop)
unsigned int iw;		// indice du word (32 bits)
unsigned int pb0;		// position du lsb du code courant dans le word courant	
unsigned int zew;		// word courant
short oldsig;			// predicteur
} type_etat;

extern const short dequant[];
extern type_etat etat;

// methodes
void audio_start( const unsigned int * leson );

