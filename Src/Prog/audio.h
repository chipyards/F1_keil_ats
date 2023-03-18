// frequence audio = 22045.3 Hz
#define QBIT 5
#define PWM_SILENCE 1633
#define PWM_PERIOD  3266
#define SAMP_PERIOD 3266
#define KOVER 1 // <-- OVERSAMPLING

// liste des sons a compiler
#define USE_frein
extern const unsigned int frein[];

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

