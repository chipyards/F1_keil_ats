/* petite lib pour le decodage des sentences NMEA du GPS NEO 8 (et autres)
*/

// format pour 1 variable
typedef struct {
volatile int val;	// raw int value
volatile char	type;	//	'a'	ascii, delim ',' or '*'
		//	'd'	decimal, delim ',' or '*'
		//	'2'	2-byte, no delim
		//	'3'	3-byte, no delim
		//	'f'	floating point,  delim ',' or '*'
		//	0	not to be stored
volatile char	frac;	// nmea floating point numbers are represented by :
		//	val which is an integer in units of the last fractional digit
		//	frac which is the number of fractional digits
volatile char	stat;	// 0=ok, 1=undefined, 2=err
volatile char	fill;
} gps_var;

// contexte du parseur

// quantite de variables pour allocation statique
// attention pas plus de 128 variables, l'indice est char, et -1 est reserve
// ATTENTION la fonction gps_var_init() ne doit pas depasser cette limite
#define QVAR 20

typedef struct {
// stockage global
volatile gps_var data[QVAR];
volatile unsigned int qdata;		// number of data <= QVAR

// sentence en cours
volatile int fourcc;	// sentence model
volatile int tmp;		// current element in progress
volatile const char * fields;	// template de la sentence en cours
volatile int qfields;		// number of fields defined for this sentence
volatile int ifield;		// index of the current/next field
volatile int ichar;		// index of current char in field
volatile int ivar;		// index of current variable in data[]
volatile char tfield;		// type of current field
volatile char status;		// 42 = checksum ok
volatile unsigned char checksum;	// checksum in progress
} nmea_ctx;

// codes FOURCC
#define NGGA ( ('N') | ((int)'G'<<8) | ((int)'G'<<16) | ((int)'A'<<24) )
#define PGGA ( ('P') | ((int)'G'<<8) | ((int)'G'<<16) | ((int)'A'<<24) )
#define NRMC ( ('N') | ((int)'R'<<8) | ((int)'M'<<16) | ((int)'C'<<24) )
#define PRMC ( ('P') | ((int)'R'<<8) | ((int)'M'<<16) | ((int)'C'<<24) )

// initialisation des variables
void gps_var_init( nmea_ctx * ctx );
void new_sentence( nmea_ctx * ctx );
void invalidate_sentence( nmea_ctx * ctx );

// traitement d'un char
void nmea_proc( nmea_ctx * ctx, char c );

// conversion des minutes decimales en microdegres
int min2microdeg( int fmin, int frac );
