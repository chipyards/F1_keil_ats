/* fifo pour log vers uart ou transcript sur LCD */
// le contenu utile de chaque ligne est delimite par un '\0'
// les caractere de fin de ligne sont superflus et ignores
// dans le cas d'une transmission par UART vers CDC/USB, le '\0' sera remplace par un \n

#define LFIFOQB		(1<<9)		// capacite en bytes
#define LFIFOMS		(LFIFOQB-1)
#define LFIFOLL		(1<<6)		// longueur d'une ligne formattee

// l'objet logfifo
typedef struct {
unsigned int wra;	// prochaine adresse a ecrire
unsigned int rda;	// prochaine adresse a lire
char circ[LFIFOQB];	// buffer circulaire
} LOGtype;

// contexte global (singleton)
extern LOGtype logfifo;

// constructeur
void logfifo_init(void);

// ajouter un caractere
void LOGputc( char c );

// ajouter une ligne de texte formattee - sera tronquee si elle est trop longue
void LOGprint( const char *fmt, ... );

// ajouter une ligne de texte brute - sera tronquee si elle est trop longue
void LOGline( const char *fmt );

// a utiliser apres une serie de LOGputc() (pas necessaire apres LOGprint() ou LOGline() ) 
void LOGflush(void);
