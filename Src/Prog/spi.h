
#define QSPIBUF		(1<<6)		// doit etre une puissance de 2
#define SPI_IMASK	(QSPIBUF-1)

#define SPI_NO_DATA	0

/* protocole fifo pour SPI slave :
   sens MOSI
	- overrun : detecte mais non signale : les data en trop sont perdues
	- underrun : prevenu (fonction get rend -1 a l'appli)
   sens MISO
	- overrun : prevenu (fonction putt rend -1 a l'appli)
	- underrun : signale par byte reserve SPI_NO_DATA que master doit ignorer
   N.B. dans le sens MISO il y a donc un byte "interdit"
*/

// l'objet SPI avec ses ring buffers
typedef struct {
unsigned char rx[QSPIBUF];
unsigned int rx_wi;	// index egaux <==> fifo vide
unsigned int rx_ri;
unsigned char tx[QSPIBUF];
unsigned int tx_wi;	// prochain byte a ecrire
unsigned int tx_ri;	// prochain byte a lire
} SPItype;

// contexte global
extern SPItype SPI_2;

// constructeur
void spi2_fifo_init(void);

// retourne 0 si ok, -1 si refus (prevention overrun)
int spi2_put8( unsigned int d );

// retourne un unsigned char, ou -1 si pas de data
int spi2_get8(void);

// attention : interrupt enable inside
void spi2_slave_init( int IRQ_prio );

