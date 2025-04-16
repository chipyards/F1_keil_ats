#define QTX1 132

// le contexte pour UART2 aka CDC
// TX : un buffer de message
// RX : un byte de commande
typedef struct {
volatile char TXbuf[QTX1];
volatile int TXindex;
volatile int verbose;
// reception CDC : fifo circulaire
#ifdef RX_FIFO
#define QRX 32		// a power of 2 !!!
char RXbuf[QRX];
volatile unsigned int RXwi=0;	// write index
volatile unsigned int RXri=0;	// read index
// exemple de lecture du fifo  :
// 	while	( RXwi - RXri )
//		{
//		int c = RXbuf[(RXri++)&(QRX-1)];
//		... }
#else
volatile int RXbyte;
#endif
} CDCtype;

extern CDCtype CDC;	// pour communiquer avec PC via ST-Link

#ifdef __cplusplus
extern "C" {
#endif

// haut niveau : objet CDC

// constructeur
void CDC_init(void);

void UART2_wait_TX_complete();

// envoyer une ligne de texte formattee (bloquant seulement si transmission en cours)
void CDC_printf( const char *fmt, ... );

// lire une commande, rendre -1 s'il n'y a rien de nouveau
int CDC_getcmd(void);

#ifdef __cplusplus
}
#endif
