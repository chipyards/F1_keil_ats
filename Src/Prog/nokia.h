
// void SPI_init(void);

#define LCD_C     0
#define LCD_D     1

#define LCD_X     84	// soit 14 chars de 6 pix de large ou 12 chars de 7 pix
#define LCD_Y     48	// 6 lignes de 8 pix de haut

#ifdef __cplusplus
extern "C" {
#endif

// inclut l'initialisation du SPI du STM32 (mais pas le GPIO)
void LcdInitialize(void);

// efface la RAM du LCD (84 * 6 = 504 bytes)
void LcdClear( int pattern );

// emission d'un seul byte vers LCD
void LcdWrite( int modeDC, int data );

// positionne l'index dans la RAM du LCD
void LcdGotoXY( int x, int y );

void LcdSetContrast( int contrast );

void LcdSetBias( int n );

void LcdNegativeImage( int negative );

void LcdSetPower( int on );

// ecrit 6 ou 7 bytes consecutifs a l'index courant dans la RAM du LCD
// 5 bytes de la font 5x8 et 1 ou 2 d'espacement
void LcdCharacter( char character, int narrow );

// narrow = 0 : 12 car par ligne, auto wrap ( 12 * 7 = 84 )
// narrow = 1 : 14 car par ligne, auto wrap ( 14 * 6 = 84 )
void LcdString( const char *characters, int narrow );

// ecrit un caractere de taille double : 12 bytes sur 2 lignes
void LcdCharacter2( int x, int y, char character );

// 7 car par ligne, auto wrap ( 7 * 12 = 84 )
void LcdString2(  int x, int y, const char *characters );

#ifdef __cplusplus
}
#endif
