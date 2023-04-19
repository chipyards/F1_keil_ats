
// void SPI_init(void);

#define LCD_C     0
#define LCD_D     1

#define LCD_X     84	// soit 14 chars de 6 pix de large ou 12 chars de 7 pix
#define LCD_Y     48	// 6 lignes de 8 pix de haut

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

// ecrit 6 bytes consecutifs a l'index courant dans la RAM du LCD
// 5 bytes de la font 5x8 et 1 d'espacement
void LcdCharacter(char character);

// ecrit 6 bytes consecutifs par caractere
void LcdString( const char *characters );
