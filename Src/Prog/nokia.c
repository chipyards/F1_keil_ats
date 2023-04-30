/* afficheur LCD mono Nokia 5110/3310, chip PCD8544, board version Banggood
   appli developpee et testee sur nucleo F103 (old stm32lib) puis portee sur L476 (Cube LL) puis retour sur F103
N.B. on est en mode SPI_NSS_Soft, la pin SS (Slave Select) <--> CE est un GPIO ordinaire

brochage SPI1 sur nucleo L476 (AF 5) et sur F103 (remap) (cf gpio.c)

	NUCLEO	ardu	NOKIA LCD
RST	PB10	D6	gris	RST	1
CE	PB4	D5	rouge	CE	2
DC	PA10	D2	bleu	DC	3
DIN	PB5	D4	jaune	DIN	4
CLK	PB3	D3	vert	CLK	5
VCC	+3V3		orange	VCC	6
GND			noir	GND	8

N.B. on pourrait aussi faire un master SPI soft... c'est dispo sur MAU08b.zip
*/

/* OLD SCHOOL utilise une reception "dummy" pour determiner quand remettre CE a 1
 * ainsi on attend que CLK soit idle (0) pour agir sur CE
 * --> le chronogramme est clean mais le code est sale
 * recommande pour F103
 * NEW SCHOOL utilise le bit BSY, alors CE est remis a 1 alors que CLK est encore a 1
 * normalement le slave n'est sensible qu'aux fronts montant de CLK - en effet c'est Ok
 * --> le code est plus clean mais le chronogramme un peu sale
 * recommande pour L476
 */
#define OLD_SCHOOL

// #include "stm32l4xx_ll_bus.h"
// #include "stm32l4xx_ll_gpio.h"
// #include "stm32l4xx_ll_spi.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include "gpio.h"
#include "nokia.h"

// N.B. general setup/hold pour PCD8544 : 100ns
// vu que RST, DC, NSS sont controles par soft, on devra au choix :
//	- moderer l'horloge du STM32 (<= 10 MHz)
//	- inserer des NOPs

// SPI1 en TX half duplex
static void SPI_init(void)
{
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
// N.B. il n'y a pas de mode "SIMPLEX TX"
// le mode HALF_DUPLEX_TX implique un controle bidirectionnel (tri-state)
// ici on devrait pouvoir utiliser FULL_DUPLEX ou HALF_DUPLEX_TX aussi bien
#ifdef OLD_SCHOOL
LL_SPI_SetTransferDirection( SPI1,LL_SPI_FULL_DUPLEX );
//LL_SPI_SetRxFIFOThreshold( SPI1, LL_SPI_RX_FIFO_TH_QUARTER );	// pour utiliser RXNE sur STM32L476
#else
LL_SPI_SetTransferDirection( SPI1,LL_SPI_HALF_DUPLEX_TX );
#endif
LL_SPI_SetMode( SPI1, LL_SPI_MODE_MASTER );
LL_SPI_SetDataWidth( SPI1, LL_SPI_DATAWIDTH_8BIT );
LL_SPI_SetClockPolarity( SPI1, LL_SPI_POLARITY_LOW );	// idle low
LL_SPI_SetClockPhase( SPI1, LL_SPI_PHASE_1EDGE );	// 1st edge samples incoming data
LL_SPI_SetNSSMode( SPI1, LL_SPI_NSS_SOFT );
LL_SPI_SetTransferBitOrder( SPI1, LL_SPI_MSB_FIRST );
if	( SystemCoreClock > 10000100 )
	LL_SPI_SetBaudRatePrescaler( SPI1, LL_SPI_BAUDRATEPRESCALER_DIV32 );	// 250 kHz @ 8 MHz, 312.5 kHz @ 10 MHz
else	LL_SPI_SetBaudRatePrescaler( SPI1, LL_SPI_BAUDRATEPRESCALER_DIV256 );	// 281.25 kHz @ 72 MHz
LL_SPI_Enable(SPI1);
}

#ifdef OLD_SCHOOL
volatile int rxdata;
#endif

// emission d'un seul byte vers LCD
void LcdWrite( int modeDC, int data )
{
/* digitalWrite(PIN_DC, dc);
   digitalWrite(PIN_SCE, LOW);
   shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data);
   digitalWrite(PIN_SCE, HIGH);
 */
// bit de mode
if	( modeDC )
	NOKIA_DC_HI();	// mode Data
else	NOKIA_DC_LO(); 	// mode Command

// NSS = CE act lo
NOKIA_CE_LO();

// poor people's delay
// better use DWT: Data watchpoint trigger
// https://deepbluembedded.com/stm32-delay-microsecond-millisecond-utility-dwt-delay-timer-delay/
__ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP");
__ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP");

// Wait for spi_m Tx buffer empty
while	( !LL_SPI_IsActiveFlag_TXE(SPI1) )
	{}
// Send spi_m data
LL_SPI_TransmitData8(SPI1, data );

#ifdef OLD_SCHOOL // methode old school (F103) pour detecter fin de byte
// Wait for spi_m data reception <==> end of transmission
while	( !LL_SPI_IsActiveFlag_RXNE(SPI1) ) {}
// Read dummy received data to clear RXNE */
rxdata = LL_SPI_ReceiveData8( SPI1 );
#else
// Wait end of transmission
while	( LL_SPI_IsActiveFlag_BSY(SPI1) )
	{}
#endif

__ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP");
__ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP");

// NSS = CE idle hi
NOKIA_CE_HI();

__ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP");
__ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP"); __ASM volatile ("NOP");

}

// code pour le LCD Nokia 5110 ou 3310 (PCD8544 chip) revu par JLN
// soft font 5x8 incluse


// inclut l'initialisation du SPI du STM32 (mais pas le GPIO)
/* parametres du LCD
 * Il y a 3 parametres physiques :
 * - Vop aka contrast aka Vlcd : de 3 à 10V en 127 steps - code 0x80 + [0:127] - on fonctionne entre 55 et 60
 * - Temperature coeff : augmente Vlcd quand la temperature baisse - code 0x04 + [0:3] - on fonctionne a zero
 * 	Tc0		Tc1		Tc2		Tc3
 * 	0mV/°K		9mV/°K		17mV/°K		24mV/°K
 * - Bias aka voltage gap : le driver utilise 6 voltages (incluant gnd et Vlcd), 2 groupes de 3 separes par un gap
 *   c'est le gap qui est programmable, via le facteur n affectant la resistance centrale du pont R - R - nR - R - R
 *   la valeur a fournir est (7-n) - code 0x10 + [0:7] - on fonctionne a 3
 *   N.B. la valeur optimale de n est liee au mux rate, qui n'est PAS configurable (fixe a 1:48 ==> n = 4)
 */
void LcdInitialize(void)
{
/* base arduino
  pinMode(PIN_SCE, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_SDIN, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  digitalWrite(PIN_RESET, HIGH);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
*/

// NOKIA reset
NOKIA_RST_LO(); 	// Nokia Reset act. lo

SPI_init();

// NOKIA end of reset
NOKIA_RST_HI();		// Nokia Reset idle hi

LcdWrite( LCD_C, 0x21 );  // LCD Extended Commands.
LcdWrite( LCD_C, 0xB1 );  // Set LCD Vop (Contrast).
			  // N.B. here 0xB1 = 0x80 | 0x31, 0x31 is 49
			  // they say : 40-60 is usually a pretty good range.
LcdWrite( LCD_C, 0x04 );  // Set Temp coefficent. //0x04
LcdWrite( LCD_C, 0x14 );  // LCD bias mode 1:48. //0x13
LcdWrite( LCD_C, 0x20 );  // LCD Basic Commands
LcdWrite( LCD_C, 0x0C );  // LCD in normal mode.
}

void LcdSetPower( int on )
{
if	( on )
	LcdWrite( LCD_C, 0x20 );  // LCD Basic Commands
else	LcdWrite( LCD_C, 0x24 );  // power down
}

// efface la RAM du LCD (84 * 6 = 504 bytes)
void LcdClear( int pattern )
{
int index;
LcdWrite( LCD_C, 0x20 ); //Set display mode
for	( index = 0; index < (LCD_X * LCD_Y / 8); index++)
	LcdWrite( LCD_D, pattern );	// 0x54 = rayures horizontales, sinon 0x00
}

// positionne l'index interne dans la RAM du LCD
// x - range: 0 to 84
// y - range: 0 to 5 (les bytes sont verticaux, la hauteur totale est 6 * 8 = 48px)
void LcdGotoXY(int x, int y)
{
LcdWrite( LCD_C, 0x20 ); //Set display mode
LcdWrite( LCD_C, 0x80 | x );  // Column.
LcdWrite( LCD_C, 0x40 | y );  // Row.
}

// Set contrast can set the LCD Vop to a value between 0 and 127.
// 40-60 is usually a pretty good range.
void LcdSetContrast( int contrast )
{
LcdWrite( LCD_C, 0x21 ); //Tell LCD that extended commands follow
LcdWrite( LCD_C, 0x80 | contrast ); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
LcdWrite( LCD_C, 0x20 ); //Set display mode
}

// Set bias aka voltage gap. Theoric optimal is 4
void LcdSetBias( int n )
{
n &= 7;
LcdWrite( LCD_C, 0x21 ); //Tell LCD that extended commands follow
LcdWrite( LCD_C, 0x10 | ( 7 - n ) ); // n in ladder  R - R - nR - R - R
LcdWrite( LCD_C, 0x20 ); //Set display mode
}

void LcdNegativeImage( int negative )
{
LcdWrite( LCD_C, 0x20 ); //Set display mode
if	( negative )
	LcdWrite( LCD_C, 0x0D );  // LCD in inverse mode.
else	LcdWrite( LCD_C, 0x0C );  // LCD in normal mode.
}

/* Font table: 5 pixels wide and 8 pixels high.
 * 1 byte = one 8-pixel, vertical column in a character.
 * MSB at the bottom - MSB never set (reserved for spacing)
 * 5 bytes per character (spacing not included). */
static const char font5x8[128-32][5] = {
  // First 32 characters (0x00-0x19) are skipped
   {0x00, 0x00, 0x00, 0x00, 0x00} // 0x20
  ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 0x21 !
  ,{0x00, 0x07, 0x00, 0x07, 0x00} // 0x22 "
  ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 0x23 #
  ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 0x24 $
  ,{0x23, 0x13, 0x08, 0x64, 0x62} // 0x25 %
  ,{0x36, 0x49, 0x55, 0x22, 0x50} // 0x26 &
  ,{0x00, 0x05, 0x03, 0x00, 0x00} // 0x27 '
  ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 0x28 (
  ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 0x29 )
  ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 0x2a *
  ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 0x2b +
  ,{0x00, 0x50, 0x30, 0x00, 0x00} // 0x2c ,
  ,{0x08, 0x08, 0x08, 0x08, 0x08} // 0x2d -
  ,{0x00, 0x60, 0x60, 0x00, 0x00} // 0x2e .
  ,{0x20, 0x10, 0x08, 0x04, 0x02} // 0x2f /
  ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 0x30 0
  ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 0x31 1
  ,{0x42, 0x61, 0x51, 0x49, 0x46} // 0x32 2
  ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 0x33 3
  ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 0x34 4
  ,{0x27, 0x45, 0x45, 0x45, 0x39} // 0x35 5
  ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 0x36 6
  ,{0x01, 0x71, 0x09, 0x05, 0x03} // 0x37 7
  ,{0x36, 0x49, 0x49, 0x49, 0x36} // 0x38 8
  ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 0x39 9
  ,{0x00, 0x36, 0x36, 0x00, 0x00} // 0x3a :
  ,{0x00, 0x56, 0x36, 0x00, 0x00} // 0x3b ;
  ,{0x08, 0x14, 0x22, 0x41, 0x00} // 0x3c <
  ,{0x14, 0x14, 0x14, 0x14, 0x14} // 0x3d =
  ,{0x00, 0x41, 0x22, 0x14, 0x08} // 0x3e >
  ,{0x02, 0x01, 0x51, 0x09, 0x06} // 0x3f ?
  ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 0x40 @
  ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 0x41 A
  ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 0x42 B
  ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 0x43 C
  ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 0x44 D
  ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 0x45 E
  ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 0x46 F
  ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 0x47 G
  ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 0x48 H
  ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 0x49 I
  ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 0x4a J
  ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 0x4b K
  ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 0x4c L
  ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 0x4d M
  ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 0x4e N
  ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 0x4f O
  ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 0x50 P
  ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 0x51 Q
  ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 0x52 R
  ,{0x46, 0x49, 0x49, 0x49, 0x31} // 0x53 S
  ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 0x54 T
  ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 0x55 U
  ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 0x56 V
  ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 0x57 W
  ,{0x63, 0x14, 0x08, 0x14, 0x63} // 0x58 X
  ,{0x07, 0x08, 0x70, 0x08, 0x07} // 0x59 Y
  ,{0x61, 0x51, 0x49, 0x45, 0x43} // 0x5a Z
  ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 0x5b [
  ,{0x02, 0x04, 0x08, 0x10, 0x20} // 0x5c '\' ah ah \ tout nu il casse la ligne
  ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 0x5d ]
  ,{0x04, 0x02, 0x01, 0x02, 0x04} // 0x5e ^
  ,{0x40, 0x40, 0x40, 0x40, 0x40} // 0x5f _
  ,{0x00, 0x01, 0x02, 0x04, 0x00} // 0x60 `
  ,{0x20, 0x54, 0x54, 0x54, 0x78} // 0x61 a
  ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 0x62 b
  ,{0x38, 0x44, 0x44, 0x44, 0x20} // 0x63 c
  ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 0x64 d
  ,{0x38, 0x54, 0x54, 0x54, 0x18} // 0x65 e
  ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 0x66 f
  ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 0x67 g
  ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 0x68 h
  ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 0x69 i
  ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 0x6a j
  ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 0x6b k
  ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 0x6c l
  ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 0x6d m
  ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 0x6e n
  ,{0x38, 0x44, 0x44, 0x44, 0x38} // 0x6f o
  ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 0x70 p
  ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 0x71 q
  ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 0x72 r
  ,{0x48, 0x54, 0x54, 0x54, 0x20} // 0x73 s
  ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 0x74 t
  ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 0x75 u
  ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 0x76 v
  ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 0x77 w
  ,{0x44, 0x28, 0x10, 0x28, 0x44} // 0x78 x
  ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 0x79 y
  ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 0x7a z
  ,{0x00, 0x08, 0x36, 0x41, 0x00} // 0x7b {
  ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 0x7c |
  ,{0x00, 0x41, 0x36, 0x08, 0x00} // 0x7d }
  ,{0x10, 0x08, 0x08, 0x10, 0x08} // 0x7e ~
  ,{0x78, 0x46, 0x41, 0x46, 0x78} // 0x7f DEL
};

// ecrit 6 ou 7 bytes consecutifs a l'index courant dans la RAM du LCD
// 5 bytes de la font 5x8 et 1 ou 2 d'espacement
void LcdCharacter( char character, int narrow )
{
int index;
if	( narrow )
	LcdWrite( LCD_D, 0x00 );
for	( index = 0; index < 5; index++ )
	{
	LcdWrite( LCD_D, font5x8[character - 0x20][index]);
	}
LcdWrite( LCD_D, 0x00 );
}

// narrow = 0 : 12 car par ligne, auto wrap ( 12 * 7 = 84 )
// narrow = 1 : 14 car par ligne, auto wrap ( 14 * 6 = 84 )
void LcdString( const char *characters, int narrow )
{
while	( *characters )
	LcdCharacter( *characters++, narrow );
}

// duplication des bits en vue de doubler l'echelle verticale d'un glyphe
static unsigned int duplicate_bits( int c )
{
unsigned int res = 0;
for	( int i = 0; i < 8; i++ )	// on espace les bits
	{
	res <<= 2;
	res |= ( c & 0x80 );
	c <<= 1;
	}
return ( res >> 6 ) | ( res >> 7 );	// et on duplique
}

// ecrit un caractere de taille double : 12 bytes sur 2 lignes
void LcdCharacter2( int x, int y, char character )
{
unsigned int i, d; unsigned char cache[5];
LcdGotoXY( x, y );
for	( i = 0; i < 5; i++ )
	{			// top row : least significant bytes
	d = duplicate_bits( font5x8[character-0x20][i] );
	cache[i] = d >> 8;
	LcdWrite( LCD_D, d ); LcdWrite( LCD_D, d );
	}
LcdWrite( LCD_D, 0x00 ); LcdWrite( LCD_D, 0x00 );
LcdGotoXY( x, y + 1 );
for	( i = 0; i < 5; i++ )
	{			// bottom row : most significant bytes
	LcdWrite( LCD_D, cache[i] ); LcdWrite( LCD_D, cache[i] );
	}
LcdWrite( LCD_D, 0x00 ); LcdWrite( LCD_D, 0x00 );
}

// 7 car par ligne, auto wrap ( 7 * 12 = 84 )
void LcdString2(  int x, int y, const char *characters )
{
while	( *characters )
	{
	LcdCharacter2( x, y, *characters++ );
	x += 12;
	}
}
