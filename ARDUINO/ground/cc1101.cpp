#include <SPI.h>
#include "cc1101.h"

/* SPI en full duplex
 * SCK  = 13
 * MISO = 12
 * MOSI = 11
 * SS = 10
 */
void SPI1_init(void)
{
SPI.begin();
SPI.beginTransaction( SPISettings( 2000000, MSBFIRST, SPI_MODE0 ) );
pinMode( 10, OUTPUT );
pinMode( 2, INPUT_PULLUP );  // GDO0
digitalWrite( 10, 1 );
}

// ecrire et lire cnt bytes en une transaction
void SPI1_multi_byte( unsigned char * txbuf, unsigned char * rxbuf, byte cnt )
{
digitalWrite( 10, 0 );  // SS low
while ( digitalRead( 12 ) ) // wait for wake-up <==> MISO low
  {}
for ( byte i = 0; i < cnt; i++ )
    *(rxbuf++) = SPI.transfer( *(txbuf++) );
digitalWrite( 10, 1 );  // SS hi
}

///
/// ROM data (enfin on voudrait)
///

const char * fsm_states[] = {     // noms des codes d'etats obtenus dans le status byte
  "IDLE", "RX", "TX", "FSTXON", "CALIB", "SETTLE", "RX_OVER", "TX_OVER"
  };

///
/// singleton
///
CC1101 CC;

///
/// CC1101 methods
///

// burst read, rend l'adresse d'un array de bytes PRECAIRE a utiliser immediatement
// ce buffer peut etre altere par un read_reg() interpose NOT THREAD SAFE
unsigned char * CC1101::read_regs( byte start_adr, byte cnt ) {
  txbuf[0] = 0xC0 | ( start_adr & 0x3f );
  SPI1_multi_byte( txbuf, rxbuf, cnt+1 );
  return rxbuf+1;
  }
// burst write, prend l'adresse d'un array de bytes NOT THREAD SAFE
void CC1101::write_regs( byte start_adr, const unsigned char * src, byte cnt ) {
  txbuf[0] = 0x40 | ( start_adr & 0x3f );
  memcpy( txbuf+1, src, cnt );
  SPI1_multi_byte( txbuf, rxbuf, cnt+1 );
  }

///
/// experiences
///

// lire les 47 registres de 00 a 2E
void CC1101::dump_config()
{
unsigned char * fbuf = read_regs( 0, 0x2F );
for ( byte i = 0; i < 0x2F; i++ )
    {
    snprintf( tbuf, sizeof(tbuf), "reg %02x : %02x\n", i, fbuf[i] );
    Serial.print( tbuf );
    }
unsigned long int fu = get_synth_frequ();
// convertir en kHz
fu *= 1625;   // 26000 / 16
fu >>= 12;
snprintf( tbuf, sizeof(tbuf), "F = %lu kHz\n", fu ); Serial.print( tbuf );
}

void CC1101::demo( byte c )
{
byte adr;
switch ( c ) {
  // minuscules : observation
  case 'd' : dump_config();
    break;
  case 'v' :
    snprintf( tbuf, sizeof(tbuf), "version %02x\n", read_status_reg( CC1101_VERSION ) );
    Serial.print( tbuf ); 
    break;
  case '?' :
    read_strobe( CC1101_SNOP );
    snprintf( tbuf, sizeof(tbuf), "FSM state %s, FIFO %d\n", fsm_states[(STATUS >> 4) & 7], STATUS & 0x0F );
    Serial.print( tbuf ); 
    break;
  // majuscules et chiffres : actions
  case 'J':
    adr = CC1101_IOCFG0;
    write_reg( adr, 0x2f ); // test GDO0 : logic 0
    break;
  case 'K':
    adr = CC1101_IOCFG0;
    write_reg( adr, 0x40 | 0x2f ); //  test LED : logic 1
    break;
  case 'F' :    // FSK manuel, use JK
    preset_P10AF();
    break;
  case '3' :
    unsigned long fu = 433000;
    fu <<= 12;
    fu /= 1625;
    set_synth_frequ( fu );
    break;
      // les strobes
  case 'Z' :
    read_strobe( CC1101_SRES );
    break;
  case 'I' :
    read_strobe( CC1101_SIDLE );
    break;
  case 'C' :
    read_strobe( CC1101_SCAL );
    break;
  case 'R' :
    read_strobe( CC1101_SRX );
    break;
  case 'S' :
    read_strobe( CC1101_SFSTXON );
    break;
  case 'T' :
    read_strobe( CC1101_STX );
    break;

  }
}
