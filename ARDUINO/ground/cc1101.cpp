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
digitalWrite( 10, 1 );  // SS hi
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
const unsigned char full_patable[] = {
//  -30   -20   -15   -10    0     5     7     10 dBm  (table 39 page 60)
  0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0
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
snprintf( tbuf, sizeof(tbuf), "F = %lu kHz\n", synth_frequ_to_kHz(get_synth_frequ()) );
Serial.print( tbuf );
}

// dump PATABLE
void CC1101::dump_patable()
{
snprintf( tbuf, sizeof(tbuf), "PATABLE : %d -> ", get_power() );
Serial.print( tbuf );
unsigned char * fbuf = get_patable();
for	( byte i = 0; i < 8; i++ )
	{
	snprintf( tbuf, sizeof(tbuf), " %02x", fbuf[i] );
	Serial.print( tbuf );
	}
Serial.print("\n");
}

void CC1101::demo( byte c )
{
byte adr;
switch ( c ) {
  // minuscules : observation
  case 'd' : dump_config(); dump_patable();
    break;
  case 'v' :
  case ' ' : {
    byte fif;
    read_strobe( CC1101_SNOP ); fif = STATUS & 0x0F;
    snprintf( tbuf, sizeof(tbuf), "FSM %s, RXFIFO %s%d", fsm_states[(STATUS >> 4) & 7], ((fif<15)?(""):(">=")), fif );
    Serial.print( tbuf ); 
    write_strobe( CC1101_SNOP ); fif = STATUS & 0x0F;
    snprintf( tbuf, sizeof(tbuf), ", TXFIFO %s%d free\n", ((fif<15)?(""):(">=")), fif );
    Serial.print( tbuf ); 
    } break;
  case '?' : {
    byte rxbytes, txbytes;
    rxbytes = read_status_reg( CC1101_RXBYTES );
    txbytes = read_status_reg( CC1101_TXBYTES );
    snprintf( tbuf, sizeof(tbuf), "RX bytes %d, TX bytes %d\n", rxbytes, txbytes );
    Serial.print( tbuf ); 
    if	( rxbytes )
	      handle_rx();
    } break;
  // majuscules et chiffres : actions
  /* experience CW *
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
  //*/
  case '1' :
  case '2' :
  case '3' :
  case '4' :
    simple_beacon_tx(c);
    break;
  case '!': {	// put some text in tx fifo
	  const char * txt = "C'est imposant pour mon petit corps";
	  unsigned int len = strlen( txt );
	  write_reg( 0x3F, len );
	  write_regs( 0x3F, (const unsigned char *)txt, len );
	  snprintf( tbuf, sizeof(tbuf), "put %d bytes in TX FIFO -> %d\n", len+1, read_status_reg( CC1101_TXBYTES ) );
	  Serial.print( tbuf );
	  } break;
  case 'G' :		// GFSK packet
	  preset_P10Gplus();
	  break;
  case 'A' : {    // AM via GDO0 ou CW
    read_strobe( CC1101_SIDLE );
    preset_P10AF();
    set_modu( CC1101_AM );
    unsigned char patable[] = { 0x60, 0x60, 0, 0, 0, 0, 0, 0 };  // levels 0 dBm
    set_patable( patable );
    set_power( 0 ); // 0 ==> CW
    } break;
  case 'B' :
    beacon_tx_enable = 1;
    break;

  // les strobes
  case 'Z' :
    beacon_tx_enable = 0;
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

byte CC1101::simple_beacon_init()
{
if  ( ( read_reg( CC1101_SYNC1 ) != 0xD3 ) || ( read_reg( CC1101_SYNC0 ) != 0x91 ) )
  return 3;
preset_P10Gplus();
read_strobe( CC1101_SIDLE );
return 0;
}

// radio TX
void CC1101::simple_beacon_tx( byte t )
{
char fbuf[16];
read_strobe( CC1101_SIDLE );
snprintf( fbuf, sizeof(fbuf), "!%d!", t );
byte len = strlen( fbuf );
if  ( read_reg( CC1101_FREQ2 ) != 0x10 )  // securite 416MHz < F < 442MHz bof c'est leger !
  return;
write_reg( 0x3F, len );
write_regs( 0x3F, (const unsigned char *)fbuf, len );
read_strobe( CC1101_STX );
}

// handle radio RX packet to CDC
void CC1101::handle_rx()
{
char rxbytes = read_status_reg( CC1101_RXBYTES );
if  ( rxbytes )
  {
  byte * rxdata = read_regs( 0x3F, rxbytes );
  byte len = rxdata[0];  // The packet length is defined excluding the length byte and the CRC
  snprintf( tbuf, sizeof(tbuf), "RX len %d (%d), {", len, rxbytes );
  Serial.print( tbuf );
  for ( byte i = 0; i < len+3; i++ )
      {
      snprintf( tbuf, sizeof(tbuf), "%02X,", rxdata[i] );  // affichage hexa, len, RSSI et LQI inclus
      Serial.print( tbuf );
      }
  Serial.print("}=\"");
  for ( byte i = 1; i < len+1; i++ )  // affichage payload en texte filtre
      {
      snprintf( tbuf, sizeof(tbuf), "%c", (char(rxdata[i])<' ')?('?'):(rxdata[i]) );
      Serial.print( tbuf );
      }
  int hrssi = (int)((char)rxdata[len+1]) - (2*74);
  byte LQI = rxdata[len+2];
  snprintf( tbuf, sizeof(tbuf),"\" %d half-dBm, CRC=%s, LQI=%u", hrssi, ((LQI&0x80)?("ok"):("err")), LQI & 0x7F );
  Serial.println( tbuf );
  }
}
