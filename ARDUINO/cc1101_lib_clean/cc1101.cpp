#include <SPI.h>
#include "cc1101.h"

/* SPI, full duplex
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

// write and read cnt bytes in one transaction
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
/// singleton
///
CC1101 CC;

///
/// CC1101 methods
///

// burst read, returns the addresse of a byte buffer
// this buffer may be modified by a subsequent read_reg() NOT THREAD SAFE
unsigned char * CC1101::read_regs( byte start_adr, byte cnt ) {
  txbuf[0] = 0xC0 | ( start_adr & 0x3f );
  SPI1_multi_byte( txbuf, rxbuf, cnt+1 );
  return rxbuf+1;
  }
// burst write, takes the addresse of a byte array
void CC1101::write_regs( byte start_adr, const unsigned char * src, byte cnt ) {
  txbuf[0] = 0x40 | ( start_adr & 0x3f );
  memcpy( txbuf+1, src, cnt );
  SPI1_multi_byte( txbuf, rxbuf, cnt+1 );
  }


///
/// utility methods
///

// read all 47 registers from 00 to 2E
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

// format radio RX packet to Serial (first byte is length)
void CC1101::format_rx_to_Serial( byte * rxdata )
{
byte len = rxdata[0];  // The packet length is defined excluding the length byte and the CRC
snprintf( tbuf, sizeof(tbuf), "RX len %d, {", len );
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
snprintf( tbuf, sizeof(tbuf),"\" %d dBm, LQI=%u", hrssi/2, LQI & 0x7F );
Serial.println( tbuf );
}

///
/// main API
///

// return 0 if Ok
byte CC1101::GFSK_radio_init()
{
SPI1_init();
delay(1000);
if  ( ( read_reg( CC1101_SYNC1 ) != 0xD3 ) || ( read_reg( CC1101_SYNC0 ) != 0x91 ) )
  return 3;
preset_P38Gplus();
read_strobe( CC1101_SIDLE );
delay(2);
read_strobe( CC1101_SRX );
return 0;
}

// radio TX, return values :
//  0: Ok
//  1: wrong frequ
//  2: TX FIFO not empty
//  3: RX FIFO not empty
//  4: message too big
byte CC1101::tx_if_can( const char * tbuf, byte len )
{
// checks
if  ( read_reg( CC1101_FREQ2 ) != 0x10 )  // securite 416MHz < F < 442MHz bof c'est leger !
  return 1;
if  ( len > 61 ) return 4;
byte rxbytes, txbytes;
rxbytes = read_status_reg( CC1101_RXBYTES );
txbytes = read_status_reg( CC1101_TXBYTES );
if  ( txbytes ) return 2;
if  ( rxbytes ) return 3;
read_strobe( CC1101_SIDLE );
write_reg( 0x3F, len );
write_regs( 0x3F, (const unsigned char *)tbuf, len );
read_strobe( CC1101_STX );
return 0;
}

// extract received data from RX FIFO
// first byte is length of remaining contents (may be 0)
byte * CC1101::extract_rx()
{
char rxbytes = read_status_reg( CC1101_RXBYTES );
if  ( rxbytes == 0 )
    return "";
return read_regs( 0x3F, rxbytes );
}
