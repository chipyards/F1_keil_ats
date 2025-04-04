#include "cc1101.h"

unsigned long crc_aixm( const unsigned char *buf, unsigned char len )
{
unsigned long crc = 0;
char i;
unsigned char lebyte, topreg;
do  {
    lebyte = *(buf++);
    for ( i = 0; i < 8; i++ )
        {
        // topreg = ((unsigned char *)&crc)[3];  // very bad idea
        topreg = crc >> 24;  // high byte
        crc <<= 1;
        if  ( ( lebyte ^ topreg ) & 0x80 )  // test the MSB
            crc ^= 0x814141ABL;
        lebyte <<= 1;
        }
    } while (--len);
return crc;
}

void setup() {
  Serial.begin(9600);
  int r = CC.GFSK_radio_init();
  if  ( r == 0 )
      Serial.println("Radio init done");
  else Serial.println("Radio init error");
}

#define FLIGHT 103
#define CRC
void interpreter( char buf[], byte buflen ) // zero-terminated string
{
// Serial.println( buf );
if  ( buf[0] == '?' ) { 	// type '?' for dumps
    CC.dump_config();
    CC.dump_patable();
    }
else {
  byte data[61]; byte LEN;
  data[0] = FLIGHT;
  switch ( buf[0] ) {
      case 'P': data[1] = 0x81; LEN = 2;
        break; 
      case 'R': data[1] = 0x82; LEN = 2;
        break; 
      case 'K': data[1] = 0x83; LEN = 3; // no space after K
        data[2] = atoi(buf+1);
        break; 
      case 'Z': data[1] = 0x84; LEN = 2;
        break;
      default: return;
      } // switch
      int resu = CC.tx_if_can( data, LEN );
      if ( resu ) Serial.println("tx error");
      else Serial.println("tx ok"); 
        /*      #ifdef CRC
        unsigned long crc = crc_aixm( data, 4 );
        data[4] = crc;
        data[5] = crc >> 8;
        data[6] = crc >> 16;
        data[7] = crc >> 24;
        int resu = CC.tx_if_can( data, 8 );
        #end
        */
  } // if '?'
}

int from_s16le( byte * bbuf ) {
  return ( ( bbuf[0] & 0xff ) | ( bbuf[1] << 8 ) );
  }

void handle_rx()
{
byte * data = CC.extract_rx();
if ( data[1] == (FLIGHT|0x80) )
  {
  switch  ( data[2] )
    {
    case 0x42:  // VAAR : vector report
        int x, y, vx, vy;
        unsigned int fl, t;
        x = from_s16le( data + 3 );
        y = from_s16le( data + 5 );
        vx = from_s16le( data + 7 );
        vy = from_s16le( data + 9 );
        fl = (unsigned int)from_s16le( data + 11 );
        t = (unsigned int)from_s16le( data + 13 );
        snprintf( CC.tbuf, sizeof(CC.tbuf), "R %d %d %d %d %u %u", x, y, vx, vy, fl, t );
        Serial.println( CC.tbuf );
        break; 
    }
	}
else	CC.format_rx_to_Serial( data );
// CC.format_rx_to_Serial( data );
}

void loop() {
static byte oldGDO0 = 0;
static char buf[16];
static int j = 0; // state of the machine !!
int c = Serial.read();
if  ( c != -1 )
    {
      if  ( ( c == 10 ) || ( j >= 15 ) )    // terminator
          {
          buf[j] = 0;   // usual terminator for a C string
          if  ( j ) interpreter( buf, j );
          j = 0;  // get ready for next message
          }
      else {
          buf[j] = c;
          j += 1;
      }
    }
if  ( ( digitalRead(2) ) && ( oldGDO0 == 0 ) ) {
    // delay(2);
    handle_rx();
    oldGDO0 = 1;
    }
else oldGDO0 = 0;
}
