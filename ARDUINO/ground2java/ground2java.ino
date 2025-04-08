#include "cc1101.h"

// litte endian utilities (work for unsigned int as well)
void to_16le( unsigned char * buf, int x ) {
  buf[0] = x;
  buf[1] = x >> 8;
  }

void to_32le( unsigned char * buf, long x ) {
  buf[0] = x;
  buf[1] = x >> 8;
  buf[2] = x >> 16;
  buf[3] = x >> 24;
  }

int from_16le( byte * bbuf ) {
  return ( ( bbuf[0] & 0xff ) | ( bbuf[1] << 8 ) );
  }

unsigned long from_32le( byte * bbuf ) {
  unsigned long retval = bbuf[3];
  retval <<= 8; retval |= bbuf[2];
  retval <<= 8; retval |= bbuf[1];
  retval <<= 8; retval |= bbuf[0];
  return retval; 
  }

void append_crc( unsigned char *buf )
{
unsigned long crc = 0;
byte i, j, len;
byte lebyte, topreg;
len = buf[0];
for ( j = 1; j < ( len - 3 ); j++ ) 
    {
    lebyte = buf[j];
    for ( i = 0; i < 8; i++ )
        {
        // topreg = ((unsigned char *)&crc)[3];  // very bad idea
        topreg = crc >> 24;  // high byte
        crc <<= 1;
        if  ( ( lebyte ^ topreg ) & 0x80 )  // test the MSB of each
            crc ^= 0x814141ABL;
        lebyte <<= 1;
        }
    }
snprintf( CC.tbuf, sizeof(CC.tbuf), "CRC %08lx", crc );
        Serial.println( CC.tbuf );
to_32le( buf + len - 3, crc );
}

void setup() {
  Serial.begin(38400);
  int r = CC.GFSK_radio_init();
  if  ( r == 0 )
      Serial.println("Radio init done");
  else Serial.println("Radio init error");
}


#define FLIGHT 103
#define CRC
void interpreter( char buf[], byte buflen ) // zero-terminated string
{
Serial.println( buf );
if  ( buf[0] == '?' ) { 	// type '?' for dumps
    CC.dump_config();
    CC.dump_patable();
    }
else {
  byte data[61];
  data[1] = FLIGHT;
  switch ( buf[0] ) {
      // --- pilot orders NEWFP=0x70, DIRECT=0x4A, TURN=0x5E, NEWFL=0x14,
      //case 'f': data[2] = 0x70; LEN = ;
      //  break; 
      case 'd': data[2] = 0x4A; data[0] = 7;  // DIRECT
        data[3] = atoi(buf+2); // 1 space after d
        append_crc( data );  
        break; 
      case 't': data[2] = 0x5E; data[0] = 8;  // TURN
        to_16le( data + 3, atoi(buf+2) );  // 1 space after t
        append_crc( data );  
        break; 
      case 'l': data[2] = 0x14; data[0] = 8;  // NEWFL
        to_16le( data + 3, atoi(buf+2) );  // 1 space after l
        append_crc( data );  
        break; 
      // --- simulation commands
      case 'P': data[2] = 0x81; data[0] = 2;
        break; 
      case 'R': data[2] = 0x82; data[0] = 2;
        break; 
      case 'K': data[2] = 0x83; data[0] = 3;
        data[3] = atoi(buf+1); // no space after K
        break; 
      case 'Z': data[2] = 0x84; data[0] = 2;
        break;
      default: return;
      } // switch
  int resu = CC.tx_if_can( data );
  if ( resu ) Serial.println("tx error");
  else Serial.println("tx ok"); 
        /*      #ifdef CRC
        unsigned long crc = crc_aixm( data, 4 );
        int resu = CC.tx_if_can( data, 8 );
        #end
        */
  } // if '?'
}


void handle_rx()
{
byte * data = CC.extract_rx();
if ( data[1] == (FLIGHT|0x80) )
  {
  switch  ( data[2] )
    {
    case 0x42: { // VAAR : vector report
        int x, y, vx, vy;
        unsigned int fl, t;
        x = from_16le( data + 3 );
        y = from_16le( data + 5 );
        vx = from_16le( data + 7 );
        vy = from_16le( data + 9 );
        fl = (unsigned int)from_16le( data + 11 );
        t = (unsigned int)from_16le( data + 13 );
        snprintf( CC.tbuf, sizeof(CC.tbuf), "@ %d %d %d %d %u %u", x, y, vx, vy, fl, t );
        Serial.println( CC.tbuf );
    } break;
    case 0x00: { // WILCO
        unsigned long crc = from_32le( data + 3 );
        snprintf( CC.tbuf, sizeof(CC.tbuf), "W %08lx", crc );
        Serial.println( CC.tbuf );
    } break;
    case 0x01: { // UNABLE
        unsigned long crc = from_32le( data + 4 );
        snprintf( CC.tbuf, sizeof(CC.tbuf), "U %02x %08lx", data[3], crc );
        Serial.println( CC.tbuf );
    } break;
    default:
        CC.format_rx_to_Serial( data );
    }
	}
else	CC.format_rx_to_Serial( data );
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
