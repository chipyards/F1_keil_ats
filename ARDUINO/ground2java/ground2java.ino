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

// calcul CRC32 AIXM
unsigned long crc_aixm( const unsigned char *buf, byte leng )
{
unsigned long crc = 0;
byte i, j;
byte lebyte, topreg;
for ( j = 0; j < leng; j++ ) 
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
return crc;
}

// verification de CRC32 dans packet p (le crc est a p + (p[0]-3))
// retour 1 si ok
int CRC32ok( byte * p )
{
byte len = p[0];
unsigned long local_crc = crc_aixm( p + 1, len - 4 );
unsigned long rx_crc = from_32le( p + (len-3) );
if  ( local_crc != rx_crc )
  {
  snprintf( CC.tbuf, sizeof(CC.tbuf), "BAD CRC %08lx vs %08lx\n", local_crc, rx_crc );
  Serial.println( CC.tbuf );
  return 0;
  }
snprintf( CC.tbuf, sizeof(CC.tbuf), "GOOD CRC %08lx\n", rx_crc );
Serial.println( CC.tbuf );
return 1;
}

// append a crc to a packet (with p[0]=len already including 4 crc bytes)
void appendCRC( unsigned char * p ) {
  byte len = p[0];
  unsigned long local_crc = crc_aixm( p + 1, len - 4 );
  snprintf( CC.tbuf, sizeof(CC.tbuf), "CRC %08lx", local_crc );
  Serial.println( CC.tbuf );
  to_32le( p + len - 3, local_crc );
}

void setup() {
  Serial.begin(38400);
  int r = CC.GFSK_radio_init();
  if  ( r == 0 )
      Serial.println("Radio init done");
  else Serial.println("Radio init error");
}

byte default_plan[] = {148,152,24,49,210,8,167,94,198,210,214,84,227,195,128,0};
 
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
      case 'n': {
        data[2] = 0x70;              // NEWFP
        byte i = 3; 
        for ( int j = 0; j < sizeof(default_plan); j++ )
            data[i++] = default_plan[j];
        // here, LEN would be i - 1
        data[0] = i + 3;  // add 4 for CRC, so LEN - i + 3
        appendCRC( data );  
        } break;
      // space after single letter command is accepted but not mandatory  
      case 'd':
        data[2] = 0x4A; data[0] = 7;  // DIRECT
        if ( buf[1] == ' ' )
           data[3] = atoi(buf+2); // 1 space skipped
        else data[3] = atoi(buf+1); // no space, ok
        appendCRC( data );  
        break; 
      case 't':
        data[2] = 0x5E; data[0] = 8;  // TURN
        if ( buf[1] == ' ' )
           to_16le( data + 3, atoi(buf+2) );
        else to_16le( data + 3, atoi(buf+1) );
        appendCRC( data );  
        break; 
      case 'a':
        data[2] = 0x14; data[0] = 8;  // NEWFL
        if ( buf[1] == ' ' )
           to_16le( data + 3, atoi(buf+2) );
        else to_16le( data + 3, atoi(buf+1) );
        appendCRC( data );  
        break;
      // --- pilot info requests REPFP=0x72, REPWCO=0x76, REPALT=0x16, REPRAT=0x7A
      case 'c' : data[2] = 0x71; data[0] = 2; // REQFP
        break;
      case 'w' : data[2] = 0x75; data[0] = 3; // REQWCO
        if ( buf[1] == ' ' )
           data[3] = atoi(buf+2);
        else data[3] = atoi(buf+1);
        break;
      case 'l' : data[2] = 0x15; data[0] = 2; // REQALT
        break;
      case 'r' : data[2] = 0x79; data[0] = 2; // REQRAT
        break;
      // --- simulation commands PAUSE=0x81, RESUME=0x82, RATECK=0x83, SRESET=0x84, SNEWFP=0x85
      case 'P': data[2] = 0x81; data[0] = 2;  // PAUSE
        break; 
      case 'R': data[2] = 0x82; data[0] = 2;  // RESUME
        break; 
      case 'K': data[2] = 0x83; data[0] = 3;  // RATECK
        if ( buf[1] == ' ' )
           data[3] = atoi(buf+2);
        else data[3] = atoi(buf+1);
        break; 
      case 'Z': data[2] = 0x84; data[0] = 2;  // SRESET
        break;
      case 'N': {                             // SNEWFP
        data[2] = 0x85;
        byte i = 3; 
        for ( int j = 0; j < sizeof(default_plan); j++ )
            data[i++] = default_plan[j];
        // here, LEN would be i - 1
        data[0] = i + 3;  // add 4 for CRC, so LEN - i + 3
        appendCRC( data );  
        } break;
      default: return;
      } // switch
  int resu = CC.tx_if_can( data );
  if ( resu ) Serial.println("tx error");
  else Serial.println("tx ok"); 
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
    case 0x72: { // REPFP
        if  ( CRC32ok( data ) )
            {
            byte j = 3;
            Serial.print("c { ");
            while ( j < ( data[0] - 3 ) )
                  { Serial.print(data[j++]); Serial.print(' '); }
            Serial.println('}'); 
            }
    } break;
    case 0x76: { // REPWCO
        if  ( CRC32ok( data ) )
            {
            byte j = 3;
            Serial.print("w ");
            Serial.print(data[j++]); Serial.print(" (");
            int x, y;
            while ( j < ( data[0] - 3 ) )
                  {
                  x = from_16le( data + j ); j += 2;
                  y = from_16le( data + j ); j += 2;
                  snprintf( CC.tbuf, sizeof(CC.tbuf), "%d:%d ", x, y );
                  Serial.print( CC.tbuf );
                  }
            Serial.println(')'); 
            }
    } break;
    case 0x16: { // REPALT
        if  ( CRC32ok( data ) )
            {
            byte j = 3;
            Serial.print("l ");
            int x = from_16le( data + j ); j += 2;
            int y = from_16le( data + j ); j += 2;
            snprintf( CC.tbuf, sizeof(CC.tbuf), "FLMIN %d FLMAX %d ", x, y );
            Serial.println( CC.tbuf );
            }
    } break;
    case 0x7A: { // REPRAT
        if  ( CRC32ok( data ) )
            {
            byte j = 3;
            Serial.print("r ");
            int x = from_16le( data + j ); j += 2;
            int y = from_16le( data + j ); j += 2;
            snprintf( CC.tbuf, sizeof(CC.tbuf), "+ %d ft/mn, - %d ft/mn, bank %d deg", x, y, data[j] );
            Serial.println( CC.tbuf );
            }
    } break;
    default:
        CC.format_rx_to_Serial( data );
    }
	}
else	CC.format_rx_to_Serial( data );
}

void loop() {
static byte oldGDO0 = 0;
static char buf[116];
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
