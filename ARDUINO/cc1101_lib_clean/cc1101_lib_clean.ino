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

#define FLIGHT 101
#define CRC
void interpreter( char buf[], byte buflen ) // zero-terminated string
{
Serial.println( buf );
if  ( buf[0] == '?' ) { 	// type '?' for dumps
    CC.dump_config();
    CC.dump_patable();
    }
else {				// type a number, to be sent to flight with opcode 0
     int n = atoi(buf);
     if ( n != 0 )
        {
        #ifdef CRC
        byte data[8];
        data[0] = FLIGHT;
        data[1] = 0;
        data[2] = n;
        data[3] = n >> 8;
        unsigned long crc = crc_aixm( data, 4 );
        data[4] = crc;
        data[5] = crc >> 8;
        data[6] = crc >> 16;
        data[7] = crc >> 24;
        int resu = CC.tx_if_can( data, 8 );
        #else
        byte data[4];
        data[0] = FLIGHT;
        data[1] = 0;
        data[2] = n;
        data[3] = n >> 8;
        int resu = CC.tx_if_can( data, 4 );
        #endif
        if ( resu ) Serial.println("tx error");
        else Serial.println("tx ok"); 
        }
     else {
        int resu = CC.tx_if_can( (byte *)buf, buflen );
        if ( resu ) Serial.println("tx error");
        else Serial.println("tx ok");
        } 
     }
}

void handle_rx()
{
byte * data = CC.extract_rx();
if	( ( data[0] >= 4 ) && ( data[1] == (FLIGHT|0x80) ) && ( data[2] == 0 ) )
	{
	Serial.println( ( data[3] & 0xff ) | ( data[4] << 8 ) );
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
          interpreter( buf, j );
          j = 0;  // get ready for next message
          }
      else {
          buf[j] = c;
          j += 1;
      }
    }
if  ( ( digitalRead(2) ) && ( oldGDO0 == 0 ) ) {
    delay(2);
    handle_rx();
    oldGDO0 = 1;
    }
else oldGDO0 = 0;
}
