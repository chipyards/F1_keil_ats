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

void interpreter( char buf[], byte len ) // zero-terminated string
{
Serial.println( buf );
if  ( buf[0] == '?' ) { 
    CC.dump_config();
    CC.dump_patable();
    }
else {
     int resu = CC.tx_if_can( buf, len );
     if ( resu ) Serial.println("tx error");
     else Serial.println("tx ok"); 
     }
}

void handle_rx()
{
byte * rxdata = CC.extract_rx();
// Serial.println( rxdata[0] );
CC.format_rx_to_Serial( rxdata );
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
