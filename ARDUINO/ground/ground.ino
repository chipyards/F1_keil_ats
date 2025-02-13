#include "cc1101.h"

#define POLY 0x814141ABL
unsigned long crc_aixm( const unsigned char *buf, unsigned char len )
{
unsigned long crc = 0;
char i;
unsigned char lebyte, topreg;
do  {
    lebyte = *(buf++);
    for ( i = 0; i < 8; i++ )
        {
        topreg = ((unsigned char *)&crc)[3];  // high byte
        crc <<= 1;
        if  ( ( lebyte ^ topreg ) & 0x80 )  // test the MSB
            crc ^= POLY;
        lebyte <<= 1;
        }
    } while (--len);
return crc;
}

void setup() {
  Serial.begin(9600);
  SPI1_init();
  // tests CRC
  unsigned long crc;
  char tbuf[12];
  crc = crc_aixm( (const unsigned char *)"C'est imposant", 14 ); // EA9F9ECC
  snprintf( tbuf, sizeof(tbuf), "0x%08lX\n", crc );
  Serial.print( tbuf );
  crc = crc_aixm( (const unsigned char *)"782", 3 ); // 6C297100
  snprintf( tbuf, sizeof(tbuf), "0x%08lX\n", crc );
  Serial.print( tbuf );
  crc = crc_aixm( (const unsigned char *)"480637N0163411E78246.7", 22 ); // 5E5DC940
  snprintf( tbuf, sizeof(tbuf), "0x%08lX\n", crc );
  Serial.print( tbuf );
  
  delay(1000);
  CC.simple_radio_init();
}

void loop() {
  static byte oldGDO0 = 0;
  static char oldcmd = ' ';
  if  ( Serial.available() ) {
      char c = Serial.read();
      if  ( oldcmd == '@' )
          {   // commande pour WUCAM
          char tbuf[2];
          tbuf[0] = '@'; tbuf[1] = c;
          CC.tx_if_can( tbuf, 2 );
          }
      else CC.demo( c );
      oldcmd = c;
      }
  if  ( ( digitalRead(2) ) && ( oldGDO0 == 0 ) ) {
      delay(2); CC.handle_rx(); oldGDO0 = 1;
      }
  else  oldGDO0 = 0;
}
