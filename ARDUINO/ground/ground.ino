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
      Serial.println("Radio reset done");
  else Serial.println("Radio reset error");
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
