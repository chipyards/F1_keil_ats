#include "cc1101.h"

void setup() {
  Serial.begin(9600);
  SPI1_init();
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
