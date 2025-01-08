#include "cc1101.h"

void setup() {
  Serial.begin(9600);
  SPI1_init();
  delay(1000);
  CC.simple_radio_init();
}

void loop() {
  if  ( Serial.available() ) {
      char c = Serial.read();
      CC.demo( c );
      }
  if  ( digitalRead(2) )
      {
      delay(2);
      CC.handle_rx_to_Serial();
      }
}
