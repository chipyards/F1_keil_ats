#include "cc1101.h"



void setup() {
  Serial.begin(115200);
  SPI1_init();
}

void loop() {
  if  ( Serial.available() ) {
      char c = Serial.read();
      CC.demo( c );
      }
}
