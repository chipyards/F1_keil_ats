#include <SPI.h>
/*
 * SCK  = 13
 * MISO = 12
 * MOSI = 11
 * SS = 10
 */
void setup() {
  Serial.begin(115200);
  SPI.begin();
  SPI.beginTransaction( SPISettings( 2000000, MSBFIRST, SPI_MODE0 ) );
  pinMode( 10, OUTPUT );
  pinMode( 2, INPUT_PULLUP );  // GDO0

  digitalWrite( 10, 1 );
}

void loop() {
  unsigned char trx[4]; 
  if  ( Serial.available() ) {
      char c = Serial.read();
      
      switch( c ) {
          case '1':
          
            digitalWrite( 10, 0 ); trx[0] = SPI.transfer( 22 ); digitalWrite( 10, 1 );
 
            Serial.print(22); Serial.print("->"); Serial.println(trx[0]);
            break;

          case '2':
            trx[0] = 101; trx[1] = 202;
            Serial.print(trx[0]); Serial.print(", "); Serial.print(trx[1]); Serial.print("->");

            digitalWrite( 10, 0 ); SPI.transfer( trx, 2 ); digitalWrite( 10, 1 );

            Serial.print(trx[0]); Serial.print(", "); Serial.println(trx[1]);
            break;

          case '3':
            trx[0] = 131; trx[1] = 232; trx[2] = 33;
            Serial.print(trx[0]); Serial.print(", ");
            Serial.print(trx[1]); Serial.print(", "); Serial.print(trx[2]); Serial.print("->");

            digitalWrite( 10, 0 ); SPI.transfer( trx, 3 ); digitalWrite( 10, 1 );

            Serial.print(trx[0]); Serial.print(", ");
            Serial.print(trx[1]); Serial.print(", "); Serial.println(trx[2]);
            break;

          case '4':
            trx[0] = 91; trx[1] = 92; trx[2] = 93; trx[3] = 94;
            Serial.print(trx[0]); Serial.print(", "); Serial.print(trx[1]); Serial.print(", ");
            Serial.print(trx[2]); Serial.print(", "); Serial.print(trx[3]); Serial.print("->");

            digitalWrite( 10, 0 ); SPI.transfer( trx, 4 ); digitalWrite( 10, 1 );

            Serial.print(trx[0]); Serial.print(", "); Serial.print(trx[1]); Serial.print(", ");
            Serial.print(trx[2]); Serial.print(", "); Serial.println(trx[3]);
            break;

          case 'g':
            Serial.print("GDO0 = "); Serial.println( digitalRead( 2 ) );
            break;
          }
      }
}
