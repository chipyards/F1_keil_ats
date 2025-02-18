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


/* disassembly by godbolt.org, AVR gcc, -std=gnu11 -mmcu=atmega328p -Os
crc_aixm(unsigned char const*, unsigned char):
        mov r20,r24
        mov r21,r22
        mov r18,r24
        mov r19,r25
        ldi r22,0
        ldi r23,0
        movw r24,r22
        add r21,r20
.L4:
        movw r30,r18
        ld r31,Z
        subi r18,-1
        sbci r19,-1
        ldi r30,lo8(8)
.L3:
        mov r20,r25
        eor r20,r31
        lsl r22
        rol r23
        rol r24
        rol r25
        sbrs r20,7
        rjmp .L2
        ldi r20,171
        eor r22,r20
        ldi r20,65
        eor r23,r20
        eor r24,r20
        ldi r20,129
        eor r25,r20
.L2:
        lsl r31
        dec r30
        brne .L3
        cpse r21,r18
        rjmp .L4
        ret
*/

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
