#include "cc1101_constants.h"

// SPI1 en full duplex
void SPI1_init(void);

// ecrire et lire cnt bytes en une transaction
void SPI1_multi_byte( unsigned char * txbuf, unsigned char * rxbuf, byte cnt );

#define STATUS (rxbuf[0])	// permet de recuperer les status apres toute operation

class CC1101 {
public:
unsigned char txbuf[65];
unsigned char rxbuf[65];
char tbuf[32];  // pour snprintf

void write_strobe( byte val ) {
	txbuf[0] = val & 0x3f;
	SPI1_multi_byte( txbuf, rxbuf, 1 );
	};
// read strobe performs the strobe action as well, the difference is the FIFO state reported in the status byte
void read_strobe( byte val ) {
	txbuf[0] = 0x80 | ( val & 0x3f );
	SPI1_multi_byte( txbuf, rxbuf, 1 );
	};

void write_reg( byte adr, byte val ) {
	txbuf[0] = adr & 0x3f;
	txbuf[1] = val;
	SPI1_multi_byte( txbuf, rxbuf, 2 );
	};
unsigned char read_reg( byte adr ) {
	txbuf[0] = 0x80 | ( adr & 0x3f );
	// txbuf[1] = 0;
	SPI1_multi_byte( txbuf, rxbuf, 2 );
	return rxbuf[1];
	};

// les registres de status sont lus aux adresses >= 0x30, avec le bit B (burst)
unsigned char read_status_reg( byte adr ) {
	txbuf[0] = 0xC0 | ( adr & 0x3f );
	SPI1_multi_byte( txbuf, rxbuf, 2 );
	return rxbuf[1];
	};

// burst read, rend l'adresse d'un array de bytes
unsigned char * read_regs( byte start_adr, byte cnt );

// burst write, prend l'adresse d'un array de bytes
void write_regs( byte start_adr, const unsigned char * src, byte cnt );

// set_gets specialises

// synth frequ : 24 bits
unsigned long int get_synth_frequ() {
  unsigned char * fbuf = read_regs( CC1101_FREQ2, 3 );
  unsigned long int f = ( ((unsigned long int)fbuf[0]) << 16 ) | (unsigned int)( fbuf[1] << 8 ) | (unsigned int)fbuf[2];
  //unsigned long int f = (unsigned long int)fbuf[0];
  //f <<= 16;
  //f |= (unsigned int)(fbuf[1] << 8);
  //f |= (unsigned int)fbuf[2];
  return f;
  };
void set_synth_frequ( unsigned long int fu ) {
  unsigned char src[] = { (unsigned char)(fu>>16), (unsigned char)(fu>>8), (unsigned char)fu };
  write_regs( CC1101_FREQ2, src, 3 );
  };

// experiments
void dump_config();

void demo( byte c );

// Async transparent mode, FSK modulation by GDO0
void preset_P10AF() {
// Product = CC1101
// Chip version = A   (VERSION = 0x04)
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 101.562500 kHz
// Deviation = 19 kHz
// Datarate = 9.992599 kBaud
// Modulation = (0) 2-FSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 433.999969 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (3) Asynchronous transparent mode. Data in on GDO0 and Data out on either of the GDO pins
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
write_reg(CC1101_FSCTRL1,  0x06); // 0B Frequency synthesizer control.
write_reg(CC1101_FSCTRL0,  0x00); // 0C Frequency synthesizer control.
write_reg(CC1101_FREQ2,    0x10); // 0D Frequency control word, high byte.
write_reg(CC1101_FREQ1,    0xB1); // 0E Frequency control word, middle byte.
write_reg(CC1101_FREQ0,    0x3B); // 0F Frequency control word, low byte.
write_reg(CC1101_MDMCFG4,  0xC8); // 10 Modem configuration.
write_reg(CC1101_MDMCFG3,  0x93); // 11 Modem configuration.
write_reg(CC1101_MDMCFG2,  0x00); // 12 Modem configuration.
write_reg(CC1101_MDMCFG1,  0x22); // 13 Modem configuration.
write_reg(CC1101_MDMCFG0,  0xF8); // 14 Modem configuration.
write_reg(CC1101_CHANNR,   0x00); // 0A Channel number.
write_reg(CC1101_DEVIATN,  0x34); // 15 Modem deviation setting (when FSK modulation is enabled).
write_reg(CC1101_FREND1,   0x56); // 21 Front end RX configuration.
write_reg(CC1101_FREND0,   0x10); // 22 Front end TX configuration.
write_reg(CC1101_MCSM0,    0x18); // 18 Main Radio Control State Machine configuration.
write_reg(CC1101_FOCCFG,   0x16); // 19 Frequency Offset Compensation Configuration.
write_reg(CC1101_BSCFG,    0x6C); // 1A Bit synchronization Configuration.
write_reg(CC1101_AGCCTRL2, 0x43); // 1B AGC control.
write_reg(CC1101_AGCCTRL1, 0x40); // 1C AGC control.
write_reg(CC1101_AGCCTRL0, 0x91); // 1D AGC control.
write_reg(CC1101_FSCAL3,   0xE9); // 23 Frequency synthesizer calibration.
write_reg(CC1101_FSCAL2,   0x2A); // 24 Frequency synthesizer calibration.
write_reg(CC1101_FSCAL1,   0x00); // 25 Frequency synthesizer calibration.
write_reg(CC1101_FSCAL0,   0x1F); // 26 Frequency synthesizer calibration.
write_reg(CC1101_FSTEST,   0x59); // 29 Frequency synthesizer calibration.
write_reg(CC1101_TEST2,    0x81); // 2C Various test settings.
write_reg(CC1101_TEST1,    0x35); // 2D Various test settings.
write_reg(CC1101_TEST0,    0x09); // 2E Various test settings.
write_reg(CC1101_FIFOTHR,  0x47); // 03 RXFIFO and TXFIFO thresholds.
write_reg(CC1101_IOCFG0,   0x0C); // 02 GDO0 output pin configuration.
write_reg(CC1101_PKTCTRL1, 0x04); // 07 Packet automation control.
write_reg(CC1101_PKTCTRL0, 0x32); // 08 Packet automation control.
write_reg(CC1101_ADDR,     0x00); // 09 Device address.
write_reg(CC1101_PKTLEN,   0xFF); // 06 Packet length.
};

}; // class

extern CC1101 CC;
