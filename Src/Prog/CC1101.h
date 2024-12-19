#include "CC1101_constants.h"

// SPI1 en full duplex
void SPI1_init(void);

// ecrire et lire cnt bytes en une transaction
void SPI1_multi_byte( unsigned char * txbuf, unsigned char * rxbuf, int cnt );

class CC1101 {
public:
unsigned char txbuf[64];
unsigned char rxbuf[64];
unsigned int status;

void write_strobe( int val ) {
	txbuf[0] = val & 0x3f;
	SPI1_multi_byte( txbuf, rxbuf, 1 );
	status = rxbuf[0];
	};
// read strobe performs the strobe action as well, the difference is the FIFO state reported in the status byte
void read_strobe( int val ) {
	txbuf[0] = 0x80 | ( val & 0x3f );
	SPI1_multi_byte( txbuf, rxbuf, 1 );
	status = rxbuf[0];
	};

void write_reg( int adr, int val ) {
	txbuf[0] = adr & 0x3f;
	txbuf[1] = val;
	SPI1_multi_byte( txbuf, rxbuf, 2 );
	status = rxbuf[0];
	};
unsigned int read_reg( int adr ) {
	txbuf[0] = 0x80 | ( adr & 0x3f );
	// txbuf[1] = 0;
	SPI1_multi_byte( txbuf, rxbuf, 2 );
	status = rxbuf[0];
	return rxbuf[1];
	};
// burst read, rend l'adresse d'un array de bytes
unsigned char * read_regs( int start_adr, int cnt );

// burst write, prend l'adresse d'un array de bytes
void write_regs( int start_adr, unsigned char * src, int cnt );

// set_gets specialises

// get 24 bits of synth frequ
unsigned int get_synth_frequ() {
	unsigned char * fbuf = read_regs( CC1101_FREQ2, 3 );
	unsigned int f = ( fbuf[0] << 16 ) | ( fbuf[1] << 8 ) | ( fbuf[2] );
	return f;
	};
// set 24 bits of synth frequ
void set_synth_frequ( unsigned int fu ) {
	unsigned char src[] = { (unsigned char)(fu>>16), (unsigned char)(fu>>8), (unsigned char)fu };
	write_regs( CC1101_FREQ2, src, 3 );
	};
// get data rate
void get_data_rate( unsigned int *M, unsigned int *E ) {
	*M = read_reg( CC1101_MDMCFG3 );
	*E = read_reg( CC1101_MDMCFG4 ) & 0x0F;
	};
// set data rate
void set_data_rate( unsigned int M, unsigned int E ) {
	write_reg( CC1101_MDMCFG3, M );
	unsigned int tmp = read_reg( CC1101_MDMCFG4 ) & 0xF0;	// preserver bandwidth
	write_reg( CC1101_MDMCFG4, tmp | E );
	};
// get channel filter bandwidth ( E = 2 MSBs, M = 2 LSBs )
unsigned int get_bandwidth() {
	return read_reg( CC1101_MDMCFG4 ) >> 4;
	};
// set channel filter bandwidth ( E = 2 MSBs, M = 2 LSBs )
void set_bandwidth( unsigned int EM ) {
	unsigned int tmp = read_reg( CC1101_MDMCFG4 ) & 0x0F;	// preserver data rate
	write_reg( CC1101_MDMCFG4, tmp | ( EM << 4 ) );
	};

void get_config( unsigned char dest );
void get_patable( unsigned char dest );

// float methods
float synth_frequ_to_float( unsigned int fu );		// MHz
unsigned int synth_frequ_from_float( float ff );	// MHz
float data_rate_to_float( unsigned int M, unsigned int E );			// kHz
void data_rate_from_float( unsigned int *M, unsigned int * E, float fK );	// kHz

// experiments
void dump_config();
void dump_patable();
// comparer les 47 registres de 00 a 2E, avec les valeurs de reference
void compare_config( const unsigned char * ref_regs );
void smarties();

void demo( int c );

void preset_async() {
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
// GDO2 signal selection = (11) Serial Clock
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
write_reg(CC1101_IOCFG0,   0x0C); // 02 GDO0 output pin configuration. Refer to SmartRF® Studio User Manual for detailed pseudo register explanation.
write_reg(CC1101_PKTCTRL1, 0x04); // 07 Packet automation control.
write_reg(CC1101_PKTCTRL0, 0x32); // 08 Packet automation control.
write_reg(CC1101_ADDR,     0x00); // 09 Device address.
write_reg(CC1101_PKTLEN,   0xFF); // 06 Packet length.
};

}; // class

extern CC1101 CC;


