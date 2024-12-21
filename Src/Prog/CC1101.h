#include "CC1101_constants.h"

// SPI1 en full duplex
void SPI1_init(void);

// ecrire et lire cnt bytes en une transaction
void SPI1_multi_byte( unsigned char * txbuf, unsigned char * rxbuf, int cnt );

#define STATUS (rxbuf[0])	// permet de recuperer les status apres toute operation

class CC1101 {
public:
unsigned char txbuf[65];
unsigned char rxbuf[65];

void write_strobe( int val ) {
	txbuf[0] = val & 0x3f;
	SPI1_multi_byte( txbuf, rxbuf, 1 );
	};
// read strobe performs the strobe action as well, the difference is the FIFO state reported in the status byte
void read_strobe( int val ) {
	txbuf[0] = 0x80 | ( val & 0x3f );
	SPI1_multi_byte( txbuf, rxbuf, 1 );
	};

void write_reg( int adr, int val ) {
	txbuf[0] = adr & 0x3f;
	txbuf[1] = val;
	SPI1_multi_byte( txbuf, rxbuf, 2 );
	};
unsigned int read_reg( int adr ) {
	txbuf[0] = 0x80 | ( adr & 0x3f );
	// txbuf[1] = 0;
	SPI1_multi_byte( txbuf, rxbuf, 2 );
	return rxbuf[1];
	};

// les registres de status sont lus aux adresses >= 0x30, avec le bit B (burst)
unsigned int read_status_reg( int adr ) {
	txbuf[0] = 0xC0 | ( adr & 0x3f );
	SPI1_multi_byte( txbuf, rxbuf, 2 );
	return rxbuf[1];
	};

// burst read, rend l'adresse d'un array de bytes
unsigned char * read_regs( int start_adr, int cnt );

// burst write, prend l'adresse d'un array de bytes
void write_regs( int start_adr, const unsigned char * src, int cnt );

// set_gets specialises

// synth frequ : 24 bits
unsigned int get_synth_frequ() {
	unsigned char * fbuf = read_regs( CC1101_FREQ2, 3 );
	unsigned int f = ( fbuf[0] << 16 ) | ( fbuf[1] << 8 ) | ( fbuf[2] );
	return f;
	};
void set_synth_frequ( unsigned int fu ) {
	unsigned char src[] = { (unsigned char)(fu>>16), (unsigned char)(fu>>8), (unsigned char)fu };
	write_regs( CC1101_FREQ2, src, 3 );
	};
// data rate aka symbol rate : 8 + 4 bits
void get_data_rate( unsigned int *M, unsigned int *E ) {
	*M = read_reg( CC1101_MDMCFG3 );
	*E = read_reg( CC1101_MDMCFG4 ) & 0x0F;
	};
void set_data_rate( unsigned int M, unsigned int E ) {
	write_reg( CC1101_MDMCFG3, M );
	unsigned int tmp = read_reg( CC1101_MDMCFG4 ) & 0xF0;	// preserver bandwidth
	write_reg( CC1101_MDMCFG4, tmp | E );
	};
// FM deviation (for FSK) or smoothing fraction (MSK) 3 + 3 bits
void get_deviation( unsigned int *M, unsigned int *E ) {
	unsigned int r = read_reg( CC1101_DEVIATN );
	*E =  ( r >> 4 ) & 7;
	*M =    r        & 7;
	};
void set_deviation( unsigned int M, unsigned int E ) {
	write_reg( CC1101_DEVIATN, ( ( E & 7 ) << 4 ) | ( M & 7 ) );
	};
// Intermediate frequency aka IF - 4 bits
unsigned int get_IF() {
	return read_reg( CC1101_FSCTRL1 ) & 0x0F;
	};
void set_IF( unsigned int f4 ) {
	write_reg( CC1101_FSCTRL1, f4 & 0x0F );
	};
// bandwidth, aka channel filter bandwidth ( E = 2 MSBs, M = 2 LSBs )
void get_bandwidth( unsigned int *M, unsigned int *E ) {
	unsigned int r = read_reg( CC1101_MDMCFG4 );
	*E =  ( r >> 6 ) & 3;
	*M =  ( r >> 4 ) & 3;
	};
void set_bandwidth( unsigned int M, unsigned int E ) {
	unsigned int tmp = read_reg( CC1101_MDMCFG4 ) & 0x0F;	// preserver data rate
	write_reg( CC1101_MDMCFG4, tmp | ( ( E & 3 ) << 6 ) | ( ( M & 3 ) << 4 ) );
	};
// max index to use in PATABLE, aka PA_POWER
void set_power( unsigned int power ) {
	unsigned int tmp = read_reg( CC1101_FREND0 ) & 0xF8;	// preserver LODIV_BUF_CURRENT_TX
	write_reg( CC1101_FREND0, tmp | ( power & 7 ) );
	}
unsigned int get_power() {
	return( read_reg( CC1101_FREND0 ) & 7 );
	};
// modulation method
void set_modu( unsigned int modulation ) {
	unsigned int tmp = read_reg( CC1101_MDMCFG2 ) & 0x8F;	// preserver DEM_DCFILT_OFF,SYNC_MODE
	write_reg( CC1101_MDMCFG2, tmp | ( ( modulation & 7 ) << 4 ) );
	}
unsigned int get_modu() {
	return( ( read_reg( CC1101_MDMCFG2 ) >> 4 ) & 7 );
	}


// gets sans set

// RSSI aka Received Signal Strength Indication
int get_RSSI_half_dB() {
	return ( (int)((char)read_status_reg( CC1101_RSSI )) - (2*74) );
	};

void get_patable( unsigned char * dest );
void set_patable( const unsigned char * src );

// float methods
float synth_frequ_to_float( unsigned int fu );		// MHz
unsigned int synth_frequ_from_float( float ff );	// MHz

float data_rate_to_float( unsigned int M, unsigned int E );			// kHz
void data_rate_from_float( unsigned int *M, unsigned int *E, float fK );	// kHz

float deviation_to_float( unsigned int M, unsigned int E );			// kHz
void deviation_from_float( unsigned int *M, unsigned int *E, float fK );	// kHz

float IF_to_float( unsigned int fu );			// kHz
unsigned int IF_from_float( float fk );			// kHz

float bandwidth_to_float( unsigned int M, unsigned int E );			// kHz
void bandwidth_from_float( unsigned int *M, unsigned int *E, float fk );	// kHz

float RSSI_to_float( int hdB );		//dB


// experiments
void dump_config();
void dump_patable();
// comparer les 47 registres de 00 a 2E, avec les valeurs de reference
void compare_config( const unsigned char * ref_regs );
void quick_view();

void smarties();
void quick_set();	// tester les float convs
void demo( int c );

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
write_reg(CC1101_IOCFG0,   0x0C); // 02 GDO0 output pin configuration.
write_reg(CC1101_PKTCTRL1, 0x04); // 07 Packet automation control.
write_reg(CC1101_PKTCTRL0, 0x32); // 08 Packet automation control.
write_reg(CC1101_ADDR,     0x00); // 09 Device address.
write_reg(CC1101_PKTLEN,   0xFF); // 06 Packet length.
};

}; // class

extern CC1101 CC;


