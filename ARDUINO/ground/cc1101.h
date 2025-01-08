#include "cc1101_constants.h"

extern const unsigned char full_patable[];

// SPI1 en full duplex
void SPI1_init(void);

// ecrire et lire cnt bytes en une transaction
void SPI1_multi_byte( unsigned char * txbuf, unsigned char * rxbuf, byte cnt );

#define STATUS (rxbuf[0])	// permet de recuperer les status apres toute operation

class CC1101 {
public:
unsigned char txbuf[65];
unsigned char rxbuf[65];
char beacon_tx_enable;
char tbuf[64];  // pour snprintf

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

// set_gets, short bitfields

// 1 bit
void set1( byte opt, byte reg, byte pos ) {
	byte tmp = read_reg( reg ) & ~( 1 << pos );	// preserve others
	write_reg( reg, tmp | ( ( opt & 1 ) << pos ) );
	}
byte get1( byte reg, byte pos ) {
	return( ( read_reg( reg ) >> pos ) & 1 );
	}
// 2 bits
void set2( byte opt, byte reg, byte pos ) {
	byte tmp = read_reg( reg ) & ~( 3 << pos );	// preserve others
	write_reg( reg, tmp | ( ( opt & 3 ) << pos ) );
	}
byte get2( byte reg, byte pos ) {
	return( ( read_reg( reg ) >> pos ) & 3 );
	}
// 3 bits
void set3( byte opt, byte reg, byte pos ) {
	byte tmp = read_reg( reg ) & ~( 7 << pos );	// preserve others
	write_reg( reg, tmp | ( ( opt & 7 ) << pos ) );
	}
byte get3( byte reg, byte pos ) {
	return( ( read_reg( reg ) >> pos ) & 7 );
	}
// 4 bits
void set4( byte opt, byte reg, byte pos ) {
  byte tmp = read_reg( reg ) & ~( 0xF << pos ); // preserve others
  write_reg( reg, tmp | ( ( opt & 0xF ) << pos ) );
  }
byte get4( byte reg, byte pos ) {
  return( ( read_reg( reg ) >> pos ) & 0xF );
  }

// set_gets specialises

// synth frequ : 24 bits
unsigned long int get_synth_frequ() {
  unsigned char * fbuf = read_regs( CC1101_FREQ2, 3 );
  unsigned long int f = ( ((unsigned long int)fbuf[0]) << 16 ) | (unsigned int)( fbuf[1] << 8 ) | (unsigned int)fbuf[2];
  return f;
  };
void set_synth_frequ( unsigned long int fu ) {
  unsigned char src[] = { (unsigned char)(fu>>16), (unsigned char)(fu>>8), (unsigned char)fu };
  write_regs( CC1101_FREQ2, src, 3 );
  };
// data rate aka symbol rate : 8 + 4 bits
void get_data_rate( byte *M, byte *E ) {
	*M = read_reg( CC1101_MDMCFG3 );
	*E = read_reg( CC1101_MDMCFG4 ) & 0x0F;
	};
void set_data_rate( byte M, byte E ) {
	write_reg( CC1101_MDMCFG3, M );
	byte tmp = read_reg( CC1101_MDMCFG4 ) & 0xF0;	// preserver bandwidth
	write_reg( CC1101_MDMCFG4, tmp | E );
	};
// FM deviation (for FSK) or smoothing fraction (MSK) 3 + 3 bits
void get_deviation( byte *M, byte *E ) {
	byte r = read_reg( CC1101_DEVIATN );
	*E =  ( r >> 4 ) & 7;
	*M =    r        & 7;
	};
void set_deviation( byte M, byte E ) {
	write_reg( CC1101_DEVIATN, ( ( E & 7 ) << 4 ) | ( M & 7 ) );
	};
// bandwidth, aka channel filter bandwidth ( E = 2 MSBs, M = 2 LSBs )
void get_bandwidth( byte *M, byte *E ) {
	byte r = read_reg( CC1101_MDMCFG4 );
	*E =  ( r >> 6 ) & 3;
	*M =  ( r >> 4 ) & 3;
	};
void set_bandwidth( byte M, byte E ) {
	byte tmp = read_reg( CC1101_MDMCFG4 ) & 0x0F;	// preserve data rate
	write_reg( CC1101_MDMCFG4, tmp | ( ( E & 3 ) << 6 ) | ( ( M & 3 ) << 4 ) );
	};

// Intermediate frequency aka IF - 4 bits
byte get_IF()				{ return read_reg( CC1101_FSCTRL1 ) & 0x0F; }
void set_IF( byte f4 )			{ write_reg( CC1101_FSCTRL1, f4 & 0x0F ); }
// fifo thresholds 4 bits (2 x 6 bits encoded into 4)
void set_fifo_thr( byte opt )  { set4( opt, CC1101_FIFOTHR, 0 ); }
byte get_fifo_thr()       { return get4( CC1101_FIFOTHR, 0 ); }
// packet length 8 bits
void set_pkt_len( byte len )		{ write_reg( CC1101_PKTLEN, len ); }
byte get_pkt_len()			{ return read_reg( CC1101_PKTLEN ); }
// Preamble Quality (PQI) threshold PQT 3 bits (0=no_check, else 4*pqt transitions are required)
void set_PQT( byte pqt )		{ set3( pqt, CC1101_PKTCTRL1, 5 ); }
byte get_PQT()				{ return get3( CC1101_PKTCTRL1, 5 ); }
// enable CRC autoflush 1 bit
void set_CRC_autoflush( byte opt )	{ set1( opt, CC1101_PKTCTRL1, 3 ); }
byte get_CRC_autoflush()		{ return get1( CC1101_PKTCTRL1, 3 ); }
// enable append status 1 bit
void set_append_status( byte opt )	{ set1( opt, CC1101_PKTCTRL1, 2 ); }
byte get_append_status()		{ return get1( CC1101_PKTCTRL1, 2 ); }
// enable adress_check 2 bits (0=0, 1=nobroadcast, 2=(00=broadcast))
void set_adress_check( byte adr_opt )	{ set2( adr_opt, CC1101_PKTCTRL1, 0 ); }
byte get_adress_check()			{ return get2( CC1101_PKTCTRL1, 0 ); }
// enable whitening 1 bit
void set_whiten( byte opt )		{ set1( opt, CC1101_PKTCTRL0, 6 ); }
byte get_whiten()			{ return get1( CC1101_PKTCTRL0, 6 ); }
// packet format 2 bits (0=normal, 3=async)
void set_packet_format( byte format )	{ set2( format, CC1101_PKTCTRL0, 4 ); }
byte get_packet_format()		{ return get2( CC1101_PKTCTRL0, 4 ); }
// CRC enable 1 bit
void set_CRC( byte opt )		{ set1( opt, CC1101_PKTCTRL0, 2 ); }
byte get_CRC()				{ return get1( CC1101_PKTCTRL0, 2 ); }
// packet len config 2 bits (0=fixed, 1=var)
void set_packet_len_config( byte cfg )	{ set2( cfg, CC1101_PKTCTRL0, 0 ); }
byte get_packet_len_config()		{ return get2( CC1101_PKTCTRL0, 0 ); }
// address 8 bits
void set_adr( byte len )		{ write_reg( CC1101_ADDR, len ); }
byte get_adr()				{ return read_reg( CC1101_ADDR ); }
// current optim (no dc filt) 1 bit
void set_no_dc_filt( byte opt )		{ set1( opt, CC1101_MDMCFG2, 7 ); }
byte get_no_dc_filt()			{ return get1( CC1101_MDMCFG2, 7 ); }
// modulation method 3 bits
void set_modu( byte modulation )	{ set3( modulation, CC1101_MDMCFG2, 4 ); }
byte get_modu()				{ return get3( CC1101_MDMCFG2, 4 ); }
// sync mode 3 bits (3=30/32, 7=30/32+CS)
void set_sync_mode( byte sync )		{ set3( sync, CC1101_MDMCFG2, 0 ); }
byte get_sync_mode()			{ return get3( CC1101_MDMCFG2, 0 ); }
// preamble size 3 bits (2, 3, 4, 6, 8, 12, 16, 24 bytes) should match PQT
void set_preamble( byte size )		{ set3( size, CC1101_MDMCFG1, 4 ); }
byte get_preamble()			{ return get3( CC1101_MDMCFG1, 4 ); }
// CCA mode 2 bits (0=always, 3=RSSI low andnot receiving a packet)
void set_CCA( byte opt )		{ set2( opt, CC1101_MCSM1, 4 ); }
byte get_CCA()				{ return get2( CC1101_MCSM1, 4 ); }
// RXOFF mode 2 bits (3=stay in RX)
void set_RXOFF( byte opt )		{ set2( opt, CC1101_MCSM1, 2 ); }
byte get_RXOFF()			{ return get2( CC1101_MCSM1, 2 ); }
// TXOFF mode 2 bits (3=goto RX)
void set_TXOFF( byte opt )		{ set2( opt, CC1101_MCSM1, 0 ); }
byte get_TXOFF()			{ return get2( CC1101_MCSM1, 0 ); }
// auto calibration 2 bits (0=0, 1=avant RX ou TX, 2=apres RX ou TX)
void set_autocal( int opt )		{ set2( opt, CC1101_MCSM0, 4 ); }
byte get_autocal()			{ return get2( CC1101_MCSM0, 4 ); }
// frequ offset compensation 2 bits (0=0, 1=BW/8, 2=BW/4, 3=BW/2)
void set_FOC_limit( int opt )		{ set2( opt, CC1101_FOCCFG, 0 ); }
byte get_FOC_limit()			{ return get2( CC1101_FOCCFG, 0 ); }
// bit sync 2 bits (0=0, 1=3.125%)
void set_BS_limit( int opt )		{ set2( opt, CC1101_BSCFG, 0 ); }
byte get_BS_limit()			{ return get2( CC1101_BSCFG, 0 ); }
// freeze FOC & BS until CS ok 1 bit
void set_FOC_BS_gate( int opt )		{ set1( opt, CC1101_FOCCFG, 5 ); }
byte get_FOC_BS_gate()			{ return get1( CC1101_FOCCFG, 5 ); }
// max index to use in PATABLE, aka PA_POWER 3 bits
void set_power( byte power )		{ set3( power, CC1101_FREND0, 0 ); }
byte get_power()			{ return get3( CC1101_FREND0, 0 ); }

unsigned char * get_patable() {
  return read_regs( 0x3E, 8 );
  }
void set_patable( const unsigned char *src ) {
  write_regs( 0x3E, src, 8 );
  }

// human friendly conversions
unsigned long synth_frequ_from_kHz( unsigned long fu ) {
    fu <<= 12; fu += 812; fu /= 1625;
    return fu;
    }
unsigned long synth_frequ_to_kHz( unsigned long fk ) {
    fk *= 1625; fk += (1<<11); fk >>= 12;
    return fk;
    }

// experiments
void dump_config();
void dump_patable();

void demo( byte c );

byte simple_radio_init();  // return 0 if CC1101 responds Ok
byte tx_if_can( const char * tbuf, byte len );  // return 0 if sent Ok
void handle_rx_to_Serial();

// GFSK modulation
void preset_P10G() {
// X-tal frequency = 26 MHz
// RX filterbandwidth = 101.562500 kHz
// Deviation = 19 kHz
// Datarate = 9.992599 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 433.999969 MHz
// Channel spacing = 199.951172 kHz
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 61
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (41) CHIP_RDY
write_reg(CC1101_FSCTRL1,  0x06); // 0B Frequency synthesizer control.
write_reg(CC1101_FSCTRL0,  0x00); // 0C Frequency synthesizer control.
write_reg(CC1101_FREQ2,    0x10); // 0D Frequency control word, high byte.
write_reg(CC1101_FREQ1,    0xB1); // 0E Frequency control word, middle byte.
write_reg(CC1101_FREQ0,    0x3B); // 0F Frequency control word, low byte.
write_reg(CC1101_MDMCFG4,  0xC8); // 10 Modem configuration.
write_reg(CC1101_MDMCFG3,  0x93); // 11 Modem configuration.
write_reg(CC1101_MDMCFG2,  0x13); // 12 Modem configuration.
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
write_reg(CC1101_IOCFG2,   0x29); // 00 GDO2 output pin configuration.
write_reg(CC1101_IOCFG0,   0x06); // 02 GDO0 output pin configuration. Refer to SmartRF® Studio User Manual for detailed pseudo register explanation.
write_reg(CC1101_PKTCTRL1, 0x04); // 07 Packet automation control.
write_reg(CC1101_PKTCTRL0, 0x05); // 08 Packet automation control.
write_reg(CC1101_ADDR,     0x00); // 09 Device address.
write_reg(CC1101_PKTLEN,   61); // 06 Packet length.
};

void preset_P10Gplus() {
read_strobe( CC1101_SIDLE );
preset_P10G();
set_synth_frequ( synth_frequ_from_kHz(BASE_TUNING+FINE_TUNING) );
set_pkt_len(61);
set_PQT(0);
set_append_status(1);
set_adress_check(0);
set_whiten(0);
set_packet_format(0);
set_packet_len_config(1);
set_no_dc_filt(0);
set_sync_mode(3);
set_preamble(2);  // The recommended setting is 4-byte preamble and 4-byte sync word,
set_CCA(0);
set_RXOFF(3);
set_TXOFF(3);
set_autocal(1);
set_FOC_limit(0);
set_BS_limit(0);
#ifdef USE_CC1101_CRC
 set_CRC(1);
 set_CRC_autoflush(1); // avoid RX overflow
 write_reg(CC1101_IOCFG0, CC1101_GDO_CRC_OK );    // 7
#else
 set_CRC(0);
 set_fifo_thr( 15 );   // le max, pour que GDO0 soit active uniquement a la fin du paquet EOP
 write_reg(CC1101_IOCFG0, CC1101_GDO_RXEND );    // 1 (RXFIFO >= RX FIFO_THR) || EOP (default THR = 32)
#endif
write_reg(CC1101_IOCFG2, CC1101_GDO_P_IN_PROC );  // Tx or Rx in process, from sync to end
set_patable( full_patable );
set_power( 4 ); // 0 dBm
}

/* Async transparent mode, FSK modulation by GDO0 */
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
set_synth_frequ( synth_frequ_from_kHz(434024) ); // SMA short, aligner la frequence sur les "blue coil"
}; //*/



}; // class

extern CC1101 CC;
