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
}; // class

extern CC1101 CC;


