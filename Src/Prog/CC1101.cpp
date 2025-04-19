#include "options.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include <string.h> // pour memcpy
#include <stdio.h>	// pour snprintf
#include "qfplib-m3.h"
#include "gpio.h"
#include "sys.h"
#include "CDC.h"
#include "CC1101.h"

#ifdef USE_TIM3_PC6
#include "pwm.h"	// experience modulation CW en mode asynchrone : relier PC6 a PA10
#endif



// SPI1 en full duplex
void SPI1_init(void)
{
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
LL_SPI_SetTransferDirection( SPI1, LL_SPI_FULL_DUPLEX );
LL_SPI_SetMode( SPI1, LL_SPI_MODE_MASTER );
LL_SPI_SetDataWidth( SPI1, LL_SPI_DATAWIDTH_8BIT );
LL_SPI_SetClockPolarity( SPI1, LL_SPI_POLARITY_LOW );	// idle low
LL_SPI_SetClockPhase( SPI1, LL_SPI_PHASE_1EDGE );	// 1st edge samples incoming data
LL_SPI_SetNSSMode( SPI1, LL_SPI_NSS_SOFT );
LL_SPI_SetTransferBitOrder( SPI1, LL_SPI_MSB_FIRST );
if	( SystemCoreClock > 10000100 )
	LL_SPI_SetBaudRatePrescaler( SPI1, LL_SPI_BAUDRATEPRESCALER_DIV32 );	// 2.25 MHz @ 72 MHz
else	LL_SPI_SetBaudRatePrescaler( SPI1, LL_SPI_BAUDRATEPRESCALER_DIV4 );	// 2 MHz @ 8 MHz
LL_SPI_Enable(SPI1);
}

// ecrire et lire cnt bytes en une transaction
void SPI1_multi_byte( unsigned char * txbuf, unsigned char * rxbuf, int cnt )
{
NSS1_LO();
//tickdelay(8);
while	( IS_MISO_SET() )	// wait for wake-up
	{}
for	( int i = 0; i < cnt; i++ )
	{
	// Wait for spi Tx buffer empty
	while	( !LL_SPI_IsActiveFlag_TXE(SPI1) )
		{}
	// Send spi tx data
	LL_SPI_TransmitData8(SPI1, *(txbuf++) );
	// Wait for spi rx data
	while	( !LL_SPI_IsActiveFlag_RXNE(SPI1) )
		{}
	// Read  received data to clear RXNE */
	*(rxbuf++) = LL_SPI_ReceiveData8( SPI1 );
	}
//tickdelay(8);
NSS1_HI();
//tickdelay(8);
}

///
/// ROM data
///

const unsigned char reset_regs[] = {	// 47 registres au reset, selon doc CC1101.pdf
0x29,0x2E,0x3F,0x07,0xD3,0x91,0xFF,0x04,0x45,0x00,0x00,0x0F,0x00,0x1E,0xC4,0xEC,0x8C,0x22,0x02,0x22,0xF8,0x47,
0x07,0x30,0x04,0x36,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x20,0x0D,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B
};

const char * fsm_states[] = { 		// noms des codes d'etats obtenus dans le status byte
	"IDLE", "RX", "TX", "FSTXON", "CALIB", "SETTLE", "RX_OVER", "TX_UNDER"
	};
const char * mod_methods[] = { 		// noms des methodes de modulation
	"2-FSK", "GFSK", "-", "ASK/OOK", "4-FSK", "-", "-", "MSK"
	};
const char * pack_format[] = { 		// noms des formats, packet ou non
	"NORM", "SYNC", "RANDOM", "ASYNC"
	};
const char * pack_len_conf[] = { 		// noms des modes pour packet length
	"FIXED", "VAR", "INFINITE", ""
	};
const char * sync_word_conf[] = { 		// noms des modes pour sync word matching
	"NONE", "15/16", "16/16", "30/32", "CS", "15/16+CS", "16/16+CS", "30/32+CS"
	};
const int preamble_size[] = {		// nombre de bytes du preambule 10101010
	2, 3, 4, 6, 8, 12, 16, 24
	};
const char * CCA_mode[] = {		// noms des modes de Clear Channel Assignment
	"ALWAYS", "RSSI", "NO_RX_IN_PROGRESS", "RSSI & NO_RX_IN_PROGRESS"
	};
const char * FSM_next_state[] = {	// noms des etats futurs pour RXOFF et TXOFF
	"IDLE", "FSTXON", "TX", "RX"
	};

const unsigned char full_patable[] = {
//	-30   -20   -15   -10    0     5     7     10 dBm  (table 39 page 60)
	0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0
	};

///
/// singleton
///
CC1101 CC;

///
/// CC1101 methods
///

// burst read, rend l'adresse d'un array de bytes PRECAIRE a utiliser immediatement
// ce buffer peut etre altere par un read_reg() interpose NOT THREAD SAFE
unsigned char * CC1101::read_regs( int start_adr, int cnt ) {
	txbuf[0] = 0xC0 | ( start_adr & 0x3f );
	SPI1_multi_byte( txbuf, rxbuf, cnt+1 );
	return rxbuf+1;
	}
// burst write, prend l'adresse d'un array de bytes NOT THREAD SAFE
void CC1101::write_regs( int start_adr, const unsigned char * src, int cnt ) {
	txbuf[0] = 0x40 | ( start_adr & 0x3f );
	memcpy( txbuf+1, src, cnt );
	SPI1_multi_byte( txbuf, rxbuf, cnt+1 );
	}

///
/// float methods
///
float CC1101::synth_frequ_to_float( unsigned int fu )
{
fu *= 26;	// on met Fosc en MHZ pour avoir le resultat en MHz
return qfp_fdiv( float( fu ), 65536.0f );
}

unsigned int CC1101::synth_frequ_from_float( float fM )	// fM en MHz
{
return (unsigned int)qfp_fadd( 0.5, qfp_fmul( 65536.0f, qfp_fdiv( fM, 26.0f ) ) );
}

float CC1101::data_rate_to_float( unsigned int M, unsigned int E )
{
M += 256;
M *= 26000;	// on met Fosc en kHz pour avoir le resultat en kHz
unsigned int D = 1 << ( 28 - E );
return qfp_fdiv( float(M), float(D) );
}

void CC1101::data_rate_from_float( unsigned int *M, unsigned int *E, float fK  ) // fK en kHz
{
fK = qfp_fdiv( fK, 26000.0f );
float tmp = qfp_fmul( fK, float(1<<20) );	// 20 = 28 - 8 car le min de (256+M) est 256
// l'exposant exact pour M = 0
tmp = qfp_fmul( qfp_fln( tmp ), 1.4426950408f );	// 1.443 = 1/ln(2)
// arrondissons-le par defaut (floor) pour que M soit > 0
*E = (unsigned int)tmp;
tmp = qfp_fadd( 0.5, qfp_fmul( fK, float(1<<(28-*E)) ) );
*M = (unsigned int)tmp - 256;
//if	( *M >= 256 )
//	{ *E += 1; *M = 0; }
}

float CC1101::deviation_to_float( unsigned int M, unsigned int E )		// kHz
{
M += 8;
M *= 26000;	// on met Fosc en kHz pour avoir le resultat en kHz
unsigned int D = 1 << ( 17 - E );
return qfp_fdiv( float(M), float(D) );
}

void CC1101::deviation_from_float( unsigned int *M, unsigned int* E, float fK )	// kHz
{
fK = qfp_fdiv( fK, 26000.0f );
float tmp = qfp_fmul( fK, float(1<<14) );	// 14 = 17 - 3 car le min de (8+M) est 8
// l'exposant exact pour M = 0
tmp = qfp_fmul( qfp_fln( tmp ), 1.4426950408f );	// 1.443 = 1/ln(2)
// arrondissons-le par defaut (floor) pour que M soit > 0
*E = (unsigned int)tmp;
tmp = qfp_fadd( 0.5, qfp_fmul( fK, float(1<<(17-*E)) ) );
*M = (unsigned int)tmp - 8;
}

float CC1101::IF_to_float( unsigned int fu )			// kHz
{
fu *= 26000;
return qfp_fdiv( (float)fu, (float)(1<<10) );
}

unsigned int CC1101::IF_from_float( float fk )			// kHz
{
return (unsigned int)qfp_fadd( 0.5, qfp_fmul( (float)(1<<10), qfp_fdiv( fk, 26000.0f ) ) );
}

float CC1101::bandwidth_to_float( unsigned int M, unsigned int E )		// kHz
{
M += 4;
M *= ( 8 * (1 << E) );
// on met Fosc en kHz pour avoir le resultat en kHz
return qfp_fdiv( 26000.0f, float(M) );
}

void CC1101::bandwidth_from_float( unsigned int *M, unsigned int *E, float fK )	// kHz
{
fK = qfp_fdiv( 26000.0f, fK );
float tmp = qfp_fdiv( fK, 32.0f );	// 32 = 8 * 4 car le min de (4+M) est 4
// l'exposant exact pour M = 0
tmp = qfp_fmul( qfp_fln( tmp ), 1.4426950408f );	// 1.443 = 1/ln(2)
// arrondissons-le par defaut (floor) pour que M soit > 0
*E = (unsigned int)tmp;
tmp = qfp_fadd( 0.5, qfp_fdiv( fK, float(1<<(3+*E)) ) );
*M = (unsigned int)tmp - 4;
}

///
/// experiences
///

// lire les 47 registres de 00 a 2E
void CC1101::dump_config()
{
unsigned char * fbuf = read_regs( 0, 0x2F );
for	( unsigned int i = 0; i < 0x2F; i++ )
	CDC_printf("reg %02x : %02x\n", i, fbuf[i] );
CDC_printf("F = %lu kHz\n", synth_frequ_to_kHz(get_synth_frequ()) );
}

// comparer les 47 registres de 00 a 2E, avec les valeurs de reference
void CC1101::compare_config( const unsigned char * ref_regs )
{
CDC_printf("regs : (reset) -> now\n");
unsigned char * fbuf = read_regs( 0, 0x2F );
for	( unsigned int i = 0; i < 0x2F; i++ )
	if	( fbuf[i] != ref_regs[i] )
		CDC_printf("reg %02x : (%02x) -> %02x\n", i, ref_regs[i], fbuf[i] );
}

// dump PATABLE
void CC1101::dump_patable()
{
CDC_printf("PATABLE : %d -> ", get_power() );
unsigned char * fbuf = get_patable();
for	( unsigned int i = 0; i < 8; i++ )
	CDC_printf(" %02x", fbuf[i] );
CDC_printf("\n");
}

void CC1101::quick_view()
{
unsigned int E, M, fu;
float ff;
CDC_printf("version %02x\n", read_status_reg( CC1101_VERSION ) );

fu = get_synth_frequ();
ff = synth_frequ_to_float( fu );
CDC_printf("freq synth 0x%06x -> %6f MHz (%u kHz)\n", fu, ff, synth_frequ_to_kHz( fu ) );

fu = 0x10A900;	// see tx_if_can()
ff = synth_frequ_to_float( fu );
CDC_printf("freq min   0x%06x -> %6f MHz (%u kHz)\n", fu, ff, synth_frequ_to_kHz( fu ) );

fu = 0x10B7FF;	// see tx_if_can()
ff = synth_frequ_to_float( fu );
CDC_printf("freq max   0x%06x -> %6f MHz (%u kHz)\n", fu, ff, synth_frequ_to_kHz( fu ) );

fu = get_IF();
ff = IF_to_float( fu );
CDC_printf("IF %d -> %.2f kHz\n", fu, ff );

get_data_rate( &M, &E );
ff = data_rate_to_float( M, E );
CDC_printf("symbol rate M=%d, E=%d -> %.5f kHz\n", M, E, ff );

get_deviation( &M, &E );
ff = deviation_to_float( M, E );
CDC_printf("deviation M=%d, E=%d -> %.2f kHz\n", M, E, ff );

get_bandwidth( &M, &E );
ff = bandwidth_to_float( M, E );
CDC_printf("bandwidth M=%d, E=%d -> %.2f kHz\n", M, E, ff );

dump_patable();

CDC_printf("packet length : %d\n", get_pkt_len() );
M = get_PQT(); CDC_printf("preamble quality threshold PQT : %d : %d transitions\n", M, M*4 );
CDC_printf("autoflush on CRC error : %d\n", get_CRC_autoflush() );
CDC_printf("append status to Rx packet : %d\n", get_append_status() );
CDC_printf("address Tx and check on Rx : %d\n", get_adress_check() );
CDC_printf("data whitening : %d\n", get_whiten() );
M = get_packet_format(); CDC_printf("packet format : %d : %s\n", M, pack_format[M] );
CDC_printf("CRC enable : %d\n", get_CRC() );
M = get_packet_len_config(); CDC_printf("packet length config : %d : %s\n", M, pack_len_conf[M] );
CDC_printf("address value : %d\n", get_adr() );
CDC_printf("disable DC filter : %d\n", get_no_dc_filt() );
M = get_modu(); CDC_printf("modulation : %d : %s\n", M, mod_methods[M] );
M = get_sync_mode(); CDC_printf("sync word matching : %d : %s\n", M, sync_word_conf[M] );
M = get_preamble(); CDC_printf("preamble size : %d : %d bytes\n", M, preamble_size[M] );
M = get_CCA(); CDC_printf("clear channel assignment CCA : %d : %s\n", M, CCA_mode[M] );
M = get_RXOFF(); CDC_printf("FSM state after RX : %d : %s\n", M, FSM_next_state[M] );
M = get_TXOFF(); CDC_printf("FSM state after TX : %d : %s\n", M, FSM_next_state[M] );
CDC_printf("auto-calibration : %d\n", get_autocal() );
CDC_printf("frequency offset compensation FOC limit : %d\n", get_FOC_limit() );
CDC_printf("bit sync BS limit : %d\n", get_BS_limit() );
CDC_printf("FOC and BS gate with carrier sense CS : %d\n", get_FOC_BS_gate() );
CDC_printf("VCO Calib %02x %02x %02x %02x\n", read_reg(CC1101_FSCAL3),
	   read_reg(CC1101_FSCAL2), read_reg(CC1101_FSCAL1), read_reg(CC1101_FSCAL0) );
}

void CC1101::demo( int c )
{
/* test wiring (sans le transceiver)
	{
	unsigned char echo;
	SPI1_multi_byte( (unsigned char *)&c, &echo, 1 );
	CDC_printf("%02x -> %02x, MISO=%d\n", c, echo, IS_MISO_SET() );
	}
*/
switch	( c ) {
	// minuscules : observation
	case 'c' : compare_config( reset_regs );
		break;
	case 'd' : dump_config(); dump_patable();
		break;
	case 'q' : quick_view();
		break;
	case ' ' : {
		unsigned int fif;
		read_strobe( CC1101_SNOP ); fif = STATUS & 0x0F;
    		CDC_printf("FSM %s, RXFIFO %s%d", fsm_states[(STATUS >> 4) & 7], ((fif<15)?(""):(">=")), fif );
    		write_strobe( CC1101_SNOP ); fif = STATUS & 0x0F;
    		CDC_printf(", TXFIFO %s%d free\n", ((fif<15)?(""):(">=")), fif );
    		} break;
	case '?' : {
		unsigned int rxbytes, txbytes;
		rxbytes = read_status_reg( CC1101_RXBYTES );
		txbytes = read_status_reg( CC1101_TXBYTES );
		CDC_printf("RX bytes %d, TX bytes %d\n", rxbytes, txbytes );
		if	( rxbytes )
			{
			unsigned char * data = extract_rx();
			formatba_rx_to_CDC( data );
			}
		} break;
	// majuscules et chiffres : actions
	case '1' :
		tx_if_can( (unsigned char *)"!1!", 3 );
		break;
	case '!': {	// put some text in tx fifo, then TX
		const char * txt = "C'est imposant pour ton petit corps";
		unsigned int len = strlen( txt );
		int retval = tx_if_can( (unsigned char *)txt, len );
		CDC_printf("sent %d bytes -> tx_if_can returned %d\n", len+1, retval );
		} break;
	case 'J':
		write_reg( CC1101_IOCFG2, 0x2f );	// test LED : logic 0
		GDO0_LO();
		CDC_printf("wrote 0 to GDO0\n");
		break;
	case 'K':
		write_reg( CC1101_IOCFG2, 0x40 | 0x2f ); //  test LED : logic 1
		GDO0_HI();
		CDC_printf("wrote 1 to GDO0\n");
		break;
	case 'F' :		// FSK manuel, use JK
		preset_P10AF();
		write_reg(CC1101_IOCFG0, 0x2E); // Hi Z, for safety when leaving async mode
		quick_view();
		LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING ); // cas ou on a connect PC6 a PA10
		break;
	case 'G' :		// GFSK packet
		preset_P38Gplus();
		quick_view();
		LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING ); // cas ou on a connect PC6 a PA10
		break;
	case 'O' : {		// OOK manuel, (start with T, stop with I), use JK to modulate
		read_strobe( CC1101_SIDLE );
		preset_P10AF();
		write_reg(CC1101_IOCFG0, 0x2E); // Hi Z, for safety when leaving async mode
		set_modu( CC1101_AM );
		unsigned char patable[] = { 0x34, 0xC8, 0, 0, 0, 0, 0, 0 };	// levels -10 dBm and 7 dBm
		set_patable( patable );
		set_power( 1 );
		quick_view();
		LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING ); // cas ou on a connect PC6 a PA10
		} break;
	case 'A' : {		// ASK manuel, (start with T, stop with I), use JK to modulate
		read_strobe( CC1101_SIDLE );
		preset_P10AF();
		write_reg(CC1101_IOCFG0, 0x2E); // Hi Z, for safety when leaving async mode
		set_modu( CC1101_AM );
		set_patable( full_patable );
		set_power( 6 );
		quick_view();
		LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING ); // cas ou on a connect PC6 a PA10
		} break;
	case 'W' : {		// pure CW (start with T, stop with I)
		read_strobe( CC1101_SIDLE );
		preset_P10AF();
		write_reg(CC1101_IOCFG0, 0x2E); // Hi Z, for safety when leaving async mode
		set_modu( CC1101_AM );
		unsigned char patable[] = { 0x60, 0x60, 0, 0, 0, 0, 0, 0 };	// level 0 dBm
		set_patable( patable );
		set_power( 0 );
		quick_view();
		LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING ); // cas ou on a connect PC6 a PA10
		} break;
	case 'B' :		// apres F, O ou A  : modulation 5kHz - please connect PC6 to PA10
		#ifdef USE_TIM3_PC6
		gpio_tim3_pc6_init();
		TIM3_PWM_init( SystemCoreClock / 5000 ); // signal generator for async CW modulation - connect PC6 to PA10
		#endif
		break;
	case 'Q' :
		AAR_tx_enable ^= 1;
		break;
	// les strobes
	case 'Z' :
		read_strobe( CC1101_SRES );
		write_reg(CC1101_IOCFG0, CC1101_GDO_CRC_OK );	// eviter que GDO0 envoie une horloge 135.4kHz !
		break;
	case 'I' :
		read_strobe( CC1101_SIDLE );
		break;
	case 'C' :
		read_strobe( CC1101_SCAL );
		break;
	case 'R' :
		read_strobe( CC1101_SRX );
		break;
	case 'S' :
		read_strobe( CC1101_SFSTXON );
		break;
	case 'T' :
		read_strobe( CC1101_STX );
		break;

	default : CDC_printf("%c\n", ((c>=' ')?(c):('?')) );
	}
}

int CC1101::GFSK_radio_init()
{
// gpio_spi1_init();	// c'est fait
// SPI1_init();		// c'est fait
if	( ( read_reg( CC1101_SYNC1 ) != 0xD3 ) || ( read_reg( CC1101_SYNC0 ) != 0x91 ) )
	return 1;
#ifdef TURBO_38K
preset_P38Gplus();
#else
preset_P10Gplus();
#endif
read_strobe( CC1101_SIDLE );
tickdelay( 8000 );	// 8000 -> 1ms @ 8MHz
read_strobe( CC1101_SRX );
return 0;
}

// radio TX : DEPRECATED
int CC1101::tx_if_can( const unsigned char * tbuf, int len )
{
// checks
unsigned int f1 = read_reg( CC1101_FREQ1 );
unsigned int f2 = read_reg( CC1101_FREQ2 );
if	( ( f2 != 0x10 ) || ( f1 < 0xA9 ) || ( f1 > 0xB7 ) )
	return 1;	// min 433.164 MHz, max 434.687 MHz
if	( len > 61 ) return 4;
unsigned int rxbytes, txbytes;
rxbytes = read_status_reg( CC1101_RXBYTES );
txbytes = read_status_reg( CC1101_TXBYTES );
if	( txbytes ) return 2;
if	( rxbytes ) return 3;
// ici on devrait verifier CCA
// let's go
read_strobe( CC1101_SIDLE );
write_reg( 0x3F, len );
write_regs( 0x3F, (const unsigned char *)tbuf, len );
read_strobe( CC1101_STX );
return 0;
}

// radio TX, return val :
//	1: wrong frequ
//	2: TX FIFO not empty
//	3: RX FIFO not empty
//	4: message too big
int CC1101::tx_if_can( const unsigned char * tbuf )
{
// checks
unsigned int f1 = read_reg( CC1101_FREQ1 );
unsigned int f2 = read_reg( CC1101_FREQ2 );
if	( ( f2 != 0x10 ) || ( f1 < 0xA9 ) || ( f1 > 0xB7 ) )
	return 1;	// min 433.164 MHz, max 434.687 MHz
if	( tbuf[0] > 61 ) return 4;
unsigned int rxbytes, txbytes;
rxbytes = read_status_reg( CC1101_RXBYTES );
txbytes = read_status_reg( CC1101_TXBYTES );
if	( txbytes ) return 2;
if	( rxbytes ) return 3;
// ici on devrait verifier CCA
// let's go
read_strobe( CC1101_SIDLE );
write_regs( 0x3F, (const unsigned char *)tbuf, 1+tbuf[0] );
read_strobe( CC1101_STX );
return 0;
}

int CC1101::cw_radio_init()
{
// gpio_spi1_init();	// c'est fait
// SPI1_init();		// c'est fait
if	( ( read_reg( CC1101_SYNC1 ) != 0xD3 ) || ( read_reg( CC1101_SYNC0 ) != 0x91 ) )
	return 1;
read_strobe( CC1101_SIDLE );
preset_P10AF();
write_reg(CC1101_IOCFG0, 0x2E); // Hi Z, for safety when leaving async mode
set_modu( CC1101_AM );
unsigned char patable[] = { 0x34, 0x34, 0, 0, 0, 0, 0, 0 };	// level -10 dBm
set_patable( patable );
set_power( 0 );
read_strobe( CC1101_SIDLE );
tickdelay( 8000 );	// 8000 -> 1ms @ 8MHz
read_strobe( CC1101_STX );
return 0;
}

// format radio RX packet to CDC, binary + ascii (first byte is length)
void CC1101::formatba_rx_to_CDC( unsigned char * rxdata )
{
#ifdef USE_CDC
unsigned int len = rxdata[0];  // The packet length is defined excluding the length byte and the CRC16
CDC_printf("RX len %d {", len );
// payload hex display
for	( unsigned int i = 1; i < len+1; i++ )
	CDC_printf("%02X,", rxdata[i] );
CDC_printf("}=\"");
// payload as filtered text
for	( unsigned int i = 1; i < len+1; i++ )
	CDC_printf("%c", (char(rxdata[i])<' ')?('_'):(rxdata[i]) );
// diagnostics
int hrssi = ((int)((signed char)rxdata[len+1])) - (2*74);
unsigned char LQI = rxdata[len+2];
CDC_printf( "\" %d dBm, LQI=%u\n", hrssi/2, LQI & 0x7F );
#endif
}

// format radio RX packet to CDC, binary only (first byte is length)
void CC1101::formatb_rx_to_CDC( unsigned char * rxdata )
{
#ifdef USE_CDC
unsigned int len = rxdata[0];  // The packet length is defined excluding the length byte and the CRC16
CDC_printf("RX %d {", len );
// payload hex display
for	( unsigned int i = 1; i < len+1; i++ )
	CDC_printf("%02X,", rxdata[i] );
// diagnostics
int hrssi = ((int)((signed char)rxdata[len+1])) - (2*74);
unsigned char LQI = rxdata[len+2];
CDC_printf( "} %d dBm, LQI=%u\n", hrssi/2, LQI & 0x7F );
#endif
}


// extract received data from RX FIFO
// first byte is length of remaining contents (may be 0)
unsigned char * CC1101::extract_rx()
{
char rxbytes = read_status_reg( CC1101_RXBYTES );
if  ( rxbytes == 0 )
    return (unsigned char *)"";
return read_regs( 0x3F, rxbytes );
}
