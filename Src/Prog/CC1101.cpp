#include "options.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include <string.h> // pour memcpy
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
/// get-set methods
///
void CC1101::get_patable( unsigned char *dest )
{
txbuf[0] = 0xC0 | 0x3E;
SPI1_multi_byte( txbuf, dest, 9 );
}

void CC1101::set_patable( const unsigned char *src )
{
write_regs( 0x3E, src, 8 );
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
unsigned char * fbuf = read_regs( 0x3E, 8 );
for	( unsigned int i = 0; i < 8; i++ )
	CDC_printf(" %02x", fbuf[i] );
CDC_printf("\n");
}

void CC1101::quick_set()	// valeurs bidon, pour tester les set methods, avec quick_view
{
unsigned int E, M, fu; float ff;
ff = 433.333f;
fu= synth_frequ_from_float( ff );
CDC_printf("set freq synth %.3f -> %06x\n", ff, fu );
set_synth_frequ( fu );

ff = 277.77f;
fu = IF_from_float( ff );
CDC_printf("set IF %.2f kHz -> %02x\n", ff, fu );
set_IF( fu );

ff = 22.22f;
data_rate_from_float( &M, &E, ff );
CDC_printf("set symbol rate %.2f kHz -> M=%d, E=%d\n", ff, M, E );
set_data_rate( M, E );

ff = 44.44f;
deviation_from_float( &M, &E, ff );
CDC_printf("set deviation %.2f kHz -> M=%d, E=%d\n", ff, M, E );
set_deviation( M, E );

ff = 333.33f;
bandwidth_from_float( &M, &E, ff );
CDC_printf("set bandwidth %.2f kHz -> M=%d, E=%d\n", ff, M, E );
set_bandwidth( M, E );

set_modu( CC1101_AM );
set_patable( full_patable );
set_power( 5 );

set_pkt_len(61);
set_PQT(5);
set_CRC_autoflush(1);
set_append_status(0);
set_adress_check(1);
set_whiten(0);
set_packet_format(1);
set_CRC(0);
set_packet_len_config(2);
set_adr( 53 );
set_no_dc_filt(1);
set_modu(0);
set_sync_mode(0);
set_preamble(4);
set_CCA(2);
set_RXOFF(2);
set_TXOFF(3);
set_autocal(1);
set_FOC_limit(3);
set_BS_limit(2);
set_FOC_BS_gate(1);

}

void CC1101::quick_view()
{
unsigned int E, M, fu;
float ff;
CDC_printf("version %02x\n", read_status_reg( CC1101_VERSION ) );

fu = get_synth_frequ();
ff = synth_frequ_to_float( fu );
CDC_printf("freq synth 0x%06x -> %6f MHz\n", fu, ff );

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

// quelques configs non documentees, intuitees par SmartRF
void CC1101::smarties()
{
write_reg( CC1101_FSCAL3, 0xE0  ); // FSCAL config 11 inst of 10, charge pump calibration still 1
write_reg( CC1101_FSCAL2, 0 );	  // automatic
write_reg( CC1101_FSCAL1, 0 );	  // automatic
write_reg( CC1101_FSCAL0, 0x1F );  // 1F instead of 0D, SmartRF said
write_reg( CC1101_TEST2, 0x81 );  //
write_reg( CC1101_TEST1, 0x35 );  //
write_reg( CC1101_TEST0, 0x09 );  //
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
	case 'd' : dump_config();
		break;
	case 'q' : quick_view();
		break;
	case ' ' :
		read_strobe( CC1101_SNOP );
		CDC_printf("FSM %s, RXFIFO %d\n", fsm_states[(STATUS >> 4) & 7], STATUS & 0x0F );
		write_strobe( CC1101_SNOP );
		CDC_printf("FSM %s, TXFIFO %d\n", fsm_states[(STATUS >> 4) & 7], STATUS & 0x0F );
		break;
	case '?' : {
		unsigned int rxbytes, txbytes;
		rxbytes = read_status_reg( CC1101_RXBYTES );
		txbytes = read_status_reg( CC1101_TXBYTES );
		CDC_printf("RX bytes %d, TX bytes %d\n", rxbytes, txbytes );
		if	( rxbytes )
			{
			unsigned char * zetxt = read_regs( 0x3F, rxbytes );
			CDC_printf( "RX len %d, [", zetxt[0] );
			unsigned int pos = zetxt[0] + 1;
			if	( pos >= sizeof( rxbuf ) )
				pos = ( sizeof( rxbuf ) - 1 );
			zetxt[pos] = 0;
			CDC_printf( (const char *)zetxt+1 );
			CDC_printf( "]\n" );
			}
		} break;
	// majuscules et chiffres : actions
	case '!': {	// put some text in tx fifo
		const char * txt = "C'est imposant pour ton petit corps";
		unsigned int len = strlen( txt );
		write_reg( 0x3F, len );
		write_regs( 0x3F, (const unsigned char *)txt, len );
		CDC_printf("put %d bytes in TX FIFO -> %d\n", len+1, read_status_reg( CC1101_TXBYTES ) );
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
		preset_P10G();
		set_pkt_len(61);
		set_PQT(0);
		set_append_status(1);
		set_adress_check(0);
		set_whiten(0);
		set_packet_format(0);
		set_CRC(0);
		set_packet_len_config(1);
		set_no_dc_filt(0);
		set_sync_mode(3);
		set_preamble(2);
		set_CCA(0);
		set_RXOFF(3);
		set_TXOFF(3);
		set_autocal(1);
		set_FOC_limit(0);
		set_BS_limit(0);
		write_reg(CC1101_IOCFG0, CC1101_GDO_RXFIFO );
		write_reg(CC1101_IOCFG2, CC1101_GDO_P_IN_PROC );
		quick_view();
		LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING ); // cas ou on a connect PC6 a PA10
		break;
	case 'O' : {		// OOK manuel, use JK
		preset_P10AF();
		write_reg(CC1101_IOCFG0, 0x2E); // Hi Z, for safety when leaving async mode
		set_modu( CC1101_AM );
		unsigned char patable[] = { 0x34, 0xC8, 0, 0, 0, 0, 0, 0 };	// levels -10 dBm and 7 dBm
		set_patable( patable );
		set_power( 1 );
		quick_view();
		LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING ); // cas ou on a connect PC6 a PA10
		} break;
	case 'A' : {		// ASK manuel, use JK
		preset_P10AF();
		write_reg(CC1101_IOCFG0, 0x2E); // Hi Z, for safety when leaving async mode
		set_modu( CC1101_AM );
		set_patable( full_patable );
		set_power( 6 );
		quick_view();
		LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING ); // cas ou on a connect PC6 a PA10
		} break;
	case 'B' :		// apres F, O ou A  : modulation 5kHz - please connect PC6 to PA10
		#ifdef USE_TIM3_PC6
		gpio_tim3_pc6_init();
		TIM3_PWM_init( SystemCoreClock / 5000 ); // signal generator for async CW modulation - connect PC6 to PA10
		#endif
		break;
	case 'U' :		// apres O : CW
		set_power( 0 );
		break;
	case 'Q' : quick_set();
		break;

	// les strobes
	case 'Z' :
		read_strobe( CC1101_SRES );
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
