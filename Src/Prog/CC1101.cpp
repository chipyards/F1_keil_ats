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
"IDLE", "RX", "TX", "FSTXON", "CALIB", "SETTLE", "RX_OVER", "TX_OVER"
};

// singleton
CC1101 CC;

// burst read, rend l'adresse d'un array de bytes
unsigned char * CC1101::read_regs( int start_adr, int cnt ) {
	txbuf[0] = 0xC0 | ( start_adr & 0x3f );
	SPI1_multi_byte( txbuf, rxbuf, cnt+1 );
	status = rxbuf[0];
	return rxbuf+1;
	}
// burst write, prend l'adresse d'un array de bytes
void CC1101::write_regs( int start_adr, unsigned char * src, int cnt ) {
	txbuf[0] = 0x40 | ( start_adr & 0x3f );
	memcpy( txbuf+1, src, cnt );
	SPI1_multi_byte( txbuf, rxbuf, cnt+1 );
	status = rxbuf[0];
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
unsigned char * fbuf = read_regs( 0x3E, 8 );
CDC_printf("PATABLE : %d -> ", get_power() );
for	( unsigned int i = 0; i < 8; i++ )
	CDC_printf(" %02x", fbuf[i] );
CDC_printf("\n");
}

void CC1101::quick_set()	// tester les float convs
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

}

void CC1101::quick_view()
{
unsigned int E, M, fu;
float ff;
CDC_printf("version %02x\n", read_status_reg( CC1101_VERSION ) );

fu = get_synth_frequ();
ff = synth_frequ_to_float( fu );
CDC_printf("freq synth %06x -> %6f MHz\n", fu, ff );

fu = get_IF();
ff = IF_to_float( fu );
CDC_printf("IF %02x -> %.2f kHz\n", fu, ff );

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
int adr;
switch	( c ) {
	// minuscules : observation
	case 'c' : compare_config( reset_regs );
		break;
	case 'q' : quick_view();
		break;
	case ' ' :
		read_strobe( CC1101_SNOP );
		CDC_printf("status FSM %s, FIFO %d\n", fsm_states[(status >> 4) & 7], status & 0x0F );
		break;
	// majuscules et chiffres : actions
	case 'J':
		adr = CC1101_IOCFG2;
		write_reg( adr, 0x2f );	// test LED : logic 0
		GDO0_LO();
		CDC_printf("wrote 0x2f to reg %02x, 0 to GDO0\n", adr  );
		break;
	case 'K':
		adr = CC1101_IOCFG2;
		write_reg( adr, 0x40 | 0x2f ); //  test LED : logic 1
		GDO0_HI();
		CDC_printf("wrote 0x6f to reg %02x, 1 to GDO0\n", adr  );
		break;
	case '3':
		{
		float ff = 433.4f;
		unsigned int fu = synth_frequ_from_float( ff );
		CDC_printf("set freq synth = %06x = %6f\n", fu, ff );
		set_synth_frequ( fu );
		} break;
	case '4':
		{
		float ff = 434.4f;
		unsigned int fu = synth_frequ_from_float( ff );
		CDC_printf("set freq synth = %06x = %6f\n", fu, ff );
		set_synth_frequ( fu );
		} break;
	case '8' :
		{
		float fK = 4.8f;
		unsigned int E, M;
		data_rate_from_float( &M, &E, fK );
		CDC_printf("set symbol rate %.5f kHz -> M=%d, E=%d\n", fK, M, E );
		set_data_rate( M, E );
		} break;
	case 'A' :		// FSK manuel, use JK
		// smarties();
		preset_P10AF();
		write_reg(CC1101_IOCFG0, 0x2E); // Hi Z, for safety when leaving async mode
		compare_config( reset_regs );
		LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING ); // cas ou on a connect PC6 a PA10
		break;
	case 'B' :		// apres A : FSK 5kHz - connect PC6 a PA10
		#ifdef USE_TIM3_PC6
		gpio_tim3_pc6_init();
		TIM3_PWM_init( SystemCoreClock / 5000 ); // signal generator for async CW modulation - connect PC6 a PA10
		#endif
		break;
	case 'U' :		// apres A : CW
		set_deviation( 0, 0 );
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
