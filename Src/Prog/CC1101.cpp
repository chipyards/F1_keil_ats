#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include <string.h> // pour memcpy
#include "qfplib-m3.h"
#include "gpio.h"
#include "sys.h"
#include "CDC.h"
#include "CC1101.h"



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
/// get-sets specialises
///

// get 24 bits of synth frequ
/* methode sans burst
unsigned int CC1101::get_synth_frequ()
{
unsigned int f;
f  = read_reg( 0x0D ) << 16;
f |= read_reg( 0x0E ) << 8;
f |= read_reg( 0x0F );
return f;
} */

// set 24 bits of synth frequ
/* methode sans burst
void CC1101::set_synth_frequ( unsigned int fu )
{
write_reg( 0x0D, fu >> 16 );
write_reg( 0x0E, fu >> 8 );
write_reg( 0x0F, fu );
*/



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
return float(M) / float(D);
}

void CC1101::data_rate_from_float( unsigned int *M, unsigned int *E, float fK  ) // fK en kHz
{
fK = qfp_fdiv( fK, 26000.0f );
float tmp = qfp_fmul( fK, float(1<<20) );
// l'exposant exact pour M = 0
tmp = qfp_fmul( qfp_fln( tmp ), 1.4426950408f );	// 1.443 = 1/ln(2)
// arrondissons-le par defaut (floor) pour que M soit > 0
*E = (unsigned int)tmp;
tmp = qfp_fadd( 0.5, qfp_fmul( fK, float(1<<(28-*E)) ) );
*M = (unsigned int)tmp - 256;
//if	( *M >= 256 )
//	{ *E += 1; *M = 0; }
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

// lire PATABLE
void CC1101::dump_patable()
{
unsigned char * fbuf = read_regs( 0x3E, 8 );
for	( unsigned int i = 0; i < 8; i++ )
	CDC_printf("PA %2d : %02x\n", i, fbuf[i] );
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
	case 'i':
		adr = CC1101_IOCFG2;
		CDC_printf("reg %02x : %02x\n", adr, read_reg( adr ) );
		adr = CC1101_IOCFG0;
		CDC_printf("reg %02x : %02x\n", adr, read_reg( adr ) );
		break;
	case 'j':
		adr = CC1101_IOCFG2;
		write_reg( adr, 0x2f );	// logic 0
		CDC_printf("wrote reg %02x\n", adr  );
		break;
	case 'k':
		adr = CC1101_IOCFG2;
		write_reg( adr, 0x40 | 0x2f ); // logic 1
		CDC_printf("wrote reg %02x\n", adr  );
		break;
	case 'l':
		{
		unsigned int fu = get_synth_frequ();
		float ff = synth_frequ_to_float( fu );
		CDC_printf("got freq synth = %06x = %6f\n", fu, ff );
		} break;
	case 'm':
		{
		float ff = 433.4f;
		unsigned int fu = synth_frequ_from_float( ff );
		CDC_printf("set freq synth = %06x = %6f\n", fu, ff );
		set_synth_frequ( fu );
		} break;
	case 'n':
		{
		float ff = 434.4f;
		unsigned int fu = synth_frequ_from_float( ff );
		CDC_printf("set freq synth = %06x = %6f\n", fu, ff );
		set_synth_frequ( fu );
		} break;
	case 't' : dump_config();
		break;
	case 'u' : dump_patable();
		break;
	case 'v' :
		{
		unsigned int E, M;
		get_data_rate( &M, &E );
		float fK = data_rate_to_float( M, E );
		CDC_printf("got data rate M=%d, E=%d -> %.5f kHz\n", M, E, fK );
		} break;
	case 'w' :
		{
		float fK = 4.8f;
		unsigned int E, M;
		data_rate_from_float( &M, &E, fK );
		CDC_printf("set data rate %.5f kHz -> M=%d, E=%d\n", fK, M, E, fK );
		set_data_rate( M, E );
		} break;
	case 'x' :
		write_strobe( CC1101_SFSTXON );
		break;
	case 'y' :
		write_strobe( CC1101_SIDLE );
		break;
	case 'z' :
		write_strobe( CC1101_SRES );
		break;
	case ' ' :
		CDC_printf("status %02x\n", status );
		break;
	default : CDC_printf("%c\n", ((c>=' ')?(c):('?')) );
	}
}
