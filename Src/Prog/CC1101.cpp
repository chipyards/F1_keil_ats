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
	return rxbuf+1;
	}
// burst write, prend l'adresse d'un array de bytes
void CC1101::write_regs( int start_adr, unsigned char * src, int cnt ) {
	txbuf[0] = 0x40 | ( start_adr & 0x3f );
	memcpy( txbuf+1, src, cnt );
	SPI1_multi_byte( txbuf, rxbuf, cnt+1 );
	}

// get 24 bits of synth frequ
unsigned int CC1101::get_synth_frequ()
{
unsigned int f;
/* methode sans burst
f  = read_reg( 0x0D ) << 16;
f |= read_reg( 0x0E ) << 8;
f |= read_reg( 0x0F );
*/
unsigned char * fbuf = read_regs( 0x0D, 3 );
f = ( fbuf[0] << 16 ) | ( fbuf[1] << 8 ) | ( fbuf[2] );
return f;
}

// set 24 bits of synth frequ
void CC1101::set_synth_frequ( unsigned int fu )
{
/* methode sans burst
write_reg( 0x0D, fu >> 16 );
write_reg( 0x0E, fu >> 8 );
write_reg( 0x0F, fu );
*/
unsigned char src[] = { (unsigned char)(fu>>16), (unsigned char)(fu>>8), (unsigned char)fu };
write_regs( 0x0D, src, 3 );
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
		CDC_printf("reg %02x : status %02x\n", adr, write_reg( adr, 0x2f ) );	// logic 0
		break;
	case 'k':
		adr = CC1101_IOCFG2;
		CDC_printf("reg %02x : status %02x\n", adr, write_reg( adr, 0x40 | 0x2f ) ); // logic 1
		break;
	case 'l':
		{
		unsigned int fu = get_synth_frequ();
		float ff = qfp_fdiv( qfp_fmul( 26.0f, (float)fu ), 65536.0f );
		CDC_printf("got freq synth = %06x = %6f\n", fu, ff );
		} break;
	case 'm':
		{
		float ff = 433.4f;
		unsigned int fu = (unsigned int)qfp_fadd( 0.5, qfp_fmul( 65536.0f, qfp_fdiv( ff, 26.0f ) ) );
		CDC_printf("set freq synth = %06x = %6f\n", fu, ff );
		set_synth_frequ( fu );
		} break;
	case 'n':
		{
		float ff = 434.4f;
		unsigned int fu = (unsigned int)qfp_fadd( 0.5, qfp_fmul( 65536.0f, qfp_fdiv( ff, 26.0f ) ) );
		CDC_printf("set freq synth = %06x = %6f\n", fu, ff );
		set_synth_frequ( fu );
		} break;
	default : CDC_printf("%c\n", ((c>=' ')?(c):('?')) );
	}
}
