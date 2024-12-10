#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include "gpio.h"
#include "sys.h"
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
	LL_SPI_SetBaudRatePrescaler( SPI1, LL_SPI_BAUDRATEPRESCALER_DIV4 );	// 2 MHz @ 8 MHz
else	LL_SPI_SetBaudRatePrescaler( SPI1, LL_SPI_BAUDRATEPRESCALER_DIV32 );	// 2.25 MHz @ 72 MHz
LL_SPI_Enable(SPI1);
}

// ecrire et lire cnt bytes en une transaction
void SPI1_multi_byte( unsigned char * txbuf, unsigned char * rxbuf, int cnt )
{
NSS1_LO();
tickdelay(8);
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
tickdelay(8);
NSS1_HI();
tickdelay(8);
}
