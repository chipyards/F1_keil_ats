#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_spi.h"
#include "spi.h"

/* choix SPI1 ou SPI2 sur nucleo ?
	- SPI2 a une frequence d'horloge moitie, etant sur le bus APB1
	- SPI2 est en FT, SPI1 non
	- SPI2 est tout shunte par les 4 LEDs rouges de la carte keil ou compatible
	  les LEDs sont commandees par un bus driver '244 avec pulldown, ce qui est genant pour NSS aka CS0 
	- SPI1 a SCK shunte par la LED verte de la nucleo (RA5) via 510R
   conclusion :
	- proscrire carte keil ou compatible
	- utiliser SPI2 sur nucleo nue
   le present code est teste et a jour seulement pour SPI2
*/

// ring buffers pour SPI2
unsigned char spi2_rx[QSPIBUF];
unsigned int spi2_rx_wi= 0;	// index egaux <==> fifo vide
unsigned int spi2_rx_ri= 0;
static unsigned char spi2_tx[QSPIBUF];
static unsigned int spi2_tx_wi= 0;
static unsigned int spi2_tx_ri= 0;

void SPI2_IRQHandler(void)
{
unsigned char d;
if	( LL_SPI_IsActiveFlag_RXNE( SPI2 ) )
	{		// Store received data, cela tient lieu d'acquittement pour l'interrupt
	d = LL_SPI_ReceiveData8( SPI2 );
	if	( spi2_rx_wi != ( spi2_rx_ri + QSPIBUF ) )	// prevention overrun (silencieuse)
		spi2_rx[ (spi2_rx_wi++) & SPI_IMASK ] = d;
	}
if	( LL_SPI_IsActiveFlag_TXE( SPI2 ) )
	{		// injecter donnee a emettre, cela tient lieu d'acquittement pour TXE
	if	( spi2_tx_ri == spi2_tx_wi )
		d = SPI_NO_DATA;				// signaler underrun
	else	d = spi2_tx[ (spi2_tx_ri++) & SPI_IMASK ];
	LL_SPI_TransmitData8( SPI2, d );
	}
}

void spi2_fifo_init(void)
{
spi2_rx_wi= 0;
spi2_rx_ri= 0;
spi2_tx_wi= 0;
spi2_tx_ri= 0;
}

// retourne 0 si ok, -1 si refus (prevention overrun)
int spi2_put8( unsigned int d )
{
if	( spi2_tx_wi == ( spi2_tx_ri + QSPIBUF ) )
	return -1;
else	spi2_tx[ (spi2_tx_wi++) & SPI_IMASK ] = d;
return 0;
}

// retourne un unsigned char, ou -1 si pas de data
int spi2_get8(void)
{
if	( spi2_rx_ri == spi2_rx_wi )
	return -1;						// signaler underrun
else	return 0xFF & spi2_rx[ (spi2_rx_ri++) & SPI_IMASK ];
}

/* signalisation compatible RASPI master
    ____                                                       _____
CS0     |_____________________________________________________|
            _____ _____ _____ _____ _____ _____ _____ _____
MOSI_______X_____X_____X_____X_____X_____X_____X_____X_____X________
               __    __    __    __    __    __    __    __ 
SCLK__________|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |________

*/

// attention : interrupt enable inside
void spi_slave_init( SPI_TypeDef * SPIx, int IRQ_prio )
{
IRQn_Type IRQn;

if	( SPIx == SPI1 )
	{
	LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_SPI1 );
	IRQn = SPI1_IRQn;
	}
else if	( SPIx == SPI2 )
	{
	LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_SPI2 );
	IRQn = SPI2_IRQn;
	}
else	return;

LL_SPI_SetTransferDirection( SPIx, LL_SPI_FULL_DUPLEX );
LL_SPI_SetDataWidth(         SPIx, LL_SPI_DATAWIDTH_8BIT );
LL_SPI_SetClockPolarity(     SPIx, LL_SPI_POLARITY_LOW );	// aka CPOL // idle low
LL_SPI_SetClockPhase(        SPIx, LL_SPI_PHASE_1EDGE );	// aka CPHA // 1st edge samples incoming data
LL_SPI_SetNSSMode(           SPIx, LL_SPI_NSS_HARD_INPUT );
LL_SPI_SetTransferBitOrder(  SPIx, LL_SPI_MSB_FIRST );
LL_SPI_SetMode(              SPIx, LL_SPI_MODE_SLAVE );
LL_SPI_SetBaudRatePrescaler( SPIx, LL_SPI_BAUDRATEPRESCALER_DIV256 );	// ignored in slave mode
LL_SPI_Enable( SPIx );

// controleur d'interruption
NVIC_SetPriority( IRQn, IRQ_prio );  
NVIC_EnableIRQ( IRQn );

// interrupton RXNE
LL_SPI_EnableIT_RXNE( SPIx );
}

/* ring buffers pour SPI1
unsigned char spi1_rx[QSPIBUF];
unsigned int spi1_rx_wi= 0;
unsigned int spi1_rx_ri= 0;
unsigned char spi1_tx[QSPIBUF];
unsigned int spi1_tx_wi= 0;
unsigned int spi1_tx_ri= 0;

void SPI1_IRQHandler(void)
{
unsigned char d;
if	( LL_SPI_IsActiveFlag_RXNE( SPI1 ) )
	{		// Store received data, cela tient lieu d'acquittement pour l'interrupt
	d = LL_SPI_ReceiveData8( SPI1 );
	spi1_rx[ (spi1_rx_wi++) & SPI_IMASK ] = d;
	}
if	( LL_SPI_IsActiveFlag_TXE( SPI1 ) )
	{		// injecter donnee a emettre, cela tient lieu d'acquittement pour TXE
	d = spi1_tx[ (spi1_tx_ri++) & SPI_IMASK ];
	LL_SPI_TransmitData8( SPI1, d );
	}
}
*/
