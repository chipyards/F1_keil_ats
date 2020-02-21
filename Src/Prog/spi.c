#include "options.h"

#ifdef USE_SPI2SL
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

// contexte global
SPItype SPI_2;
     
void SPI2_IRQHandler(void)
{
unsigned char d;
if	( LL_SPI_IsActiveFlag_RXNE( SPI2 ) )
	{		// Store received data, cela tient lieu d'acquittement pour l'interrupt
	d = LL_SPI_ReceiveData8( SPI2 );
	if	( SPI_2.rx_wi != ( SPI_2.rx_ri + QSPIBUF ) )	// prevention overrun (silencieuse)
		SPI_2.rx[ (SPI_2.rx_wi++) & SPI_IMASK ] = d;
	}
if	( LL_SPI_IsActiveFlag_TXE( SPI2 ) )
	{		// injecter donnee a emettre, cela tient lieu d'acquittement pour TXE
	if	( SPI_2.tx_ri == SPI_2.tx_wi )
		d = SPI_NO_DATA;				// signaler underrun
	else	d = SPI_2.tx[ (SPI_2.tx_ri++) & SPI_IMASK ];
	LL_SPI_TransmitData8( SPI2, d );
	}
}

// fonction generique, definie plus loin
static void spi_slave_init( SPI_TypeDef * SPIx, int IRQ_prio );

// attention : interrupt enable inside
void spi2_slave_init( int IRQ_prio )
{
SPI_2.rx_wi= 0;
SPI_2.rx_ri= 0;
SPI_2.tx_wi= 0;
SPI_2.tx_ri= 0;
spi_slave_init( SPI2, IRQ_prio );
}

// retourne 0 si ok, -1 si refus (prevention overrun)
int spi2_put8( unsigned int d )
{
if	( SPI_2.tx_wi == ( SPI_2.tx_ri + QSPIBUF ) )
	return -1;
else	SPI_2.tx[ (SPI_2.tx_wi++) & SPI_IMASK ] = d;
return 0;
}

// retourne un unsigned char, ou -1 si pas de data
int spi2_get8(void)
{
if	( SPI_2.rx_ri == SPI_2.rx_wi )
	return -1;						// signaler underrun
else	return 0xFF & SPI_2.rx[ (SPI_2.rx_ri++) & SPI_IMASK ];
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
static void spi_slave_init( SPI_TypeDef * SPIx, int IRQ_prio )
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

#endif
