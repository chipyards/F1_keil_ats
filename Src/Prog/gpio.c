#include "options.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_gpio.h"
#include "gpio.h"

// N.B. LL_GPIO_MODE_FLOATING <==> pas de pull
//      LL_GPIO_MODE_INPUT    <==> pull up ou down selon ODR


void gpio_init(void)
{
#ifdef NUCLEO
// Nucleo LED = PA5 act. hi
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL );
// blue button act. lo
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOC );
LL_GPIO_SetPinMode(       GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_FLOATING );
#else
// blue pill LED = PC13 act. lo
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOC );
LL_GPIO_SetPinMode(       GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL );
#endif
// Blue pill n'a pas de bouton bleu, alors utiliser PA12 qui a un pullup en dur
// on ajout pullup interne pour compat. sur Nucleo
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
LL_GPIO_SetOutputPin(     GPIOA, LL_GPIO_PIN_12 );
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_INPUT );

// profiling | signalisation sur PB12
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOB );
#ifdef PROF_PB12
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_HIGH );
#else	// N.B. les ecriture vont quand meme actionner les pull-up/down
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_INPUT );
#endif

#ifdef USE_ADC
// analog in sur PA0, ch. 0
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG );
#endif
}

#ifdef USE_PWM
// initialiser PWM out sur PA6
void gpio_timer3_init()
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
// pin PA6 = T3.1
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL );
}
#endif

#ifdef USE_NOKIA
void gpio_spi1_tr_r_init(void)	// SPI 1 remapped (TX only)
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOB );
// UWAGA #1 : il faut activer l'horloge AFIO pour faire du remap
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_AFIO );
// UWAGA #2 : il faut disabler le legacy JTAG car on emprunte 2 de ses pins
LL_GPIO_AF_Remap_SWJ_NOJTAG();
LL_GPIO_AF_EnableRemap_SPI1();
// SPI.SCK connected to PB3 (CN10.31 aka D3)
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
// SPI1.MOSI connected to PB5 (CN10.29 aka D4)
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
// SPI1.NSS (soft) connected to PB4 (CN10.27 aka D5)
LL_GPIO_SetOutputPin(     GPIOB, LL_GPIO_PIN_4 );	// act lo
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
}

void gpio_nokia_init(void)
{
gpio_spi1_tr_r_init();
// NOKIA DC connected to PA10 (CN10.33 aka D2)
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_MEDIUM);
// NOKIA RST connected to PB10 (CN10.25 aka D6)
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOB );
LL_GPIO_ResetOutputPin(   GPIOB, LL_GPIO_PIN_10 );	// mettre ce reset a zero le plus tot possible !
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_MEDIUM);
}
#endif

/* initialiser GPIO pour UART1 *
void gpio_uart1_init(void)
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
// pin PA9 = TX
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_MEDIUM );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL );
// pin PA10 = RX
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_FLOATING );
}
//*/

// initialiser GPIO pour UART2
void gpio_uart2_init(void)
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
// pin PA2 = TX
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_MEDIUM );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL );
// pin PA3 = RX
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_FLOATING );
}

#ifdef USE_UART3
/* initialiser GPIO pour UART3 */
void gpio_uart3_init(void)
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOB );
// pin PB10 = TX
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_MEDIUM );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL );
// pin PB11 = RX
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_FLOATING );
// supplement pour transceiver FM 433MHz
// Tx Cmd	PB9
LL_GPIO_ResetOutputPin(   GPIOB, LL_GPIO_PIN_9 );
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_MEDIUM);
// Rx Cmd	PB8
LL_GPIO_ResetOutputPin(   GPIOB, LL_GPIO_PIN_8 );
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_MEDIUM);
}

void Tx_cmd( int on )
{
if	( on )	LL_GPIO_SetOutputPin(   GPIOB, LL_GPIO_PIN_9 );
else		LL_GPIO_ResetOutputPin( GPIOB, LL_GPIO_PIN_9 );
}

void Rx_cmd( int on )
{
if	( on )	LL_GPIO_SetOutputPin(   GPIOB, LL_GPIO_PIN_8 );
else		LL_GPIO_ResetOutputPin( GPIOB, LL_GPIO_PIN_8 );
}
#endif

