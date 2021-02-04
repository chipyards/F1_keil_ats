   #include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_gpio.h"
#include "gpio.h"

// N.B. LL_GPIO_MODE_FLOATING <==> pas de pull
//      LL_GPIO_MODE_INPUT    <==> pull up ou down selon ODR


void gpio_init(void)
{
// nucleo LED = PA5
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL );
// nucleo bouton bleu = PC13
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOC );
}


// initialiser GPIO pour UART1
void gpio_uart1_init(void)
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
// pin PA9 = TX (FT)
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_MEDIUM );
// LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN );	// 5V pullup !!!
// pin PA10 = RX (FT)
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_FLOATING );
}

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

// initialiser GPIO pour UART3
void gpio_uart3_init(void)
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOB );
// pin PB10 = TX (FT)
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_MEDIUM );
// LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN );	// 5V pullup !!!
// pin PB11 = RX (FT)
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_FLOATING );
}

// initialiser GPIO pour SPI1 slave
// RA5 conflit avec LED verte carte Nucleo !!!!!
void gpio_spi1_slave_init(void)
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
// RA6 alternate push pull, 10MHz */		// SPI1 MISO
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_MEDIUM );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL );
// RA5 et RA7 input				// SPI1 SCK & MOSI
// RA5 conflit avec LED verte carte Nucleo !!!!!
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_FLOATING );
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_FLOATING );
// RA4 input pullup 					// SPI1 NSS
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT );
LL_GPIO_SetPinPull(       GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_UP );
}

// initialiser GPIO pour SPI2 slave
void gpio_spi2_slave_init(void)
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOB );
// RB14 alternate push pull, 10MHz */		// SPI2 MISO
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_MEDIUM );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_14, LL_GPIO_OUTPUT_PUSHPULL );
// RB13, RB15 input 				// SPI2 SCK & MOSI
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_FLOATING );
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_FLOATING );
// RB12 input w/ pullup				// SPI2 NSS
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_INPUT );
LL_GPIO_SetPinPull(       GPIOB, LL_GPIO_PIN_12, LL_GPIO_PULL_UP );
}
