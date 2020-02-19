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
// pin PA9 = TX
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_MEDIUM );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL );
// pin PA10 = RX
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
// pin PB10 = TX
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_MEDIUM );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL );
// pin PB11 = RX
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_FLOATING );
}
