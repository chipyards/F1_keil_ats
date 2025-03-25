#include "options.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_gpio.h"
#include "sys.h"
#include "gpio.h"

// N.B. LL_GPIO_MODE_FLOATING <==> pas de pull
//      LL_GPIO_MODE_INPUT    <==> pull up ou down selon ODR

/* N.B. l'option USE_LCD2x16 configure les pins suivantes, en bare metal
   - DB4 = PC3
   - DB5 = PC2
   - DB6 = PC1
   - DB7 = PC0
   - E   = PC10
   - RW  = PC11
   - RS  = PC12
*/

void gpio_init(void)
{
#ifdef NUCLEO
// Nucleo LED = PA5 act. hi	!!! ecrase par SPI SCK
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
// 4.7k @ 5V (prevu pour socket USB)
// on ajoute pullup interne pour compat. sur Nucleo
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
LL_GPIO_SetOutputPin(     GPIOA, LL_GPIO_PIN_12 );
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_INPUT );
// sur Blue Pill on peut aussi utiliser jumper A2-A3 voir plus loin
// profiling | signalisation sur PB12
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOB );
#ifdef PROF_PB12
LL_GPIO_ResetOutputPin(   GPIOB, LL_GPIO_PIN_12 );
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_HIGH );

#else	// N.B. les ecriture vont quand meme actionner les pull-up/down
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_INPUT );
#endif
} // gpio_init(void)

#ifdef USE_CC1101
void gpio_spi1_init(void)	// SPI 1
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOB );
// SPI.SCK connected to PA5 (D13)
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
// SPI1.MISO connected to PA6 (D12)
LL_GPIO_SetOutputPin(     GPIOA, LL_GPIO_PIN_6 );	// pullup
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT );
// SPI1.MOSI connected to PA7 (D11)
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
// SPI1.NSS (soft) connected to PB6 (D10)
LL_GPIO_SetOutputPin(     GPIOB, LL_GPIO_PIN_6 );	// act lo
LL_GPIO_SetPinMode(       GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinOutputType( GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL );
LL_GPIO_SetPinSpeed(      GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
// GDO0 connected to PA10 (D2)
LL_GPIO_ResetOutputPin(   GPIOA, LL_GPIO_PIN_10 );	// pulldown
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT );
}
#endif

// timer pour modulation du CC1101 a freqience constante
#ifdef USE_TIM3_PC6
void gpio_tim3_pc6_init(void)
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOC );
// UWAGA : il faut activer l'horloge AFIO pour faire du remap
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_AFIO );
// Full remap     (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9)
LL_GPIO_AF_EnableRemap_TIM3();
// pin PC6 = T3.1
LL_GPIO_SetPinMode(       GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE );
LL_GPIO_SetPinSpeed(      GPIOC, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH );
LL_GPIO_SetPinOutputType( GPIOC, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL );
}
#endif

/* initialiser GPIO pour UART1 */
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

// test jumper A2-A3, to use before gpio_uart2_init() (replaced by blue button on nucleo)
int gpio_test_jmpA23(void)
{
#ifdef NUCLEO
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOC );
LL_GPIO_SetPinMode(       GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_FLOATING );
return BLUE_PRESS();
#else
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_GPIOA );
// pin PA2 = TX
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT );
LL_GPIO_SetPinSpeed(      GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_MEDIUM );
LL_GPIO_SetPinOutputType( GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL );
// pin PA3 = RX
LL_GPIO_SetPinMode(       GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT );
// test 0
LL_GPIO_ResetOutputPin(   GPIOA, LL_GPIO_PIN_2 );	// down
LL_GPIO_SetOutputPin(     GPIOA, LL_GPIO_PIN_3 );	// pull up
tickdelay( 800 );	// HCLK units, 800 -> 0.1ms @ 8MHz
if	( LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_3 ) )
	return 0;	// already failed !
// test 1
LL_GPIO_SetOutputPin(     GPIOA, LL_GPIO_PIN_2 );	// up
LL_GPIO_ResetOutputPin(   GPIOA, LL_GPIO_PIN_3 );	// pull down
tickdelay( 800 );	// HCLK units, 800 -> 0.1ms
if	( !LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_3 ) )
	return 0;	// now failed !
return 1; // jumper present
#endif
}
