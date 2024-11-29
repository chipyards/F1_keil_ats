#ifdef NUCLEO
#define LED_ON()	LL_GPIO_SetOutputPin(   GPIOA, LL_GPIO_PIN_5 )
#define LED_OFF()	LL_GPIO_ResetOutputPin( GPIOA, LL_GPIO_PIN_5 )
#define BLUE_PRESS()	(!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13 ))	// K2 sur carte Keil/compatible
#define K1_PRESS()	(!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0 ))		// K1 sur carte Keil/compatible
#else	// Blue Pill
#define LED_OFF()	LL_GPIO_SetOutputPin(   GPIOC, LL_GPIO_PIN_13 )
#define LED_ON()	LL_GPIO_ResetOutputPin( GPIOC, LL_GPIO_PIN_13 )
#endif

#define IS_PA12_SET()	LL_GPIO_IsInputPinSet(  GPIOA, LL_GPIO_PIN_12 )

#define PB12_PROFIL_1()	LL_GPIO_SetOutputPin(   GPIOB, LL_GPIO_PIN_12 )
#define PB12_PROFIL_0()	LL_GPIO_ResetOutputPin( GPIOB, LL_GPIO_PIN_12 )

#ifdef __cplusplus
extern "C" {
#endif

void gpio_init(void);
void gpio_timer3_init(void);

void gpio_uart1_init(void);
void gpio_uart2_init(void);
void gpio_uart3_init(void);
void Tx_cmd( int on );
void Rx_cmd( int on );

// NOKIA RST connected to PB10 (CN10.25 aka D6)
#define NOKIA_RST_LO()	LL_GPIO_ResetOutputPin( GPIOB, LL_GPIO_PIN_10 )
#define NOKIA_RST_HI()	LL_GPIO_SetOutputPin(   GPIOB, LL_GPIO_PIN_10 )
// NOKIA DC connected to PA10 (CN10.33 aka D2)
#define NOKIA_DC_LO()	LL_GPIO_ResetOutputPin( GPIOA, LL_GPIO_PIN_10 )
#define NOKIA_DC_HI()	LL_GPIO_SetOutputPin(   GPIOA, LL_GPIO_PIN_10 )
// SPI1.NSS (soft) connected to PB4 (CN10.27 aka D5)
#define NOKIA_CE_LO()	LL_GPIO_ResetOutputPin( GPIOB, LL_GPIO_PIN_4 )
#define NOKIA_CE_HI()	LL_GPIO_SetOutputPin(   GPIOB, LL_GPIO_PIN_4 )

void gpio_nokia_init(void);

#ifdef __cplusplus
} // extern "C"
#endif
