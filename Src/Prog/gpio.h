#ifdef NUCLEO
#define LED_ON()	LL_GPIO_SetOutputPin(   GPIOA, LL_GPIO_PIN_5 )
#define LED_OFF()	LL_GPIO_ResetOutputPin( GPIOA, LL_GPIO_PIN_5 )
#else
#define LED_OFF()	LL_GPIO_SetOutputPin(   GPIOC, LL_GPIO_PIN_13 )
#define LED_ON()	LL_GPIO_ResetOutputPin( GPIOC, LL_GPIO_PIN_13 )
#endif

#define IS_PA12_SET()	LL_GPIO_IsInputPinSet(  GPIOA, LL_GPIO_PIN_12 )

#define PB12_PROFIL_1()	LL_GPIO_SetOutputPin(   GPIOB, LL_GPIO_PIN_12 )
#define PB12_PROFIL_0()	LL_GPIO_ResetOutputPin( GPIOB, LL_GPIO_PIN_12 )

void gpio_init(void);
void gpio_timer3_init(void);

void gpio_nokia_init(void);

// void gpio_uart1_init(void);
void gpio_uart2_init(void);
// void gpio_uart3_init(void);
