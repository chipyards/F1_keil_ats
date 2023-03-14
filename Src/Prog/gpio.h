#define LED_OFF()	LL_GPIO_SetOutputPin(    GPIOC, LL_GPIO_PIN_13 )
#define LED_ON()	LL_GPIO_ResetOutputPin(  GPIOC, LL_GPIO_PIN_13 )

void gpio_init(void);
void gpio_timer3_init(void);

// void gpio_uart1_init(void);
void gpio_uart2_init(void);
// void gpio_uart3_init(void);
