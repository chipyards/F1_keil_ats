#define LED_ON()	LL_GPIO_SetOutputPin(    GPIOA, LL_GPIO_PIN_5 )
#define LED_OFF()	LL_GPIO_ResetOutputPin(  GPIOA, LL_GPIO_PIN_5 )
#define BLUE_BUTTON()	(!LL_GPIO_IsInputPinSet( GPIOC, LL_GPIO_PIN_13 ))

void gpio_init(void);
void gpio_uart1_init(void);
void gpio_uart2_init(void);
void gpio_uart3_init(void);
