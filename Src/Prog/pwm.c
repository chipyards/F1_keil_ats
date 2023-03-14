#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_tim.h"
#include "pwm.h"

void TIM3_PWM_init( unsigned int period )
{
// clock
LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM3 ); 
  
// count up
// LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
  
// prescaler
LL_TIM_SetPrescaler( TIM3, 0 );
  
// shadow system
LL_TIM_EnableARRPreload( TIM3 );
  
// frequency
LL_TIM_SetAutoReload( TIM3, period - 1 );
  
// PWM mode sur Ch 1
LL_TIM_OC_SetMode( TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
  
// output polarity
// LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  
// PW = 50%
LL_TIM_OC_SetCompareCH1( TIM3, period / 2 );
  
// Enable TIM3_CCR1 register preload
LL_TIM_OC_EnablePreload( TIM3, LL_TIM_CHANNEL_CH1 );

// enable channel
LL_TIM_CC_EnableChannel( TIM3, LL_TIM_CHANNEL_CH1 );

// forcer un demarrage propre!! sinon cela peut prendre longtemps...
// ou ne pas demarrer du tout, a cause des shadow registers
LL_TIM_GenerateEvent_UPDATE( TIM3 );

// Enable counter
LL_TIM_EnableCounter( TIM3 );

// Interrupt
NVIC_SetPriority( TIM3_IRQn, 2 );
//NVIC_EnableIRQ( TIM3_IRQn );
//LL_TIM_EnableIT_???( TIM3 );

}
