// systeme de profilage utilisant systick

#include "stm32f1xx.h"

extern volatile int tick0;
extern volatile int dtick;

#define DTICK_VARS  volatile int tick0 = 0; volatile int dtick = 0;

__STATIC_INLINE void DTICK_BEGIN() { tick0 = SysTick->VAL; }
__STATIC_INLINE void DTICK_END() { dtick = tick0 - SysTick->VAL; if ( dtick < 0 ) dtick += (SystemCoreClock / 100); }
