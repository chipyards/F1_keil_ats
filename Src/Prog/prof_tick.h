// systeme de profilage utilisant systick

#include "stm32f1xx.h"

extern volatile int tick0;
extern volatile int dtick;
extern volatile int max_dtick;

#define DTICK_VARS  volatile int tick0 = 0; volatile int dtick = 0; volatile int max_dtick = 0;

__STATIC_INLINE void DTICK_BEGIN() { tick0 = SysTick->VAL; }
__STATIC_INLINE void DTICK_END() { dtick = tick0 - SysTick->VAL; if ( dtick < 0 ) dtick += ( SysTick->LOAD + 1); if (dtick > max_dtick) max_dtick = dtick; }
