#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "options.h"
#include "sys.h"
#include "CDC.h"	// pour snprintf

// systick avec interrupt
// ATTENTION SysTick_Handler() doit etre defini quelque part
void systick_init( unsigned int freq )
{
// periode
SysTick->LOAD  = (SystemCoreClock / freq) - 1;
// init counter
SysTick->VAL = 0;
// prescale (0 ===> %8)
SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;	// no prescale

// priorite
NVIC_SetPriority( SysTick_IRQn, 7 );
// enable timer, enable interrupt
SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

// disable systick interrupt
void systick_no_interrupt()
{
SysTick->CTRL &= (~SysTick_CTRL_TICKINT_Msk);
}

// temporisation base sur systick
// unites en periodes d'horloge du timer ( HCLK ou HCLK/8 )
// tickd doit etre inferieur a (LOAD+1)/2
void tickdelay( unsigned int tickd )
{
int tper = SysTick->LOAD + 1;
int nextVAL, diff;
nextVAL = SysTick->VAL - tickd;
if	( nextVAL < 0 )
	nextVAL += tper;
do	{				// diff c'est le temps restant a attendre
	diff = SysTick->VAL - nextVAL;
	if	( diff <= -(tper/2) )	// on maintient diff entre -(tper/2) et (tper/2)
		diff = 1;
	else if ( diff > (tper/2) )
		break;
	} while ( diff > 0 );
}

// creer un report
// N.B. pour avoir la correspondance numero <--> perif , voir IRQn_Type
// F103 : UARTS 1,2,3 : 37, 38, 39; TIM 2, 3, 4 : 28, 29, 30
void report_interrupts()
{
int i, p;
p = __NVIC_GetPriorityGrouping();
CDC_printf("P.G. %d\n", p );
// special systick (#-1)
i = -1;
if	(  SysTick->CTRL & SysTick_CTRL_TICKINT_Msk )
	{
	p = __NVIC_GetPriority((IRQn_Type)i);
	CDC_printf("i #%2d, p %d\n", i, p );
	}
// tous les autres
for	( i = 0; i <=  97; ++i )
	{
	if	( __NVIC_GetEnableIRQ((IRQn_Type)i) )
		{
		p = __NVIC_GetPriority((IRQn_Type)i);
		CDC_printf("i #%2d, p %d\n", i, p );
		}
	}
}

// System Clock Configuration selon options
void SystemClock_Config(void)
{

#ifdef HSE_EXT
#define HSE
#endif

LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

#ifdef HSE
/* Enable HSE oscillator or bypass */
#ifdef HSE_EXT
LL_RCC_HSE_EnableBypass();	// pas de quartz ==> MCO du ST-Link
#endif
LL_RCC_HSE_Enable();
while(LL_RCC_HSE_IsReady() != 1)
  { }
#else
LL_RCC_HSI_Enable();
while(LL_RCC_HSI_IsReady() != 1)
  { }
#endif

#ifdef USE_PLL
  /* Set FLASH latency : 2 for HCLK > 48 MHz  */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  /* Main PLL configuration and activation */
  #ifdef HSE
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  #else
  // HSI est obligatoirement %2, donc avec MUL_16 qui est le max on a 64 MHz
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  #endif

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
    { }

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    { }
  /* Set APB1 prescaler : max 36 MHz */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
#else
  #ifdef HSE
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE)
    { }
  #endif
  /* Set FLASH latency : 0 for HCLK <= 24 MHz */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  /* Set APB1 prescaler : max 36 MHz */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
#endif
  /* Set APB12 prescaler : max 72 MHz */
LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Update SystemCoreClock variable */
  SystemCoreClockUpdate();
}
