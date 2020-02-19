
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
// #if defined(USE_FULL_ASSERT)
// #include "stm32_assert.h"
// #endif /* USE_FULL_ASSERT */

#define LED2_PIN		LL_GPIO_PIN_5
#define LED2_GPIO_PORT		GPIOA
#define LED2_GPIO_CLK_ENABLE()	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA)
#define BLUE_GPIO_CLK_ENABLE()	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC)
#define LED_ON()		LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN)
#define LED_OFF()		LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN)
#define BLUE_BUTTON()	(!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13))

void     SystemClock_Config(void);
void     Configure_GPIO(void);

// HSE_EXT est pour utiliser une source d'horloge 8MHz externe
// si HSE_EXT : MCO de la sonde ST-LINK    8MHz -> PLL -> 72 MHz
// sinon      : oscillateur RC interne HSI 8MHz -> PLL -> 64 Mhz
// HSE_EXT ne marche pas sur une nucleo coupee ni sur Blue Pill

//#define HSE_EXT

unsigned int cnt100Hz = 0;

// systick interrupt handler
void SysTick_Handler()
{
++cnt100Hz;
switch	(cnt100Hz % 100)
	{
	case 0 :
	//case 10 :
		LED_ON();
		break;
	case 5 :
	//case 15 :
		LED_OFF();
		break;
	}
}


int main(void)
{
  /* Configure the system clock to 72 MHz */
  SystemClock_Config();
// config LED
  Configure_GPIO();
// config systick @ 100Hz

  // periode
  SysTick->LOAD  = (SystemCoreClock / 100) - 1;
  // priorite
  NVIC_SetPriority( SysTick_IRQn, 7 );
  // init counter
  SysTick->VAL = 0;
  // prescale (0 ===> %8)
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
  // enable timer, enable interrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

 while (1)
 	{
	if	( BLUE_BUTTON() )
		{//LED_ON();
		}
	else
		{
		SCB->SCR = 0;				// avoid deep sleep
		PWR->CR &= ~(PWR_CR_PDDS|PWR_CR_LPDS);	// avoid power down
		__WFI();				// Wait for Interrupt
		}
 	}
}

void Configure_GPIO(void)
{
  LED2_GPIO_CLK_ENABLE();
  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  BLUE_GPIO_CLK_ENABLE();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000 or 64000000
  *            HCLK(Hz)                       = 72000000 or 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

#ifdef HSE_EXT
/* Enable HSE oscillator or bypass */
LL_RCC_HSE_EnableBypass();	// pas de quartz ==> MCO du ST-Link
LL_RCC_HSE_Enable();
while(LL_RCC_HSE_IsReady() != 1)
  {  };
#else
LL_RCC_HSI_Enable();
while(LL_RCC_HSI_IsReady() != 1)
  {  };
#endif

/* Main PLL configuration and activation */
#ifdef HSE_EXT
LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
#else
// HSI est obligatoirement %2, donc avec MUL_16 qui est le max on a 64 MHz
LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
#endif

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {  };

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* Update SystemCoreClock variable */
  SystemCoreClockUpdate();
}
