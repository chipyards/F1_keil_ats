/* code ADC 4 canaux derive de MRC_SCAN_13.zip (nucleo F103), porte pour LL
	Utilise les deux ADCs simultanement, pour mesure differentielle
	Convertit sequentiellement 2 canaux sur chaque ADC

*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_tim.h"

#include "adc.h"

// shared global storage
volatile unsigned int fircnt = 0;		// dont 1 lsb pour selection du canal
volatile unsigned int adc1_res0 = 0;
volatile unsigned int adc1_res1 = 0;
volatile unsigned int adc2_res0 = 0;
volatile unsigned int adc2_res1 = 0;
volatile int adc_res_ready = 0;		// handshake

// interrupt handler for TIM3
void TIM3_IRQHandler(void)
{
static unsigned int adc1_sum0 = 0;
static unsigned int adc1_sum1 = 0;
static unsigned int adc2_sum0 = 0;
static unsigned int adc2_sum1 = 0;

LL_TIM_ClearFlag_UPDATE( TIM3 );

// recuperer resultat derniere conversion et changer de canal
if	( ( fircnt & 1 ) == 0 )
	{
	adc1_sum0 += LL_ADC_REG_ReadConversionData12( ADC1 );
	adc2_sum0 += LL_ADC_REG_ReadConversionData12( ADC2 );
	// numero de channel en 1ere position de la sequence
	// ben c'est ici qu'on choisit le canal c'est tout !
	LL_ADC_REG_SetSequencerRanks( ADC1, LL_ADC_REG_RANK_1, ADC1_CH1 );
	LL_ADC_REG_SetSequencerRanks( ADC2, LL_ADC_REG_RANK_1, ADC2_CH1 );
	}
else	{
	adc1_sum1 += LL_ADC_REG_ReadConversionData12( ADC1 );
	adc2_sum1 += LL_ADC_REG_ReadConversionData12( ADC2 );
	LL_ADC_REG_SetSequencerRanks( ADC1, LL_ADC_REG_RANK_1, ADC1_CH0 );
	LL_ADC_REG_SetSequencerRanks( ADC2, LL_ADC_REG_RANK_1, ADC2_CH0 );
	}
// compter
++fircnt;
if	( fircnt >= TOTFIR )	// TOTFIR = 2 * ordre du FIR de chaque canal
	{
	fircnt = 0;
	adc1_res1 = adc1_sum1;
	adc2_res1 = adc2_sum1;
	adc_res_ready = 1;
	adc1_sum1 = 0;
	adc2_sum1 = 0;
	}
else if ( fircnt == (TOTFIR-1) )
	{
	adc1_res0 = adc1_sum0;
	adc2_res0 = adc2_sum0;
	adc1_sum0 = 0;
	adc2_sum0 = 0;
	}
// demarrer nouvelle conversions
LL_ADC_REG_StartConversionSWStart( ADC1 );
LL_ADC_REG_StartConversionSWStart( ADC2 );
/* verif timing : bloquage pendant la conversion pour mesure duree conversion a l'oscillo
 * N.B. PLANTAGE si cette fonction est appelee depuis l'interrupt TIM3 update,
 * EOS ne passe pas a 1 ! Mais on observe son fonctionnement Ok depuis la main loop.
 *
while	( !LL_ADC_IsActiveFlag_EOS(ADC1) )
	{}
 */
}


// configurer le timer TIM3 en timebase (pour interrupts seulement)
void adc_timer_init( unsigned int period )
{
// clock
LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM3 ); 
  
// count up
LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
  
// prescaler
LL_TIM_SetPrescaler( TIM3, 0 );
  
// shadow system
LL_TIM_EnableARRPreload( TIM3 );
  
// frequency
LL_TIM_SetAutoReload( TIM3, period - 1 );
// forcer un demarrage propre!! sinon cela peut prendre longtemps...
// ou ne pas demarrer du tout, a cause des shadow registers
LL_TIM_GenerateEvent_UPDATE( TIM3 );

// Enable counter
LL_TIM_EnableCounter( TIM3 );

// Interrupt
NVIC_SetPriority( TIM3_IRQn, 2 );
NVIC_EnableIRQ( TIM3_IRQn );

LL_TIM_EnableIT_UPDATE( TIM3 );
}

// 2 ADCs
void adc_init()
{
// perif clock
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

// ADC clock prescaler (avail: 2, 4, 6, 8) max clock is 14 MHz
// common for both ADCs
// exemple : DIV_8 @ 72 MHz -> 9 MHz
LL_RCC_SetADCClockSource( LL_RCC_ADC_CLKSRC_PCLK2_DIV_8 );

// LL_ADC_SetResolution( ADC1, LL_ADC_RESOLUTION_12B );	// toujours 12 bits
LL_ADC_SetDataAlignment( ADC1, LL_ADC_DATA_ALIGN_RIGHT );
LL_ADC_SetDataAlignment( ADC2, LL_ADC_DATA_ALIGN_RIGHT );

// software trigger
LL_ADC_REG_SetTriggerSource( ADC1, LL_ADC_REG_TRIG_SOFTWARE );
LL_ADC_REG_SetTriggerSource( ADC2, LL_ADC_REG_TRIG_SOFTWARE );

// no continuous mode
LL_ADC_REG_SetContinuousMode( ADC1, LL_ADC_REG_CONV_SINGLE );
LL_ADC_REG_SetContinuousMode( ADC2, LL_ADC_REG_CONV_SINGLE );

// no scan (= 1 conversion)
LL_ADC_REG_SetSequencerLength( ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE );
LL_ADC_REG_SetSequencerLength( ADC2, LL_ADC_REG_SEQ_SCAN_DISABLE );

// temp sensor + vref enable (ch 16 et 17) (le bit TSVREFE)
LL_ADC_SetCommonPathInternalCh( __LL_ADC_COMMON_INSTANCE(), LL_ADC_PATH_INTERNAL_VREFINT );

// sampling time for EACH channel individually, le temps total de conversion est (sampling + 12.5) cycles
/*         LL_ADC_SAMPLINGTIME_1CYCLE_5		 14 cy
 *         LL_ADC_SAMPLINGTIME_7CYCLES_5	 20 cy
 *         LL_ADC_SAMPLINGTIME_13CYCLES_5	 26 cy
 *         LL_ADC_SAMPLINGTIME_28CYCLES_5	 41 cy
 *         LL_ADC_SAMPLINGTIME_41CYCLES_5	 54 cy
 *         LL_ADC_SAMPLINGTIME_55CYCLES_5	 68 cy
 *         LL_ADC_SAMPLINGTIME_71CYCLES_5	 84 cy
 *         LL_ADC_SAMPLINGTIME_239CYCLES_5 	252 cy */
// attention les symboles LL_ADC_CHANNEL_nn ce n'est pas seulement le numero
LL_ADC_SetChannelSamplingTime( ADC1, ADC1_CH0, LL_ADC_SAMPLINGTIME_41CYCLES_5 );
LL_ADC_SetChannelSamplingTime( ADC1, ADC1_CH1, LL_ADC_SAMPLINGTIME_41CYCLES_5 );
LL_ADC_SetChannelSamplingTime( ADC2, ADC2_CH0, LL_ADC_SAMPLINGTIME_41CYCLES_5 );
LL_ADC_SetChannelSamplingTime( ADC2, ADC2_CH1, LL_ADC_SAMPLINGTIME_41CYCLES_5 );

// numero de channel par defaut (sera mis a jour par interrupt)
LL_ADC_REG_SetSequencerRanks( ADC1, LL_ADC_REG_RANK_1, ADC1_CH0 );
LL_ADC_REG_SetSequencerRanks( ADC2, LL_ADC_REG_RANK_1, ADC2_CH0 );

// enable
LL_ADC_Enable( ADC1 );
LL_ADC_Enable( ADC2 );

/*
// reset calibration register 
ADC_ResetCalibration( ADC1 );
ADC_ResetCalibration( ADC2 );
// Check the end of calibration reset
while ( ADC_GetResetCalibrationStatus(ADC1) ) {}
while ( ADC_GetResetCalibrationStatus(ADC2) ) {}
// Start calibration
ADC_StartCalibration(ADC1);
ADC_StartCalibration(ADC2);
// Check the end of ADC calibration
while ( ADC_GetCalibrationStatus(ADC1) ) {}
while ( ADC_GetCalibrationStatus(ADC2) ) {}
*/
}

