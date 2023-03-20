#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_gpio.h"

#include "gpio.h"
#include "opto.h"
#include "audio.h"

volatile int adc_raw;
volatile int aticks = 0;
volatile int acc1 = 0;
volatile int acc2 = 0;
volatile int log_demod;

void adc_init(void)
{

// perif clock
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
//RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

// clock prescaler X8 --> 9 MHz (avail: 2, 4, 6, 8)
LL_RCC_SetADCClockSource( LL_RCC_ADC_CLKSRC_PCLK2_DIV_8 );
// RCC->CFGR |= RCC_CFGR_ADCPRE;	// 0x0000C000

// LL_ADC_SetResolution( ADC1, LL_ADC_RESOLUTION_12B );	// toujours 12 bits
LL_ADC_SetDataAlignment( ADC1, LL_ADC_DATA_ALIGN_RIGHT );

// software trigger
LL_ADC_REG_SetTriggerSource( ADC1, LL_ADC_REG_TRIG_SOFTWARE );
// ADC1->CR2 |= ADC_CR2_EXTSEL;	// 0x000E0000

// no continuous mode
LL_ADC_REG_SetContinuousMode( ADC1, LL_ADC_REG_CONV_SINGLE );
// ADC1->CR2 &= (~ADC_CR2_CONT);

// no scan (= 1 conversion)
LL_ADC_REG_SetSequencerLength( ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE );
//ADC1->SQR1 &= (~ADC_SQR1_L);	// with mask 0x00F00000 set 0 (lol)

// temp sensor + vref enable (ch 16 et 17)
// LL_ADC_SetCommonPathInternalCh( (ADC_Common_TypeDef *)ADC123_COMMON_BASE, LL_ADC_PATH_INTERNAL_VREFINT );
// ADC1->CR2 |= ADC_CR2_TSVREFE;

// sampling time for EACH channel individually, le max est 1.6us + 26.6uS
/*         @arg @ref LL_ADC_SAMPLINGTIME_1CYCLE_5
 *         @arg @ref LL_ADC_SAMPLINGTIME_7CYCLES_5
 *         @arg @ref LL_ADC_SAMPLINGTIME_13CYCLES_5
 *         @arg @ref LL_ADC_SAMPLINGTIME_28CYCLES_5
 *         @arg @ref LL_ADC_SAMPLINGTIME_41CYCLES_5
 *         @arg @ref LL_ADC_SAMPLINGTIME_55CYCLES_5
 *         @arg @ref LL_ADC_SAMPLINGTIME_71CYCLES_5
 *         @arg @ref LL_ADC_SAMPLINGTIME_239CYCLES_5 */
// attention les symboles LL_ADC_CHANNEL_nn ce n'est pas seulement le numero
LL_ADC_SetChannelSamplingTime( ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_239CYCLES_5 );
// ADC1->SMPR1 |= ( 7 << 21 );	// 0x00E00000,  shift = 3 * 7 car les LSB c'est canal 10

// numero de channel en 1ere position de la sequence
// ben c'est ici qu'on choisit le canal quoi !
LL_ADC_REG_SetSequencerRanks( ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0 );
// ADC1->SQR3 = 17;

// enable
LL_ADC_Enable(ADC1);
// ADC1->CR2 |= ADC_CR2_ADON;

}

void adc_start_cal(void)
{
LL_ADC_StartCalibration(ADC1);
while	( LL_ADC_IsCalibrationOnGoing(ADC1) != 0 )
	{ }
// ADC1->CR2 |= ADC_CR2_CAL;
}

void adc_start_conv(void)
{
LL_ADC_REG_StartConversionSWStart(ADC1);
// ADC1->CR2 |= ( ADC_CR2_SWSTART | ADC_CR2_EXTTRIG );

/* verif timing : bloquage pendant la conversion pour mesure duree conversion a l'oscillo
 * N.B. PLANTAGE si cette fonction est appelee depuis l'interrupt TIM3 update,
 * EOS ne passe pas a 1 ! Mais on observe son fonctionnement Ok depuia la main loop.
 *
while	( !LL_ADC_IsActiveFlag_EOS(ADC1) )
	{}
//*/
}

unsigned int adc_get(void)
{
// Retrieve ADC conversion data (clears EOC I presume)
return LL_ADC_REG_ReadConversionData12( ADC1 );
// return ( ADC1->DR & 0x0FFF );	// default = right align
}

void opto_process(void)
{
adc_raw = adc_get();
adc_start_conv();
aticks++;
// cycle 1002 Hz
if	( aticks >= 22 )
	{
	if	( etat.pos < 0 )	// inhiber laser pendant le son
		OPTO_DRIVE_HI();
	aticks = 0;
	demod_process(1);
	}
if	( aticks == 11 )
	{
	OPTO_DRIVE_LO();
	demod_process(-1);
	}
}

// calculer une approximation par exces du log2
int ceil_log2( int x )
{
if	( x < 0 )
	x = -x;
int cnt, last_one = 0;
for	( cnt = 0; cnt < 31; ++cnt )
	{
	if	( x & 1 )
		last_one = cnt;
	x >>= 1;
	}
return last_one;
}

void demod_process( int carrier )
{
// HPF 1 on raw_adc, dans le but d'enlever la composante DC
int hpf1 = adc_raw - ( acc1 >> LOG_TAU1 );
acc1 += hpf1;
// demodulation synchrone
int demod = hpf1 * carrier;
// LPF 2 sur demod
int hpf2 = demod - ( acc2 >> LOG_TAU2 );
acc2 += hpf2;
// acc2 est la sortie demodulee
if	( acc2 >= 0 )
	log_demod = 0;
else	log_demod = ceil_log2( -acc2 ) - ( LOG_TAU2 - 8 );
}
