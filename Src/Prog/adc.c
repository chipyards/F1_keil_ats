/* code ADC 4 canaux derive de MRC_SCAN_13.zip (nucleo F103), porte pour LL
	Utilise les deux ADCs simultanement, pour mesure differentielle
	Convertit sequentiellement 2 canaux sur chaque ADC

*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_tim.h"
#include "options.h"
#include "sys.h"

#include "adc.h"

#ifdef PROF_PB12
#include "stm32f1xx_ll_gpio.h"
#include "gpio.h"
#endif

// shared global storage
volatile unsigned int fircnt = 0;		// dont 1 lsb pour selection du canal
volatile unsigned int adc1_res0 = 0;
volatile unsigned int adc1_res1 = 0;
volatile unsigned int adc2_res0 = 0;
volatile unsigned int adc2_res1 = 0;
// handshake :
//	mis a 2 par ce handler quand il a mis a jour les resus
//	mis a 0 par l'appli quand elle a assimile les resus
// le handler ne remet pas a jour les resus tant qu'ils n'ont pas ete assimiles
volatile int adc_res_ready = 0;


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
if	( fircnt == (TOTFIR-1) )	// avant derniere conversion du cycle
	{
	if	( adc_res_ready == 0 )
		{
		adc1_res0 = adc1_sum0;
		adc2_res0 = adc2_sum0;
		adc_res_ready = 1;
		}
	adc1_sum0 = 0;
	adc2_sum0 = 0;
	}
else if	( fircnt >= TOTFIR )	// TOTFIR = 2 * ordre du FIR de chaque canal
	{
	fircnt = 0;
	if	( adc_res_ready == 1 )
		{
		adc1_res1 = adc1_sum1;
		adc2_res1 = adc2_sum1;
		adc_res_ready = 2;
		}
	adc1_sum1 = 0;
	adc2_sum1 = 0;
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

// disable timer interrupts
void adc_timer_stop()
{
LL_TIM_DisableIT_UPDATE( TIM3 );
// tempo 1ms @ 72 MHz
tickdelay( 72 * 1000 );
}


// 2 ADCs
void adc_init()
{
// perif clock
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

// ADC clock prescaler (avail: 2, 4, 6, 8) max clock is 14 MHz
// common for both ADCs
#ifdef USE_PLL
// exemple : DIV_8 @ 72 MHz -> 9 MHz
LL_RCC_SetADCClockSource( LL_RCC_ADC_CLKSRC_PCLK2_DIV_8 );
#else
// exemple : DIV_2 @ 8 MHz -> 4 MHz
LL_RCC_SetADCClockSource( LL_RCC_ADC_CLKSRC_PCLK2_DIV_2 );
#endif

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
#define MY_SAMPLING_TIME LL_ADC_SAMPLINGTIME_239CYCLES_5
LL_ADC_SetChannelSamplingTime( ADC1, ADC1_CH0, MY_SAMPLING_TIME );
LL_ADC_SetChannelSamplingTime( ADC1, ADC1_CH1, MY_SAMPLING_TIME );
LL_ADC_SetChannelSamplingTime( ADC2, ADC2_CH0, MY_SAMPLING_TIME );
LL_ADC_SetChannelSamplingTime( ADC2, ADC2_CH1, MY_SAMPLING_TIME );

// numero de channel par defaut (sera mis a jour par interrupt)
LL_ADC_REG_SetSequencerRanks( ADC1, LL_ADC_REG_RANK_1, ADC1_CH0 );
LL_ADC_REG_SetSequencerRanks( ADC2, LL_ADC_REG_RANK_1, ADC2_CH0 );

// enable
LL_ADC_Enable( ADC1 );
LL_ADC_Enable( ADC2 );
}

// Run calibration on 2 ADCs
void adc_calib(void)
{
// en theorie il faut un delai de 2 ADC clock periods avant et apres la calib
// Certaines comments disent que l'ADC doit etre disabled avant de calibrer, d'autres disent le contraire !
  #ifdef PROF_PB12
  PB12_PROFIL_1();
  #endif
// tempo 10 us @ 72 MHz
tickdelay( 72 * 10 );
  #ifdef PROF_PB12
  PB12_PROFIL_0();
  #endif
// la calibration
LL_ADC_StartCalibration(ADC1);
LL_ADC_StartCalibration(ADC2);
// la boucle d'attente plante si cette fonction est appelee depuis une interrupt t.q. UART
while	( ( LL_ADC_IsCalibrationOnGoing(ADC1) != 0 ) || ( LL_ADC_IsCalibrationOnGoing(ADC1) != 0 ) )
	{ }
  #ifdef PROF_PB12
  PB12_PROFIL_1();
  #endif
// tempo min 2 cycles
tickdelay( 8 * 3 );	// 3 cycles ADC ne durent pas plus que 8 Tck puisque le diviseur max est 8
  #ifdef PROF_PB12
  PB12_PROFIL_0();
  #endif
}

// reset calibration on 2 ADCs N.B. ceci est pour tests seulrment, n'est pas supporte par LL !
void adc_uncalib(void)
{
ADC1->CR2 |= ADC_CR2_RSTCAL;
ADC2->CR2 |= ADC_CR2_RSTCAL;
}

// demarrer une conversion de test sur ADC1_CH0 et ADC2_CH0
// il faudra lire directement dans ADC1->DR, ADC2->DR
void adc_start_conv(void)
{
LL_ADC_REG_SetSequencerRanks( ADC1, LL_ADC_REG_RANK_1, ADC1_CH0 );
LL_ADC_REG_SetSequencerRanks( ADC2, LL_ADC_REG_RANK_1, ADC2_CH0 );
LL_ADC_ClearFlag_EOS(ADC1);
LL_ADC_ClearFlag_EOS(ADC2);
LL_ADC_REG_StartConversionSWStart(ADC1);
LL_ADC_REG_StartConversionSWStart(ADC2);
/* verif timing : bloquage pendant la conversion pour mesure duree conversion a l'oscillo
 * N.B. PLANTAGE si cette fonction est appelee depuis une interrupt,
 * EOS aka EOC ne passe pas a 1 ! Mais on observe son fonctionnement Ok depuis la main loop.
 *
while	( !LL_ADC_IsActiveFlag_EOS(ADC1) )
	{}
//*/
}

// calcul intensite instantanee selon shunt
//
//	i = iref * (shuntP-shuntN)/Vrefint
//
//	iref = Vref / ( gain * Rshunt )
//
// note 1 : CHANFIR n'a pas d'effet direct sur le resultat (mais sur le risque de debordement, oui),
// c'est grace a la division par la mesure de VREFINT
// mais la var. shunt_offset contient l'offset en LSBs multiplie par CHANFIR
// note 2 : risque de debordement sur numerateur :
//	- hyp : CHANFIR = 2000 prend 11 bits
//	  ==> adcx_resy prend 11 + 12 bits signes = 23 bits
//	- hyp : diff ne prend (quasi) pas plus car une des 2 mesures est calee sur 1/2 echelle
//	- hyp : iref = 1.2 / ( 220 * 0.845 mOhms ) = 6.45 A soit 6450 mA, prend 13 bits non signes
// alors le numerateur aurait 36 bits ! il faut en enlever 4
// on decide d'enlever 6 bits a diff, ajouter 2 a iref pour plus de finesse de calibration
// --> 17 + 15 = 32 bits pour le num
// on enleve 4 bits au denom pour un resultat en mA
int shunt_offset = 0;
int shunt_iref = 25313; // theor. 25800; // 4 * 6450

int adc_shunt_mA(void)
{
int num = adc2_res0 - adc1_res0 - shunt_offset;
num >>= 6;			// shuntP-shuntN avec correction offset
num *= shunt_iref;
int denom = adc1_res1 >> 4;	// mesure VREFINT
return num / denom;
}

// calcul intensite instantanee selon Hall
//
//	i = iref * ( Hall - hall_offset )/Vrefint
//
//	iref = Vref / Rhall )
//
// note 1 : CHANFIR n'a pas d'effet direct sur le resultat (mais sur le risque de debordement, oui),
// c'est grace a la division par la mesure de VREFINT
// mais la var. hall_offset contient l'offset en LSBs multiplie par CHANFIR
// note 2 : risque de debordement sur numerateur :
//	- hyp : CHANFIR = 2000 prend 11 bits
//	  ==> adcx_resy prend 11 + 12 bits non signes = 23 bits
//	- hyp : diff ne prend (quasi) pas plus car hall_offset est cale sur 1/2 echelle
//	- hyp : iref = 1.2 / ( 12.5 mOhms ) = 96 A soit 96000 mA, prend 17 bits non signes
// alors le numerateur aurait 40 bits ! il faut en enlever 8
// on decide d'enlever 6 bits a diff, et enlever 2 a iref
// --> 17 + 15 = 32 bits pour le num
// on enleve 8 bits au denom pour un resultat en mA
int hall_offset = 2029 * CHANFIR; // theor. 2047 * CHANFIR;
int hall_iref = 24000; // 96000 / 4

int adc_hall_mA(void)
{
int num = adc2_res1 - hall_offset;
num >>= 6;			// Hall - hall_offset
num *= hall_iref;
int denom = adc1_res1 >> 8;	// mesure VREFINT
return num / denom;
}

