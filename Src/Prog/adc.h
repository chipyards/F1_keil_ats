#ifdef __cplusplus
extern "C" {
#endif

// attention il faut un symbole LL_ADC_CHANNEL_xx, pas un simple numero
// attention les pins sont communes aux 2 ADCs, mais Temp et Vrefint sont seulement pour ADC1
#define ADC1_CH0	LL_ADC_CHANNEL_0	// PA0 shunt N
#define ADC2_CH0	LL_ADC_CHANNEL_1	// PA1 shunt P
#define ADC1_CH1	LL_ADC_CHANNEL_17	// Vrefint (1.20V +- 4%)
#define ADC2_CH1	LL_ADC_CHANNEL_8	// PB0 Hall

#define CHANFIR		2000		// ordre du FIR de chaque canal, determine la periodicite des resultats
#define TOTFIR		(2*CHANFIR)	// cycle complet
#define ADCFREQ		2000		// 2 kHz ==> 1 ksamp/s pour chaque canal avant FIR

// shared global storage
extern volatile unsigned int fircnt;	// dont 1 lsb pour le canal
extern volatile unsigned int adc1_res0;	// shunt N
extern volatile unsigned int adc1_res1;	// Vrefint
extern volatile unsigned int adc2_res0;	// shunt P
extern volatile unsigned int adc2_res1;	// Hall
extern volatile int adc_res_ready;	// handshake

extern int shunt_offset;
extern int shunt_iref;
extern int hall_offset;
extern int hall_iref;

// configurer le timer TIM3 en timebase (pour interrupts seulement)
void adc_timer_init( unsigned int period );

// disable timer interrupts
void adc_timer_stop(void);

// 2 ADCs
void adc_init(void);

// Run calibration on 2 ADCs
void adc_calib(void);

// reset calibration on 2 ADCs N.B. ceci n'est pas supporte par LL !
void adc_uncalib(void);

// demarrer une conversion de test sur ADC1_CH0 et ADC2_CH0
void adc_start_conv(void);

// fonctions pour extraire des resultats des adcx_resy
int adc_shunt_mA(void);
int adc_hall_mA(void);

#ifdef __cplusplus
}
#endif
