
extern volatile int adc_raw;
extern volatile int acc1;
extern volatile int acc2;

void adc_init(void);
void adc_start_cal(void);
void adc_start_conv(void);
unsigned int adc_get(void);

// Tsamp = 1/22045 = 45.36 us
//	LOG	TAU	fc
//	16	2.97s	0.054 Hz
//	15	1.48s	0.107 Hz
//	14	0.74s	0.214 Hz
//	13	0.37s	0.428 Hz
//	12	0.186s	0.856 Hz
//	11	0.093s	1.71 Hz
//	10	0.046s	3.43 Hz
//	 9	0.023s	6.86 Hz
#define LOG_TAU1	14
#define LOG_TAU2	9

void opto_process(void);
void demod_process( int carrier );
