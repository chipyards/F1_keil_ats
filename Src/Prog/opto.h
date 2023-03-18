
extern volatile unsigned int adc_raw;

void adc_init(void);
void adc_start_cal(void);
void adc_start_conv(void);
unsigned int adc_get(void);
void opto_process(void);
