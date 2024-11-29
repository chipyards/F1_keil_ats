#ifdef __cplusplus
extern "C" {
#endif

// systick avec interrupt
// ATTENTION SysTick_Handler() doit etre defini quelque part
void systick_init( unsigned int freq );

// temporisation base sur systick
// unites en periodes d'horloge du timer ( HCLK ou HCLK/8 )
// tickd doit etre inferieur a (LOAD+1)/2
void tickdelay( unsigned int tickd );

// creer un report dans le buffer
// N.B. pour avoir la correspondance numero <--> perif , voir IRQn_Type
// F103 : UARTS 1,2,3 : 37, 38, 39; TIM 2, 3, 4 : 28, 29, 30
void report_interrupts();

// System Clock Configuration selon options
void SystemClock_Config(void);

// disable systick interrupt
void systick_no_interrupt();

#ifdef __cplusplus
}
#endif
