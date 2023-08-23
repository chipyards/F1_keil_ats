
// profilage
// #define PROF_PB12
// #define PROF_PB12_EOS

// osc. modes :
//		HSI		HSE		HSE_EXT
// fmin		 8MHz		 8MHz		 8MHz
// fmax PLL	64MHz		72MHz		72MHz
// nucleo	Y		N		Y
// nucleo cut	Y		N		N
// blue pill	Y		Y		N


// choix du main
// #define MAIN_COULOMB
#define MAIN_TERMINAL

// presets
#ifdef MAIN_COULOMB

// nous sommes dans la branche blue_pill, c'est blue pill par defaut
// #define NUCLEO
// #define USE_PLL	// 64 MHz (HSI) ou 72 MHZ (HSE, HSE_EXT)

// SLEEP necessite connexion reset du ST-Link, sinon brick !
#define GREEN_CPU

// modules optionnels
// #define USE_CDC
#define USE_ADC_4CH	// uses TIM3
// #define USE_PWM	// uses TIM3, retired for the moment
// #define USE_NOKIA	// implies SPI1 remap or AF 5
// #define USE_FLASHY	// eeprom zone @ (0x08020000-0x400)
#define USE_UART3_FM
// #define USE_LCD2x16

#endif

#ifdef MAIN_TERMINAL

// nous sommes dans la branche blue_pill, c'est blue pill par defaut
#define NUCLEO
#define USE_PLL	// 64 MHz (HSI) ou 72 MHZ (HSE, HSE_EXT)

// SLEEP necessite connexion reset du ST-Link, sinon brick !
// #define GREEN_CPU

// modules optionnels
#define USE_CDC
// #define USE_ADC_4CH	// uses TIM3
// #define USE_PWM	// uses TIM3, retired for the moment
// #define USE_NOKIA	// implies SPI1 remap or AF 5
// #define USE_FLASHY	// eeprom zone @ (0x08020000-0x400)
#define USE_UART3_FM
#define USE_LCD2x16

#define USE_UART1
#define USE_GPS

#endif


// HSE_EXT est pour utiliser une source d'horloge 8MHz externe
// sur nucleo : MCO de la sonde ST-LINK    8MHz -> PLL -> 72 MHz
// sur blue pill et Olimex : quartz local  8MHz -> PLL -> 72 MHz
// sinon      : oscillateur RC interne HSI 8MHz -> PLL -> 64 Mhz
#define HSE
#ifdef NUCLEO
#define HSE_EXT
#endif
