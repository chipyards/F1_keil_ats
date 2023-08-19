// Blue Pill par defaut
// #define NUCLEO

// profilage
#define PROF_PB12
// #define PROF_PB12_DLY
#define PROF_PB12_EOS
// osc. modes :
//		HSI		HSE		HSE_EXT
// fmax		64MHz		72MHz		72MHz
// nucleo	Y		N		Y
// nucleo cut	Y		N		N
// blue pill	Y		Y		N

// HSE_EXT est pour utiliser une source d'horloge 8MHz externe
// sur nucleo : MCO de la sonde ST-LINK    8MHz -> PLL -> 72 MHz
// sur blue pill et Olimex : quartz local  8MHz -> PLL -> 72 MHz
// sinon      : oscillateur RC interne HSI 8MHz -> PLL -> 64 Mhz

// ici branche blue_pill, c'est blue pill par defaut 
#define HSE
#ifdef NUCLEO
#define HSE_EXT
#endif

// SLEEP necessite connexion reset du ST-Link, sinon brick !
// #define GREEN_CPU

// modules optionnels
#define USE_ADC_4CH	// uses TIM3
// #define USE_PWM	// uses TIM3, retired for the moment
// #define USE_NOKIA	// implies SPI1 remap or AF 5
// #define USE_FLASHY	// eeprom zone @ (0x08020000-0x400)
// #define USE_UART3
// #define USE_LCD2x16

