
// profilage
// #define PROF_PB12
// #define PROF_PB12_EOS	// ADC End Of Sequence

// osc. modes :
//		HSI		HSE		HSE_EXT
// fmin		 8MHz		 8MHz		 8MHz
// fmax PLL	64MHz		72MHz		72MHz
// nucleo	Y		N		Y
// nucleo cut	Y		N		N
// blue pill	Y		Y		N

/* resume des differences entre Nucleo et Blue Pill :
			Flash		8MHz		LED		BUTTON		PA12
	Nucleo		128k		HSE_EXT	| HSI	PA5  act hi	PC13 act lo	-
	Blue 		 64k		HSE		PC13 act lo	-		pullup USB @ 5V
	ATTENTION au linker script, celui de la Blue Pill est Debug_STM32F103C8_FLASH.ld, il est Ok pour Nucleo mais limite la flash a 64k
 */

// CC1101 fine tuning

// arduino shields
// #define FINE_TUNING (36)  // 1  short SMA shield 1	433.964 -48dBm	-83ppm
// #define FINE_TUNING (37)  // 2  short SMA shield 2   433.963 -50dBm	-85ppm
// #define FINE_TUNING (33)  // 13 short SMA shield 3 
// #define FINE_TUNING (31)  // 14 short SMA shield 4 
// #define FINE_TUNING (31)  // 15 short SMA shield 5 
// #define FINE_TUNING (30)  // 16 short SMA shield 6

// others
// #define FINE_TUNING (13)  // 3	long SMA	433.987 -46dBm	-30ppm	nucleo
// #define FINE_TUNING (12)  // 4	ex-long SMA	433.988 -43dBm	-28ppm	WUCAM
// #define FINE_TUNING (12)  // 5	blue ex-coil	433.988 -52dBm	-28ppm	WUCAM
// #define FINE_TUNING (12)  // 6	blue ex-coil	433.988 -53dBm	-28ppm	WUCAM #1
// #define FINE_TUNING (-8)  // 7	green coil	434.008 -70dBm	+18ppm
// #define FINE_TUNING (-8)  // 8	green coil	434.008 -71dBm	+18ppm

#define BASE_TUNING (434000)

// choix du main
// #define MAIN_COULOMB
// #define MAIN_TERMINAL
#define MAIN_GENERIC

#ifdef MAIN_GENERIC
// nous sommes dans la branche blue_pill, c'est blue pill par defaut
#define NUCLEO
// #define USE_PLL	// 64 MHz (HSI) ou 72 MHZ (HSE, HSE_EXT)

// pour eviter brick de la blue-pill ou nucleo coupee, SLEEP n'est effectif qu'apres 10s depuis reset
#define GREEN_CPU

// modules optionnels
#define USE_CC1101
// N.B Attention FINE TUNING radio board attributions : pour le moment arduino #1, nucleo #2, blue pill #6
#ifdef NUCLEO
  #define USE_CDC
  // #define USE_TIM3_PC6	// uses TIM3 pour moduler via GDO0 = PA10
#define FINE_TUNING (13)	// 3	long SMA	433.987 -46dBm	-30ppm	434013
#else
  #define FINE_TUNING (12)  	// 6	blue coil	433.988 -53dBm	-28ppm	434012
#endif

// #define USE_FLASHY	// eeprom zone @ (0x08020000-0x400)
// #define USE_LCD2x16
#endif

// experiences optionnelles
// #define LEPILOT_TEST	// simulation utilisant la classe Apilot, rejeu-like
#define AUTO_BEACON	// demarrage spontanee de l'emission de msg UHF periodiques
#define TURBO_38K	// 38.4 kbaud au lieu de 10

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
