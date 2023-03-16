/* compression predictive avec perte
PWM resolution = 3266
amplitude nominale target_pk = 1633
frequence horloge = 72000000 Hz
frequence PWM = 22045.3 Hz
frequence audio = 22045.3 Hz
kplaf = 0.4, 5 bits de code soit QLIN = 4, QLOG = 12
serie exponentielle :
	3.5
	4
	5.51499
	7
	8.69002
	11
	13.693
	17
	21.5761
	27
	33.9977
	43
	53.5706
	67
	84.4117
	106
	133.008
	167
	209.583
	263
	330.241
	415
	520.365
	653
tests :
	2 -> 2 -> 2
	-2 -> 18 -> -2
	100 -> 11 -> 106
	-200 -> 28 -> -167
	666 -> 15 -> 653
	1111 -> 15 -> 653
	0 -> 0 -> 0
	2700 -> 15 -> 653
	-7000 -> 31 -> -653
 */
// definitions a reporter dans audio.h :
// frequence audio = 22045.3 Hz
#define QBIT 5
#define PWM_SILENCE 1633
#define PWM_PERIOD  3266
#define SAMP_PERIOD 3266
#define KOVER 1 // <-- OVERSAMPLING
const short dequant[] = {
0, 1, 2, 3, 4, 7, 11, 17, 
27, 43, 67, 106, 167, 263, 415, 653, 
0, -1, -2, -3, -4, -7, -11, -17, 
-27, -43, -67, -106, -167, -263, -415, -653 };
