/* protocole pour emettre ou recevoir des messages en FM 433 MHz

- resume
	- messages de taille variable, pas de delimiteur
	- payload binaire arbitraire, 15 bytes max
	- le premier byte ou opcode contient 4 bits (MSBs) d'operation et 4 bits (LSBs) de taille de payload
	  (la taille n'inclut pas l'opcode)
	- detection d'erreurs par CRC16 sur opcode + payload
- emission :
	- structure du paquet :
		- quelques FF de sync, dont au moins 1 doit arriver au recepteur
		  (il en faut plusieurs au départ a cause du temps de démarrage de l'emetteur et de stabilisation du récepteur)
		- 2 magic bytes, pour éliminer les faux départs causés par le bruit FM
		- 1 opcode, la payload
		- 2 bytes de CRC
- reception :
	- la FSM de reception detecte au moins un FF, les magic numbers, lit l'opcode, deduit la taille de la payload
	  stocke opcode et payload, et verifie le crc
- CRC :
	- polynome CCIT-16, little endian, mask 0x8408 (Koopman 0x8810), CRC-16/KERMIT chez reveng.sourceforge.io
	- init 0, simuler avec CRC16_jln -p
 */

// opcodes (les 4 MSBs)
#define OP_TEST		0x00
#define OP_ECHO		0x10
#define OP_GET		0x20	// le 1er byte est un index de variable
#define OP_SET		0x30	// 	"	"
#define OP_REPORT	0x40	//	"	"

// variables pour GET-SET-REPORT
#define VAR_AUTO1	1
#define VAR_AMP		2

// Tx data
extern char txbuf3[16];	// opcode plus 0 to 15 bytes payload
extern volatile int txindex3;
extern volatile unsigned int tx_paycnt;
extern volatile unsigned int tx_crc;
extern volatile int tx_status;

// Rx data
extern char rxbuf3[16];	// opcode plus 0 to 15 bytes payload
extern volatile int rxindex3;
extern volatile unsigned int rx_paycnt;
extern volatile unsigned int rx_crc;
extern volatile int rx_status;

#ifdef __cplusplus
extern "C" {
#endif

void FM_send( unsigned int opcode, const unsigned char * payload );
// message avec un byte t.q. numero de variable
void FM_send2( unsigned int opcode, unsigned int num, const unsigned char * data );

#ifdef __cplusplus
}
#endif
