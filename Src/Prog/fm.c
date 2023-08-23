#include "options.h"
#ifdef USE_UART3_FM

#include "stm32f1xx_ll_usart.h"
#include "gpio.h"
#include "uarts.h"
#include "fm.h"

// systemes de communication FM 433 MHz
#define FM_MAGIC1 0xA5
#define FM_MAGIC2 0xe6


// Tx data
char txbuf3[16];	// opcode plus 0 to 15 bytes payload
volatile int txindex3;
volatile unsigned int tx_paycnt;
volatile unsigned int tx_crc;
volatile int tx_status = 0;

// Rx data
char rxbuf3[16];	// opcode plus 0 to 15 bytes payload
volatile int rxindex3 = 0;
volatile unsigned int rx_paycnt;
volatile unsigned int rx_crc;
volatile int rx_status = 0;

// CRC CCITT 16 little endian
static unsigned int crc_1byte( unsigned int crc, unsigned int b )
{
for	( unsigned int i = 0; i < 8; i++ )
	{
	if	( ( crc ^ b ) & 1 )
		crc = ( crc >> 1 ) ^ 0x8408;
	else	crc >>= 1;
	b >>= 1;
	}
return crc;
}

// UART3 interrupt handler
void USART3_IRQHandler( void )
{
char c;
if	(
	( LL_USART_IsActiveFlag_TXE( USART3 ) ) &&
	( LL_USART_IsEnabledIT_TXE( USART3 ) )
	)
	{
	// FM Rx FSM status
	//	0 : idle
	//	1..7 : FF to send
	//	8 : magic1 to send
	//	9 : magic2 to send
	//	10 : payload to begin, opcode to send
	//	11 : sending payload data
	//	12 : 2nd CRC byte to send
	//	13 : sacrified byte to send
	//	14 : transmitter to be switched off
	switch	( tx_status )
		{
		case 1:	Tx_cmd(1); Rx_cmd(0);
			LL_USART_TransmitData8( USART3, 0xFF ); tx_status++; break;
		case 2:	LL_USART_TransmitData8( USART3, 0xFF ); tx_status++; break;
		case 3:	LL_USART_TransmitData8( USART3, 0xFF ); tx_status++; break;
		case 4:	LL_USART_TransmitData8( USART3, 0xFF ); tx_status = 8; break;
		case 8:	LL_USART_TransmitData8( USART3, FM_MAGIC1 ); tx_status++; break;
		case 9:	LL_USART_TransmitData8( USART3, FM_MAGIC2 ); tx_status++; break;
		case 10: txindex3 = 0;
			c = txbuf3[txindex3++];
			LL_USART_TransmitData8( USART3, c );
			tx_crc = crc_1byte( 0, c );		// init crc = 0
			tx_paycnt = c & 0x0F;			// taille = 4 LSBs
			tx_status++;
			break;
		case 11: if	( tx_paycnt )
				{
				c = txbuf3[txindex3++];
				LL_USART_TransmitData8( USART3, c );
				tx_crc = crc_1byte( tx_crc, c );
				tx_paycnt--;
				}
			else	{
				LL_USART_TransmitData8( USART3, tx_crc & 0xFF );
				tx_status++;
				}
			break;
		case 12: LL_USART_TransmitData8( USART3, ( tx_crc >> 8 ) & 0xFF );
			tx_status++;
			break;
		case 13: LL_USART_TransmitData8( USART3, ' ' );
			tx_status++;
			break;
		case 14: UART3_TX_INT_disable(); Tx_cmd(0); Rx_cmd(1);
			tx_status = 0;
			break;
		}
	}
if	(
	( LL_USART_IsActiveFlag_RXNE( USART3 ) ) &&
	( LL_USART_IsEnabledIT_RXNE( USART3 ) )
	)
	{
	// FM Rx FSM status
	//	0 : idle
	//	1 : FF received, waiting fo magic
	//	2 : magic1 received, waiting for magic2
	//	3 : magic2 received, waiting for opcode
	//	4 : opcode received, crc started, accepting data or 1st CRC byte
	//	5 : 1st CRC byte Ok, waiting for 2nd CRC byte
	//	10 : CRC ok, waiting for ack
	//	20 : buffer full, waiting for ack
	//	21 : bad crc 1st, waiting for ack
	//	22 : bad crc 2nd, waiting for ack
	c = LL_USART_ReceiveData8( USART3 );
	switch	( rx_status )
		{
		case 0: if	( c == 0xFF )
				{ rx_status = 1; rxindex3 = 0; }
			break;
		case 1:	if	( c == FM_MAGIC1 )
				rx_status = 2;
			else if	( c != 0xFF )
				rx_status = 0;
			break;
		case 2:	if	( c == FM_MAGIC2 )
				rx_status = 3;
			else	rx_status = 0;
			break;
		case 3: if	( rxindex3 < sizeof(rxbuf3) )
				{
				rxbuf3[rxindex3++] = c;			// opcode
				rx_paycnt = c & 0x0F;			// taille = 4 LSBs
				rx_crc = crc_1byte( 0, c );		// init crc = 0
				rx_status = 4;
				}
			else	rx_status = 20;				// buffer full
			break;
		case 4:	if	( rxindex3 <= rx_paycnt )
				{
				if	( rxindex3 < sizeof(rxbuf3) )
					{
					rxbuf3[rxindex3++] = c;
					rx_crc = crc_1byte( rx_crc, c );
					}
				else	rx_status = 20;			// buffer full
				}
			else	{
				if	( c == ( rx_crc & 0xFF ) )
					rx_status = 5;
				else	rx_status = 21;			// bad CRC
				}
			break;
		case 5:
			if	( c == ( ( rx_crc >> 8 ) & 0xFF ) )
				rx_status = 10;
			else	rx_status = 22;			// bad CRC
			break;
		case 10: // nothing to do, app will reset rx_status
		case 20: // nothing to do, app will reset rx_status
		case 21: // nothing to do, app will reset rx_status
		case 22: // nothing to do, app will reset rx_status
			break;
		default: rx_status = 0;
		}
	}
}


// message formatage for FM 433MHz
// message general
void FM_send( unsigned int opcode, const unsigned char * payload )
{
unsigned int i=0, paycnt;
txbuf3[i++] = opcode;
paycnt = opcode & 0x0F;			// taille = 4 LSB
while	( paycnt )
	{
	txbuf3[i++] = *(payload++);
	paycnt--;
	if	( i >= sizeof(txbuf3) )
		break;
	}
tx_status = 1;
UART3_TX_INT_enable();
}

// message avec un byte t.q. numero de variable
void FM_send2( unsigned int opcode, unsigned int num, const unsigned char * data )
{
unsigned int i=0, paycnt;
txbuf3[i++] = opcode;
paycnt = opcode & 0x0F;			// taille = 4 LSB
while	( paycnt )
	{
	if	( i == 1 )
		txbuf3[i++] = (unsigned char)num;
	else	txbuf3[i++] = *(data++);
	paycnt--;
	if	( i >= sizeof(txbuf3) )
		break;
	}
tx_status = 1;
UART3_TX_INT_enable();
}
#endif
