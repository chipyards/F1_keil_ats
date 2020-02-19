#ifdef __cplusplus
extern "C" {
#endif

// UART1 pour RS232
void UART1_init( unsigned int bauds );
void UART1_TX_INT_enable(void);
void UART1_TX_INT_disable(void);

// UART2 pour XBee, ou interface CDC-USB sur nucleo non modifiee
void UART2_init( unsigned int bauds );
void UART2_TX_INT_enable(void);
void UART2_TX_INT_disable(void);

// UART3 pour FM
void UART3_init( unsigned int bauds );
void UART3_TX_init( unsigned int bauds );	// UART3 TX only
void UART3_TX_INT_enable(void);
void UART3_TX_INT_disable(void);

#ifdef __cplusplus
}
#endif
