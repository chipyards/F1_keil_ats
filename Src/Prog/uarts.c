#include "options.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
// #include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_usart.h"
#include "uarts.h"

// code d'initialisation commun aux UARTs
static void UART_8_N_1( USART_TypeDef * U, unsigned int perif_clk, unsigned int bauds )
{
//LL_USART_InitTypeDef usart_initstruct;
//usart_initstruct.BaudRate            = bauds;
//usart_initstruct.DataWidth           = LL_USART_DATAWIDTH_8B;
//usart_initstruct.StopBits            = LL_USART_STOPBITS_1;
//usart_initstruct.Parity              = LL_USART_PARITY_NONE;
//usart_initstruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
//usart_initstruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//usart_initstruct.OverSampling        = LL_USART_OVERSAMPLING_16;
//LL_USART_Init( U, &usart_initstruct);


LL_USART_SetBaudRate( U, perif_clk, bauds );
// 8 data bit, 1 start bit, 1 stop bit, no parity */
LL_USART_ConfigCharacter( U, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1 );
LL_USART_SetTransferDirection( U, LL_USART_DIRECTION_TX_RX );
LL_USART_SetHWFlowCtrl( U, LL_USART_HWCONTROL_NONE );
// LL_USART_SetOverSampling( U, LL_USART_OVERSAMPLING_16 );
LL_USART_Enable( U );
}

// config d'un canal d'interruption
static void NVIC_init( IRQn_Type IRQn, int prio )
{
NVIC_SetPriority( IRQn, prio );  
NVIC_EnableIRQ( IRQn );
}

#ifdef USE_UART1
// UART1 en mode interruption ==============================================================
// la fonction USART1_IRQHandler() doit etre définie par ailleurs

// N.B. la pin PA9 correpondant a la sortie TX doit etre configuree en alternate push pull
//      la pin PA10 correpondant a l'entree RX doit etre configuree en alternate
void UART1_init( unsigned int bauds )
{
LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_USART1 );
UART_8_N_1( USART1, SystemCoreClock, bauds );

NVIC_init( USART1_IRQn, 9 );
NVIC_ClearPendingIRQ( USART1_IRQn );
/* Enable USART1 Receive interrupts --> USART1_IRQHandler */
LL_USART_EnableIT_RXNE( USART1 );
// N.B. interruption TX sera validee a la demande
}

// mise en route de l'emission par interruption
void UART1_TX_INT_enable()
{
LL_USART_EnableIT_TXE( USART1 );
}

// arret de l'emission par interruption
void UART1_TX_INT_disable()
{
LL_USART_DisableIT_TXE( USART1 );
}
#endif

#ifdef USE_UART2
// UART2 en mode interruption ==============================================================
// la fonction USART2_IRQHandler() doit etre définie par ailleurs

// N.B. la pin PA2 correpondant a la sortie TX doit etre configuree en alternate push pull
//      la pin PA3 correpondant a l'entree RX doit etre configuree en alternate
// N.B. sur nucleo, PA2 et PA3 peuvent etre reliees au ST-Link pour usage CDC/USB
void UART2_init( unsigned int bauds )
{
LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_USART2 );

UART_8_N_1( USART2, SystemCoreClock/2, bauds );		// UWAGA : APB1 Prescaler = 2

NVIC_init( USART2_IRQn, 10 );
NVIC_ClearPendingIRQ( USART2_IRQn );
/* Enable USART2 Receive interrupts --> USART2_IRQHandler */
LL_USART_EnableIT_RXNE( USART2 );
// N.B. interruption TX sera validee a la demande
}

// mise en route de l'emission par interruption
void UART2_TX_INT_enable()
{
LL_USART_EnableIT_TXE( USART2 );
}

// arret de l'emission par interruption
void UART2_TX_INT_disable()
{
LL_USART_DisableIT_TXE( USART2 );
}
#endif

#ifdef USE_UART3
// UART3 en mode interruption ==============================================================
// la fonction USART3_IRQHandler() doit etre définie par ailleurs

// N.B. la pin PB10 correpondant a la sortie TX doit etre configuree en alternate push pull
//      la pin PB11 correpondant a l'entree RX doit etre configuree en alternate
void UART3_init( unsigned int bauds )
{
LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_USART3 );
UART_8_N_1( USART3, SystemCoreClock/2, bauds );		// UWAGA : APB1 Prescaler = 2

NVIC_init( USART3_IRQn, 11 );
NVIC_ClearPendingIRQ( USART3_IRQn );
/* Enable USART3 Receive interrupts --> USART3_IRQHandler */
LL_USART_EnableIT_RXNE( USART3 );
// N.B. interruption TX sera validee a la demande
}

// mise en route de l'emission par interruption
void UART3_TX_INT_enable()
{
LL_USART_EnableIT_TXE( USART3 );
}

// arret de l'emission par interruption
void UART3_TX_INT_disable()
{
LL_USART_DisableIT_TXE( USART3 );
}
#endif
