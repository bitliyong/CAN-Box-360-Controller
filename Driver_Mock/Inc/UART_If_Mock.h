#ifndef __UART_IF_MOCK__
#define __UART_IF_MOCK__

#include "TypeDef_Std.h"
#include "TypeDef_User.h"
#include "SupportLib.h"

extern Rolling_Buffer_t uart_rx_buf_360;

/*
Interface Functions, read design document for details
*/
void Uart_Init_360(s8_t *perr);
void Uart_Tx_Enable_360(s8_t *perr);
void Uart_Rx_Enable_360(s8_t *perr);
void Uart_Tx_Disable_360(s8_t *perr);
void Uart_Rx_Disable_360(s8_t *perr);
u32_t Uart_Tx_360(u8_t *pbuf, u32_t cnt, s8_t *perr);
u32_t Uart_Rx_360(u8_t *pbuf, u32_t cnt, s8_t *perr);
void Uart_TxBuf_Flush_360(s8_t *perr);
void Uart_RxBuf_Flush_360(s8_t *perr);

#endif