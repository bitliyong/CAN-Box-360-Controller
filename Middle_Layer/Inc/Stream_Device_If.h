#ifndef __STREAM_DEVICE_IF__
#define __STREAM_DEVICE_IF__

#include "UART_If_Mock.h"

#ifndef STATIC
#define STATIC              static
#endif

#define UART_INIT(x)				Uart_Init_360(x)
#define Uart_Tx_Enable(x)		Uart_Tx_Enable_360(x)
#define Uart_Rx_Enable(x)		Uart_Rx_Enable_360(x)
#define Uart_Tx_Disable(x)	Uart_Tx_Disable_360(x)
#define Uart_Rx_Disable(x)	Uart_Rx_Disable_360(x)
#define Uart_Tx(x, y, z)		Uart_Tx_360(x, y, z)
#define Uart_Rx(x, y, z)		Uart_Rx_360(x, y, z)
#define Uart_TxBuf_Flush(x)	Uart_TxBuf_Flush_360(x)
#define Uart_RxBuf_Flush(x)	Uart_RxBuf_Flush_360(x)
/*
Stream Device Struct Define
*/
typedef struct __Stream_Device
{
    Handler Initialization;
    Handler Open;
    Handler Close;

    Transmitter Read;
    Transmitter Write;

    Processer ReadBufferFlusher;
    Processer WriteBufferFlusher;
}Stream_Device_t;

/*
Interface Functions, read design document for details
*/
Stream_Device_t* Stream_Device_Instance(void);

#endif