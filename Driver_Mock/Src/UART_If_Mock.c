#include "UART_If_Mock.h"
#include "uart.h"

Rolling_Buffer_t uart_rx_buf_360;

/*
串口初始化
*/
void Uart_Init_360(s8_t *perr)
{
    if(perr != Ptr_NULL)
    {
        perr = Error_None;
    }

    RollingBufferInit(&uart_rx_buf_360);

    //UART_Init();
}
/*
使能发送
*/
void Uart_Tx_Enable_360(s8_t *perr)
{
    if(perr != Ptr_NULL)
    {
        perr = Error_None;
    }
}
/*
使能接收
*/
void Uart_Rx_Enable_360(s8_t *perr)
{
    if(perr != Ptr_NULL)
    {
        perr = Error_None;
    }
}
/*
禁止发送
*/
void Uart_Tx_Disable_360(s8_t *perr)
{
    if(perr != Ptr_NULL)
    {
        perr = Error_None;
    }
}
/*
禁止接收
*/
void Uart_Rx_Disable_360(s8_t *perr)
{
    if(perr != Ptr_NULL)
    {
        perr = Error_None;
    }
}
/*
发送
*/
u32_t Uart_Tx_360(u8_t *pbuf, u32_t cnt, s8_t *perr)
{
    if(perr != Ptr_NULL)
    {
        perr = Error_None;
    }

    //Uart3_Put_data(cnt, pbuf);

    return cnt;
}
/*
接收
*/
u32_t Uart_Rx_360(u8_t *pbuf, u32_t cnt, s8_t *perr)
{
    u32_t dataLen = 0;

    if(perr != Ptr_NULL)
    {
        perr = Error_None;
    }

    dataLen = GetDataFromRollingBuffer(&uart_rx_buf_360, pbuf, cnt);

    return dataLen;
}
/*
刷新发送缓冲区
*/
void Uart_TxBuf_Flush_360(s8_t *perr)
{
    if(perr != Ptr_NULL)
    {
        perr = Error_None;
    }
}
/*
刷新接收缓冲区
*/
void Uart_RxBuf_Flush_360(s8_t *perr)
{
    if(perr != Ptr_NULL)
    {
        perr = Error_None;
    }
}