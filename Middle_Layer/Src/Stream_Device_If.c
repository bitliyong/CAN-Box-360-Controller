#include "Stream_Device_If.h"

STATIC bool_t Stream_Device_Init(void);
STATIC bool_t Stream_Device_Open(void);
STATIC bool_t Stream_Device_Close(void);
STATIC u32_t Stream_Device_Read(u8_t *pbuf);
STATIC u32_t Stream_Device_Write(u8_t *pbuf, u8_t length);
STATIC void Stream_Device_Read_Flush(void);
STATIC void Stream_Device_Write_Flush(void);

/*
instance定义
*/
Stream_Device_t stream_device_instance = 
{
    Stream_Device_Init,
    Stream_Device_Open,
    Stream_Device_Close,

    Stream_Device_Read,
    Stream_Device_Write,

    Stream_Device_Read_Flush,
    Stream_Device_Write_Flush,
};

/*
获取instance的接口函数
*/
Stream_Device_t* Stream_Device_Instance(void)
{
    return &stream_device_instance;
}

/*
函数具体实现
*/
STATIC bool_t Stream_Device_Init(void)
{
    bool_t res = TRUE;
    s8_t  err = Error_None;

    UART_INIT(&err);

    if(err != Error_None)
    {
        res = FALSE;
    }

    return res;
}
STATIC bool_t Stream_Device_Open(void)
{
    bool_t res = TRUE;
    s8_t  err = Error_None;

    Uart_Tx_Enable(&err);

    if(err != Error_None)
    {
        res = FALSE;
    }
    else
    {
        Uart_Rx_Enable(&err);
    }
    
    if(err != Error_None)
    {
        res = FALSE;
    }    
    
    return res;
}
STATIC bool_t Stream_Device_Close(void)
{
    bool_t res = TRUE;
    s8_t  err = Error_None;

    Uart_Tx_Disable(&err);

    if(err != Error_None)
    {
        res = FALSE;
    }
    else
    {
        Uart_Rx_Disable(&err);
    }
    
    if(err != Error_None)
    {
        res = FALSE;
    } 

    return res;
}
STATIC u32_t Stream_Device_Read(u8_t *pbuf)
{
    u32_t cnt = 0;
    u32_t len = 1;
    s8_t  err = Error_None;

    if(pbuf != Ptr_NULL)
    {
        cnt = Uart_Rx(pbuf, len, &err);
    }
    
    return cnt;
}
STATIC u32_t Stream_Device_Write(u8_t *pbuf, u8_t length)
{
    u32_t cnt = 0;
    s8_t  err = Error_None;

    if(pbuf != Ptr_NULL)
    {
        cnt = Uart_Tx(pbuf, length, &err);
    }
    
    return cnt;
}
STATIC void Stream_Device_Read_Flush(void)
{
    s8_t  err = Error_None;

    Uart_RxBuf_Flush(&err);
}
STATIC void Stream_Device_Write_Flush(void)
{
    s8_t  err = Error_None;
    
    Uart_TxBuf_Flush(&err);
}

