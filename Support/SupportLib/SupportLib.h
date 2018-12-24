#ifndef __SUPPORT_LIB__
#define __SUPPORT_LIB__

#include "TypeDef_Std.h"

#define ROLLING_BUFFER_SIZE     (64u)
#define ROLLING_BUFFER_START(x) (&(x->buffer[0]))
#define ROLLING_BUFFER_END(x)   (&(x->buffer[ROLLING_BUFFER_SIZE - 1]))

/*
环形缓冲区定义
*/
typedef struct _rolling_buffer_struct
{
    u8_t *pIn;
    u8_t *pOut;
    u8_t buffer[ROLLING_BUFFER_SIZE];
}Rolling_Buffer_t;

extern void RollingBufferInit(Rolling_Buffer_t *pbuf);
extern u8_t GetDataFromRollingBuffer(Rolling_Buffer_t *pbuf, u8_t *des, u8_t count);
extern void PutDataIntoRollingBuffer(Rolling_Buffer_t *pbuf, u8_t *src, u8_t count);







#endif