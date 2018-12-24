#include "SupportLib.h"

void RollingBufferInit(Rolling_Buffer_t *pbuf)
{
    pbuf->pIn = (u8_t*)ROLLING_BUFFER_START(pbuf);
    pbuf->pOut = (u8_t*)ROLLING_BUFFER_START(pbuf);
}

u8_t GetDataFromRollingBuffer(Rolling_Buffer_t *pbuf, u8_t *des, u8_t count)
{
    s16_t leftDataCount = 0;
    u8_t currentCopyCount = 0u;
    u8_t totalCopyCount = 0u;

    leftDataCount = (pbuf->pIn - pbuf->pOut + ROLLING_BUFFER_SIZE) % ROLLING_BUFFER_SIZE;

    if(leftDataCount > 0)
    {
        totalCopyCount = leftDataCount > count? count : leftDataCount;

        while(currentCopyCount < totalCopyCount)
        {
            *des++ = *pbuf->pOut++;

            currentCopyCount++;

            if(pbuf->pOut > ROLLING_BUFFER_END(pbuf))
            {
                pbuf->pOut = (u8_t*)ROLLING_BUFFER_START(pbuf);
            }
        }
    }

    return currentCopyCount;
}
void PutDataIntoRollingBuffer(Rolling_Buffer_t *pbuf, u8_t *src, u8_t count)
{
    u8_t currentCopyCount = 0u;

    while(currentCopyCount < count)
    {
        *pbuf->pIn++ = *src++;

        currentCopyCount++;

        if(pbuf->pIn > ROLLING_BUFFER_END(pbuf))
        {
            pbuf->pIn = (u8_t*)ROLLING_BUFFER_START(pbuf);
        }
    }  
}