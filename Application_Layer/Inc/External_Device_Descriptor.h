#ifndef __EXTERNAL_DEVICE_DESCRIPTOR__
#define __EXTERNAL_DEVICE_DESCRIPTOR__


#include "TypeDef_Std.h"
#include "TypeDef_User.h"

/*
External Device Struct Define
*/
typedef struct __External_Device
{
    const char *name;
    u8_t classCode;
    u8_t index;
    u8_t version;
    u8_t *pdata;
    
    Handler Initialization;
    Handshaker Handshake;
    DeviceProcesser Process;
} External_Device_t;

#endif