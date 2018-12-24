#ifndef __EXTERNAL_DEVICE_PROCESS_IF__
#define __EXTERNAL_DEVICE_PROCESS_IF__

#include "_360_Device.h"
#include "TimeMgr.h"
#include "NVM_Mock_If.h"

#ifndef STATIC
#define STATIC static
#endif

#define EXTERNAL_DEVICE_REGISTER_TABLE_SIZE         (8)

/*
External Device Process Struct Define
*/
typedef struct __External_Device_Process
{
    Handler Initialization;
    Register DeviceRegister;
    GetIndicator GetExternalDeviceInteractionIndicator;
    Processer Process;
} External_Device_Process_t;

/*
Interface Functions, read design document for details
*/
External_Device_Process_t *External_Device_Process_Instance(void);

#endif