#ifndef __LOGICAL_CONTROL_IF__
#define __LOGICAL_CONTROL_IF__

#include "CAN_Signal_Process_If.h"
#include "External_Device_Process_If.h"

#ifndef STATIC
#define STATIC              static
#endif

#define EXTERNAL_DEVICE_REGISTER_TABLE_SIZE         (8)

/*
CAN Signal Process Struct Define
*/
typedef struct __Logical_Control
{
    Handler Initialization;

    Processer Process;
    CanBoxModeGetter CanboxMode;
}Logical_Control_t;

/*
Interface Functions, read design document for details
*/
Logical_Control_t* Logical_Control_Instance(void);

#endif