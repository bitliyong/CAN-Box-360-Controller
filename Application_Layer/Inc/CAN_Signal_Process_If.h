#ifndef __CAN_SIGNAL_PROCESS_IF__
#define __CAN_SIGNAL_PROCESS_IF__

#include "CAN_If_Mock.h"
#include "TimeMgr.h"

#ifndef STATIC
#define STATIC              static
#endif

/*
CAN Signal Process Struct Define
*/
typedef struct __CAN_Signal_Process
{
    Handler Initialization;

    GetCANSignal Get_CAN_Signal;
}CAN_Signal_Process_t;

/*
Interface Functions, read design document for details
*/
CAN_Signal_Process_t* CAN_Signal_Process_Instance(void);

#endif