#ifndef __CAN_IF_MOCK__
#define __CAN_IF_MOCK__

#include "TypeDef_Std.h"
#include "TypeDef_User.h"

/*
Interface Functions, read design document for details
*/
void CAN_Driver_Init(void);
CAN_Driver_Signal_t* CAN_Driver_GetVehicleSignal(void);

#endif