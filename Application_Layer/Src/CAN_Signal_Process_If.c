#include "CAN_Signal_Process_If.h"

#define TURNING_MONITOR_DELAY          (3000u)

STATIC bool_t CAN_Signal_Process_Init(void);
STATIC CAN_Signal_t* CAN_Signal_Process_GetVehicleSignal(void);

/*
CAN驱动层传输过来的指针
*/
CAN_Driver_Signal_t *pCanDriverSignal;
/*
车身信息记录，用于判断是否发生了变化
*/
Vehicle_Info_t vehicleInfo_Record;
/*
触摸屏信息记录，用于判断是否发生了变化
*/
TS_Oprt_t tsInfo_Record;
/*
经过处理的CAN信号
*/
CAN_Signal_t canSignal;
CAN_Signal_Flag_t canSignalFlag;
/*
CAN信号处理定时器
*/
TimeMgr_t *pCSPITimeMgr_Instance;
Timer_ID_t canTimer_200ms = TIMER_ID_INVALID;
Timer_ID_t turningMonitorTimer = TIMER_ID_INVALID;
/*
list键按下的时间
*/
u32_t listKeyPressedTime;
/*
instance定义
*/
CAN_Signal_Process_t can_signal_process_instance = 
{
    CAN_Signal_Process_Init,
    CAN_Signal_Process_GetVehicleSignal,
};

/*
获取instance的接口函数
*/
CAN_Signal_Process_t* CAN_Signal_Process_Instance(void)
{
    return &can_signal_process_instance;
}

/*
函数具体实现
*/
STATIC void CAN_Signal_Process_Timer_Callback(void *arg)
{
    //如果2s定时器到，说明list键被按下达到2s
    listKeyPressedTime += 200u;
}
STATIC void CAN_Signal_Process_TurningMonitor_Timer_Callback(void *arg)
{
    //如果2s定时器到，说明转向信号彻底消失
    canSignal.flag->flag_b.Steering_End_Sts = 1u;
}
STATIC bool_t CAN_Signal_Process_Init(void)
{
    bool_t res = TRUE;

    //初始化CAN总线
    CAN_Driver_Init();

    //初始化CAN_Flag
    canSignalFlag.flag_b.ACC_OUT_Sts = 1;
    canSignalFlag.flag_b.VehicleSpeed_Low_Sts = 1;
    canSignal.flag = &canSignalFlag;

    //获取时间管理句柄
    pCSPITimeMgr_Instance = TimeMgr_Instance();

    if(canTimer_200ms == TIMER_ID_INVALID)
    {
        //创建定时器
        canTimer_200ms = pCSPITimeMgr_Instance->CreateTimer(200u, Timer_Repeat, CAN_Signal_Process_Timer_Callback, Ptr_NULL);
    
        TIMER_ASSERT(canTimer_200ms, TIMER_ID_INVALID);
    }
    if(turningMonitorTimer == TIMER_ID_INVALID)
    {
        turningMonitorTimer = pCSPITimeMgr_Instance->CreateTimer(TURNING_MONITOR_DELAY, Timer_Once, CAN_Signal_Process_TurningMonitor_Timer_Callback, Ptr_NULL);
    
        TIMER_ASSERT(turningMonitorTimer, TIMER_ID_INVALID);
    }
    
    listKeyPressedTime = 0u;
    //初始化车身信息记录
    vehicleInfo_Record.basicInfo.basicInfos = 0;
    vehicleInfo_Record.lightAcceleratorInfo.lightAndAcceleratorInfos = 0;
    vehicleInfo_Record.steeringAngleInfo = 0;
    vehicleInfo_Record.speedInfo = 0;
    vehicleInfo_Record.frontRadarInfo.radarInfos = 0;
    vehicleInfo_Record.rearRadarInfo.radarInfos = 0;
    vehicleInfo_Record.engineSpeedInfo = 0;
    vehicleInfo_Record.doorPInfo.doorPInfos = 0;

    tsInfo_Record.x.high = 0u;
    tsInfo_Record.x.low = 0u;
    tsInfo_Record.y.high = 0u;
    tsInfo_Record.y.low = 0u;
    tsInfo_Record.status = Non_Pressed;

    return res;
}
/*
处理CAN报文
*/
STATIC void CAN_Signal_Process_Key_Pressed_Process(void)
{
    //list被按下，开启长按检测
    if(pCanDriverSignal->keyFlag->keyFlags_b.ListKey_Pressed_Flag)
    {
        pCSPITimeMgr_Instance->StartTimer(canTimer_200ms);
    }
    //没有被按下，检测定时器关闭
    else
    {
        //停止定时器
        pCSPITimeMgr_Instance->StopTimer(canTimer_200ms);
        //list键曾被按下
        if(listKeyPressedTime > 0u)
        {
            //长按键
            if(listKeyPressedTime >= 2000u)
            {
                canSignalFlag.flag_b.Key_List_PressedLong_Sts = 1u;
                canSignalFlag.flag_b.Key_List_Pressed_Sts = 0u;
            }
            //短按
            else
            {
                canSignalFlag.flag_b.Key_List_PressedLong_Sts = 0u;
                canSignalFlag.flag_b.Key_List_Pressed_Sts = 1u;
            }

            listKeyPressedTime = 0u;
        }
        //list键未被按下
        else
        {
            canSignalFlag.flag_b.Key_List_PressedLong_Sts = 0u;
            canSignalFlag.flag_b.Key_List_Pressed_Sts = 0u;
        }
        
    }
    //任意键被按下
    if(pCanDriverSignal->keyFlag->keyFlags_b.AnyKey_Pressed_Flag)
    {
        canSignalFlag.flag_b.Key_Any_Pressed_Sts = 1;
    }
    else
    {
        canSignalFlag.flag_b.Key_Any_Pressed_Sts = 0;
    }
}
STATIC void CAN_Signal_Process_BasicvehicleInfo_Process(void)
{
    Vehicle_Basic_Info_Bit_t basicInfo = pCanDriverSignal->vehicleInfo->basicInfo;

    //检测钥匙的位置
    canSignalFlag.flag_b.ACC_OUT_Sts = (basicInfo.basicInfos_b.ACC_Sts == 0u)? 1u : 0u;
    canSignalFlag.flag_b.ACC_OFF_Sts = (basicInfo.basicInfos_b.ACC_Sts == 1u)? 1u : 0u;
    canSignalFlag.flag_b.ACC_Start_Sts = (basicInfo.basicInfos_b.ACC_Sts == 2u)? 1u : 0u;
    canSignalFlag.flag_b.ACC_ON_Sts = (basicInfo.basicInfos_b.ACC_Sts == 3u)? 1u : 0u;

    //是否是从OFF-->ST/ON
    if((vehicleInfo_Record.basicInfo.basicInfos_b.ACC_Sts <= 1) && (basicInfo.basicInfos_b.ACC_Sts > 1))
    {
        canSignalFlag.flag_b.PowerOn_Sts = 1;
    }
    else
    {
        canSignalFlag.flag_b.PowerOn_Sts = 0;
    }

    //档位状态发生变化
    if(vehicleInfo_Record.basicInfo.basicInfos_b.GER_Sts != basicInfo.basicInfos_b.GER_Sts)
    {
        canSignalFlag.flag_b.Gear_P_Sts = (basicInfo.basicInfos_b.GER_Sts == 0u)? 1u : 0u;
        canSignalFlag.flag_b.Gear_R_Sts = (basicInfo.basicInfos_b.GER_Sts == 1u)? 1u : 0u;
        canSignalFlag.flag_b.Gear_DN_Sts = (basicInfo.basicInfos_b.GER_Sts > 1u)? 1u : 0u;
    }
    //档位状态保持不变
    else
    {
        canSignalFlag.flag_b.Gear_P_Sts = 0u;
        canSignalFlag.flag_b.Gear_R_Sts = 0u;
        canSignalFlag.flag_b.Gear_DN_Sts = 0u;
    }
}
STATIC void CAN_Signal_Process_LampAcceleratorInfo_Process(void)
{
    Vehicle_Light_Accelerator_Info_Bit_t lampAcc = pCanDriverSignal->vehicleInfo->lightAcceleratorInfo;

    //转向灯数据
    canSignalFlag.flag_b.Turn_Left_Sts = lampAcc.lightAcceleratorInfo_b.leftLight_Sts;
    canSignalFlag.flag_b.Turn_Right_Sts = lampAcc.lightAcceleratorInfo_b.rightLight_Sts;
    canSignalFlag.flag_b.Double_Flash_Sts = lampAcc.lightAcceleratorInfo_b.DFLight_Sts;

    //转向灯是否变化
    if((vehicleInfo_Record.lightAcceleratorInfo.lightAcceleratorInfo_b.leftLight_Sts != 
    lampAcc.lightAcceleratorInfo_b.leftLight_Sts) ||
    (vehicleInfo_Record.lightAcceleratorInfo.lightAcceleratorInfo_b.rightLight_Sts != 
    lampAcc.lightAcceleratorInfo_b.rightLight_Sts) ||
    (vehicleInfo_Record.lightAcceleratorInfo.lightAcceleratorInfo_b.DFLight_Sts != 
    lampAcc.lightAcceleratorInfo_b.DFLight_Sts))
    {
        //转向信号变为0
        if((lampAcc.lightAcceleratorInfo_b.leftLight_Sts == 0u) &&
        (lampAcc.lightAcceleratorInfo_b.rightLight_Sts == 0u))
        {
            //继续监视一段时间，这段时间内如果转向信号一直为0，则认为转向信号已经结束
            pCSPITimeMgr_Instance->StartTimer(turningMonitorTimer);
        }
        else
        {
            //转向信号依然存在，停止监视，直到转向信号消失再重新开始监视
            pCSPITimeMgr_Instance->StopTimer(turningMonitorTimer);
            canSignalFlag.flag_b.Steering_End_Sts = 0u;
        }

        //提示转向信号已经改变
        canSignalFlag.flag_b.Steering_Changed_Sts = 1;
    }
    else
    {
        canSignalFlag.flag_b.Steering_Changed_Sts = 0;
    }
}
STATIC void CAN_Signal_Process_VehicleSpeedInfo_Process(void)
{
    Vehicle_Speed_Info_t speed = pCanDriverSignal->vehicleInfo->speedInfo;

    canSignalFlag.flag_b.VehicleSpeed_Low_Sts = (speed < 30)? 1 : 0;
}
STATIC void CAN_Signal_Process_SteerWheelAngleInfo_Process(void)
{
    Vehicle_SteeringAngle_Info_t angle = pCanDriverSignal->vehicleInfo->steeringAngleInfo;

    //方向盘转角变化标志
    canSignalFlag.flag_b.SteerWheelAngle_Change_Sts = (vehicleInfo_Record.steeringAngleInfo != angle)? 1 : 0;
}
STATIC void CAN_Signal_Process_FrontRadarInfo_Process(void)
{
    //
}
STATIC void CAN_Signal_Process_RearRadarInfo_Process(void)
{
    //
}
STATIC void CAN_Signal_Process_DoorPKeyInfo_Process(void)
{
    //
}
STATIC void CAN_Signal_Process_EngineSpeedInfo_Process(void)
{
    //
}
STATIC void CAN_Signal_Process_vehicleInfo_Process(void)
{
    //处理车身信息
    //首先处理车身基本状态
    CAN_Signal_Process_BasicvehicleInfo_Process();
    //处理车灯油门状态
    CAN_Signal_Process_LampAcceleratorInfo_Process();
    //处理车速信息
    CAN_Signal_Process_VehicleSpeedInfo_Process();
    //处理方向盘转角信息
    CAN_Signal_Process_SteerWheelAngleInfo_Process();
    //处理前雷达信息
    CAN_Signal_Process_FrontRadarInfo_Process();
    //处理后雷达信息
    CAN_Signal_Process_RearRadarInfo_Process();
    //处理车门状态，P键状态
    CAN_Signal_Process_DoorPKeyInfo_Process();
    //处理发送机转速信息
    CAN_Signal_Process_EngineSpeedInfo_Process();

    //记录当前车身状态
    vehicleInfo_Record = *pCanDriverSignal->vehicleInfo;
}
STATIC void CAN_Signal_Process_LampInfo_Process(void)
{
    canSignalFlag.flag_b.Lamp_1_Sts = (pCanDriverSignal->lampFlag->LampFlags_b.Lamp1_Flag == 1u)? 1u : 0u;
}
STATIC bool_t CAN_Signal_Process_TouchScreenInfo_Changed(TS_Oprt_t *pre, TS_Oprt_t *cur)
{
    bool_t res = FALSE;

    res |= (pre->x.high != cur->x.high);
    res |= (pre->x.low != cur->x.low);
    res |= (pre->y.high != cur->y.high);
    res |= (pre->y.low != cur->y.low);
    res |= (pre->status != cur->status);

    return res;
}
STATIC void CAN_Signal_Process_TouchScreenInfo_Process(void)
{
    if(CAN_Signal_Process_TouchScreenInfo_Changed(&tsInfo_Record, canSignal.ts_oprt))
    {
        canSignalFlag.flag_b.TouchScreen_Change_Sts = 1u;

        tsInfo_Record = *canSignal.ts_oprt;
    }
    else
    {
        canSignalFlag.flag_b.TouchScreen_Change_Sts = 0u;
    }
}
STATIC void CAN_Signal_Process_Process(void)
{
    //从驱动层获取到了信号
    if(pCanDriverSignal != Ptr_NULL)
    {
        //首先判断按键状态
        CAN_Signal_Process_Key_Pressed_Process();
        //处理车身信息
        CAN_Signal_Process_vehicleInfo_Process();
        //处理车灯状态
        CAN_Signal_Process_LampInfo_Process();
        //处理触摸屏信息
        CAN_Signal_Process_TouchScreenInfo_Process();
    }
}
STATIC CAN_Signal_t* CAN_Signal_Process_GetVehicleSignal(void)
{
    //获取底层CAN信号
    pCanDriverSignal = CAN_Driver_GetVehicleSignal();

    //获取车身信息指针
    canSignal.vehicleInfo = pCanDriverSignal->vehicleInfo;
    canSignal.vehicleTime = pCanDriverSignal->vehicleTime;
    canSignal.ts_oprt = pCanDriverSignal->ts_oprt;

    //处理CAN信号
    CAN_Signal_Process_Process();

    return &canSignal;
}