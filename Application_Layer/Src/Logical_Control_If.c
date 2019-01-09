#include "Logical_Control_If.h"

#define LOGICAL_CONTROL_IF_VERSION  (0102)
#define GEAR_P_ENTER_DELAY          (2000u)//2s
        


//时间管理模块instance句柄
TimeMgr_t *pLCITimeMgr_Instance;
//CAN信号处理模块instance句柄
CAN_Signal_Process_t *pCan_signal_process_instance;
//外部设备处理模块instance句柄
External_Device_Process_t *pExternal_device_process_instance;

/*
外部设备交互指示指针
*/
External_Device_Indicator_t *pIndicator;
/*
CAN信号flag指针
*/
CAN_Signal_Flag_t *pFlag;
/*
D档累计计时定时器
*/
Timer_ID_t gearD_TimerID = TIMER_ID_INVALID;
/*
D当倒计时完成的flag
*/
bool_t gearDDone;
/*
state machine of logical controla
*/
enum _logical_control_enum
{
    //不能被打断的状态
    StartPowerOn,//处理上电 
    StartSurround,//处理上电环绕
    BackToPreWorkMode,//返回到上一模式
    SwitchToNewWorkMode,//切换到新的模式
    WaitPowerOn,//等待上电信号
    WaitPowerOnDone,//等待上电处理完成
    //可以被打断的状态
    WaitSurroundDone = 128u,//等待上电环绕完成
    WaitBackToPreWorkModeDone,//等待返回到上一模式完成
    WaitSwitchToNewWorkModeDone,//等待切换到新模式成功
    WaitForNewOperation,//等待新的操作
}logical_control_state_machine;

/*
上一操作是否完成的flag
*/
bool_t lastOperationDone;
/*
上电倒计时定时器
*/
Timer_ID_t powerOnTimerID = TIMER_ID_INVALID;
/*
环绕倒计时定时器
*/
Timer_ID_t surroundTimerID = TIMER_ID_INVALID;
/*
延时定时器
*/
Timer_ID_t delayTimerID = TIMER_ID_INVALID;
/*
P档进入定时器
*/
Timer_ID_t gearP_TimerID = TIMER_ID_INVALID;
/*
视频通道进入方式
*/
typedef enum _video_channel_enter_way_enum
{
    Undefined_Way = 0x00u,//未定义进入方式
    Auto_Enter_Way = 0x01u,//自动进入

    Manual_Enter_Way = 0x80u,//手动进入
}Video_Enter_Way_t;
/*
模式记录
*/
struct _logical_video_channel_tracker_struct
{
    Video_Channle_t channel;
    Video_Enter_Way_t enterWay;
}preModeChannelTracker, curModeChannelTracker;
/*
上一模式记录
*/
//Video_Channle_t preModeChannel;
/*
下一模式记录
*/
//Video_Channle_t nextModeChannel;

STATIC bool_t Logical_Control_Init(void);
STATIC void Logical_Control_Process(void);
CANBox_Mode_t Logical_Control_CanBox_Mode(void);
STATIC bool_t Logical_Control_Switch_To_New_Work_Mode_Process(Video_Channle_t newChannel, Video_Enter_Way_t enterWay);


/*
instance定义
*/
Logical_Control_t logical_control_instance = 
{
    Logical_Control_Init,
    Logical_Control_Process,
    Logical_Control_CanBox_Mode,
};

/*
优先级：倒车>转向>D档
*/
#define PRIO_INVALID    (0xFFu)
u8_t videoChannelPriority[Surround_View + 1u];

struct __sys_status__
{
    bool_t accOffFlag;
}sysStatus;
/*
获取instance的接口函数
*/
Logical_Control_t* Logical_Control_Instance(void)
{
    return &logical_control_instance;
}
CANBox_Mode_t Logical_Control_CanBox_Mode(void)
{
    return (curModeChannelTracker.channel == Original_View)? Original_Mode : _360_Canbox_Mode;
}

STATIC void videoChannelPriorityInit(void)
{
    videoChannelPriority[Original_View] = 0u;
    videoChannelPriority[Global_Rear_View] = 1u;
    videoChannelPriority[Global_Left_View] = 2u;
    videoChannelPriority[Global_Right_View] = 2u;
    videoChannelPriority[Global_Front_View] = 3u;
    videoChannelPriority[Surround_View] = 15u;
}
STATIC void GearP_Timer_Callback(void *arg)
{
    Logical_Control_Switch_To_New_Work_Mode_Process(preModeChannelTracker.channel, Auto_Enter_Way);
}
STATIC void GearD_Timer_Callback(void *arg)
{
    gearDDone = TRUE;
}
/*
函数具体实现
*/
STATIC bool_t Logical_Control_Init(void)
{
    bool_t res = TRUE;

    //获取时间管理模块instance句柄
    pLCITimeMgr_Instance = TimeMgr_Instance();
    //获取CAN信号处理模块instance句柄
    pCan_signal_process_instance = CAN_Signal_Process_Instance();
    //获取外部设备处理模块instance句柄
    pExternal_device_process_instance = External_Device_Process_Instance();

    //初始化时间管理模块
    pLCITimeMgr_Instance->Initialization();
    //初始化CAN信号处理模块
    pCan_signal_process_instance->Initialization();
        
    //初始化D档相关参数
    //创建D档定时器
    if(gearD_TimerID == TIMER_ID_INVALID)
    {
        gearD_TimerID = pLCITimeMgr_Instance->CreateTimer(15000, Timer_Once, GearD_Timer_Callback, Ptr_NULL);

        TIMER_ASSERT(gearD_TimerID, TIMER_ID_INVALID);
    }

    gearDDone = FALSE;

    if(gearP_TimerID == TIMER_ID_INVALID)
    {
        gearP_TimerID = pLCITimeMgr_Instance->CreateTimer(GEAR_P_ENTER_DELAY, Timer_Once, GearP_Timer_Callback, Ptr_NULL);

        TIMER_ASSERT(gearP_TimerID, TIMER_ID_INVALID);
    }
    //将状态机初始化为等待上电状态
    logical_control_state_machine = WaitPowerOn;
    //上一模式为原车模式
    preModeChannelTracker.channel = Original_View;
    preModeChannelTracker.enterWay = Undefined_Way;
    //下一模式为原车模式
    curModeChannelTracker.channel = Original_View;
    curModeChannelTracker.enterWay = Undefined_Way;

    videoChannelPriorityInit();

    sysStatus.accOffFlag = FALSE;

    return res;
}
//Initialize external device, registe avalidble external device and get indicator from external device
STATIC void Logical_Control_External_Device_Initialization(void)
{
    //初始化外部设备处理模块
    pExternal_device_process_instance->Initialization();

    //注册_360_device到外部设备处理模块
    pExternal_device_process_instance->DeviceRegister(&_360_device);

    //获取外部设备交互指示变量的指针
    pIndicator = pExternal_device_process_instance->GetExternalDeviceInteractionIndicator();
}
STATIC void Logical_Control_Back_To_PreWorkMode_Process(void)
{
    //只有等待状态才可以切换
    if(logical_control_state_machine > 127u)
    {
        //在切换回上一模式之前，如果当前模式正在使用某些资源（如等待环绕完成状态会使用一个定时器），则需要释放这些定时器
        if(WaitSurroundDone == logical_control_state_machine)
        {
            if(surroundTimerID != TIMER_ID_INVALID)
            {
                //删除上电环绕定时器
                pLCITimeMgr_Instance->DeleteTimer(surroundTimerID);

                surroundTimerID = TIMER_ID_INVALID;
            }
        }

        curModeChannelTracker = preModeChannelTracker;

        logical_control_state_machine = BackToPreWorkMode;
    }
}
STATIC bool_t Logical_Contorl_NeedUpdateChannel(Video_Channle_t newChannel)
{
    if((curModeChannelTracker.channel == Global_Rear_View) && (newChannel == Global_Left_View))
        return FALSE;

    if((curModeChannelTracker.channel == Global_Rear_View) && (newChannel == Global_Right_View))
        return FALSE;

    if((curModeChannelTracker.channel == Global_Left_View) && (newChannel == Global_Front_View))
        return FALSE;

    if((curModeChannelTracker.channel == Global_Right_View) && (newChannel == Global_Front_View))
        return FALSE;
    
    return TRUE;
}
STATIC bool_t Logical_Control_Switch_To_New_Work_Mode_Process(Video_Channle_t newChannel, Video_Enter_Way_t enterWay)
{
    bool_t res = TRUE;

    //只有等待状态才可以切换
    if(logical_control_state_machine > 127u)
    {
        //在切换模式之前，如果当前模式正在使用某些资源（如等待环绕完成状态会使用一个定时器），则需要释放这些定时器
        if(WaitSurroundDone == logical_control_state_machine)
        {
            if(surroundTimerID != TIMER_ID_INVALID)
            {
                //删除上电环绕定时器
                pLCITimeMgr_Instance->DeleteTimer(surroundTimerID);

                surroundTimerID = TIMER_ID_INVALID;
            }
        }

        //如果当前模式是手动进入的，则将其标记为上一模式
        if(enterWay == Manual_Enter_Way)
        {
            if(curModeChannelTracker.channel == Global_Rear_View)
                return FALSE;
            
            preModeChannelTracker.channel = newChannel;
            //更新当前模式
            curModeChannelTracker.channel = newChannel;

            logical_control_state_machine = SwitchToNewWorkMode;
        }
        else if(TRUE == Logical_Contorl_NeedUpdateChannel(newChannel))
        {
            pLCITimeMgr_Instance->StopTimer(gearD_TimerID);
            pLCITimeMgr_Instance->StopTimer(gearP_TimerID);
            //更新当前模式
            curModeChannelTracker.channel = newChannel;

            logical_control_state_machine = SwitchToNewWorkMode;
        }
        else
        {
            res = FALSE;
        }
    }
    else
    {
        res = FALSE;
    }

    return res;
}
STATIC void Logical_Control_Gear_R_Process(void)
{
    Logical_Control_Switch_To_New_Work_Mode_Process(Global_Rear_View, Auto_Enter_Way);
}
STATIC void Logical_Control_Gear_DN_Process(void)
{
    bool_t res = FALSE;

    res = Logical_Control_Switch_To_New_Work_Mode_Process(Global_Front_View, Auto_Enter_Way);

    //开启DN档倒计时定时器
    if(res == TRUE)
    {
        pLCITimeMgr_Instance->StartTimer(gearD_TimerID);
    }
}
STATIC void Logical_Control_Gear_P_Process(void)
{
    pLCITimeMgr_Instance->StartTimer(gearP_TimerID);
}

STATIC void Logical_Control_PowerOn_Done_Callback(void *arg)
{
    //倒计时完成，转入上电环绕状态
    lastOperationDone = TRUE;
}
STATIC void Logical_Control_Surround_Done_Callback(void *arg)
{
    //倒计时完成，结束上电环绕模式
    lastOperationDone = TRUE;
}
STATIC void Logical_Control_Delay_Done_Callback(void *arg)
{
    //发送系统时间
    pIndicator->indicator_bits->flag_b.Send_TimeInfo_Sts = 1;
    //删除定时器
    pLCITimeMgr_Instance->DeleteTimer(delayTimerID);
    //Set it to invalid
    delayTimerID = TIMER_ID_INVALID;
}
STATIC void Logical_Control_Change_WorkMode_Process(Video_Channle_t channel)
{
    static Video_Channle_t lastChannel = Invalid_View;

    //避免重复发送命令
    if(channel == lastChannel)
	{
        //last ooperation is done since nothing is going to do
        lastOperationDone = TRUE;

        return;
	}

    lastChannel = channel;
    
    //根据要转入的模式发送命令
    switch(channel)
    {
        case Original_View:
            pIndicator->indicator_bits->flag_b.Send_OriginalView_Sts = 1;
        break;
        case Global_Front_View:
            pIndicator->indicator_bits->flag_b.Send_ShowFrontView_Sts = 1;
        break;
        case Global_Rear_View:
            pIndicator->indicator_bits->flag_b.Send_Reversing_Sts = 1;
        break;
        case Global_Left_View:
            pIndicator->indicator_bits->flag_b.Send_ShowLeftView_Sts = 1;
        break;
        case Global_Right_View:
            pIndicator->indicator_bits->flag_b.Send_ShowRightView_Sts = 1;
        break;
        case Four_Splits_View:
            pIndicator->indicator_bits->flag_b.Send_ShowSplitView_Sts = 1;
        break;
        case Left_Right_Front_View:
            pIndicator->indicator_bits->flag_b.Send_ShowFrontLRView_Sts = 1;
        break;
        case Left_Right_Rear_View:
            pIndicator->indicator_bits->flag_b.Send_ShowRearLRView_Sts = 1;
        break;
        case Global_Front_ZoomOut_View:
            pIndicator->indicator_bits->flag_b.Send_ShowFrontZoomOutView_Sts = 1;
        break;
        case Global_Rear_ZoomOut_View:
            pIndicator->indicator_bits->flag_b.Send_ShowRearZoomOutView_Sts = 1;
        break;
        case Front_FullScreen_View:
            pIndicator->indicator_bits->flag_b.Send_FrontFullScreenView_Sts = 1;
        break;
        case Rear_FullScreen_View:
            pIndicator->indicator_bits->flag_b.Send_RearFullScreenView_Sts = 1;
        break;
        case Global_FullScreen_View:
            pIndicator->indicator_bits->flag_b.Send_ShowGblFullScreenView_Sts = 1;
        break;
        case _3D_FullScreen_View:
            pIndicator->indicator_bits->flag_b.Send_3DFullScreenView_Sts = 1;
        break;
        case Surround_View:
            pIndicator->indicator_bits->flag_b.Send_PowerOnSurround_Sts = 1;
        break;
        default:

        break;
    }
}
STATIC void Logical_Control_Temp_Timer_Release(void)
{
    if(powerOnTimerID != TIMER_ID_INVALID)
    {
        pLCITimeMgr_Instance->StopTimer(powerOnTimerID);
        pLCITimeMgr_Instance->DeleteTimer(powerOnTimerID);
        powerOnTimerID = TIMER_ID_INVALID;
    }

    if(surroundTimerID != TIMER_ID_INVALID)
    {
        pLCITimeMgr_Instance->StopTimer(surroundTimerID);
        pLCITimeMgr_Instance->DeleteTimer(surroundTimerID);
        surroundTimerID = TIMER_ID_INVALID;
    }

    if(delayTimerID != TIMER_ID_INVALID)
    {
        pLCITimeMgr_Instance->StopTimer(delayTimerID);
        pLCITimeMgr_Instance->DeleteTimer(delayTimerID);
        delayTimerID = TIMER_ID_INVALID;
    }
}
STATIC void Logical_Control_ACC_ON_Process(void)
{
    Logical_Control_External_Device_Initialization();
}
STATIC void Logical_Control_ACC_OFF_Process(void)
{
    
    //delete all temprary timer(powerOn, surround, delay) because they will be created again once the system runtime status is occured. 
    Logical_Control_Temp_Timer_Release();
    
    if(1u == pFlag->flag_b.Manual_ShowMode_Sts)
    {
        pFlag->flag_b.Manual_ShowMode_Sts = 0u;

        preModeChannelTracker.channel = Original_View;
    }

    // if(logical_control_state_machine > 127u)
    // {
    //     Logical_Control_Back_To_PreWorkMode_Process();
    // }
	// else
	{
        //if acc is off, then current channel shall set to be original view directly, since no more communication with external device is performed 
        curModeChannelTracker.channel = Original_View;

		logical_control_state_machine = WaitPowerOn;
	}
}
STATIC void Logical_Control_Vehicle_Speed_High_Process(void)
{
    //可以退回到一个新的模式
    if(curModeChannelTracker.channel != preModeChannelTracker.channel)
    {
        if((curModeChannelTracker.channel != Global_Rear_View) /*R档视图不受车速的影响*/
        &&(curModeChannelTracker.channel != Surround_View)/*环绕视图不受车速影响*/
        &&(curModeChannelTracker.channel != preModeChannelTracker.channel))/* 有切回上一视图的必要 */
        {
            //车速过高，切换到上一视图
            Logical_Control_Back_To_PreWorkMode_Process();
        }
    }
}
STATIC void Logical_Control_Enter_ManualGblView_Process(void)
{
    if(Logical_Control_Switch_To_New_Work_Mode_Process(Global_Front_View, Manual_Enter_Way) == TRUE)
    {
        pFlag->flag_b.Manual_ShowMode_Sts = 1u;
    }
}
STATIC void Logical_Control_Exit_ManualGblView_Process(void)
{
    pFlag->flag_b.Manual_ShowMode_Sts = 0u;

    preModeChannelTracker.channel = Original_View;

    Logical_Control_Back_To_PreWorkMode_Process();
}
STATIC void Logical_Control_List_PressedLong_Process(void)
{
    //非手动模式下
    if(0u == pFlag->flag_b.Manual_ShowMode_Sts)
    {
        Logical_Control_Enter_ManualGblView_Process();
    }
    //手动模式下按键不做处理
    else
    {
        //
    }
}
STATIC void Logical_Control_List_Pressed_Process(void)
{
    //手动模式下
    if(1u == pFlag->flag_b.Manual_ShowMode_Sts)
    {
        Logical_Control_Exit_ManualGblView_Process();
    }
    //非手动模式下按键不做处理
    else
    {
        //
    }
}
STATIC void Logical_Control_AnyKey_Pressed_Process(void)
{
    //手动模式下
    if(1u == pFlag->flag_b.Manual_ShowMode_Sts)
    {
        Logical_Control_Exit_ManualGblView_Process();
    }
    //手动模式下按键不做处理
    else
    {
        //
    }
}
STATIC void Logical_Control_TouchScreenInfo_Process(void)
{
    pIndicator->indicator_bits->flag_b.Send_TSInfo_Sts = 1u;
}
STATIC void Logical_Control_Steering_Process(void)
{
    //左转
    if(1u == pFlag->flag_b.Turn_Left_Sts)
    {
        Logical_Control_Switch_To_New_Work_Mode_Process(Global_Left_View, Auto_Enter_Way);
    }
    //右转
    else if(1u == pFlag->flag_b.Turn_Right_Sts)
    {
        Logical_Control_Switch_To_New_Work_Mode_Process(Global_Right_View, Auto_Enter_Way);
    }
    else
    {
        //TODO
    }
}
STATIC void Logical_Control_Vehicle_Signal_Process(void)
{
    //根据flag的值进行处理
    //钥匙在OFF档时根据当前状态机进行处理
    if((1 == pFlag->flag_b.ACC_OFF_Sts) || (1 == pFlag->flag_b.ACC_OUT_Sts))
    {
        if(sysStatus.accOffFlag != TRUE)
        {
            sysStatus.accOffFlag = TRUE;
            
            Logical_Control_ACC_OFF_Process();
        }
    }
    else
    {
        if(sysStatus.accOffFlag != FALSE)
        {
            sysStatus.accOffFlag = FALSE;

            Logical_Control_ACC_ON_Process();
        }
    }
    //档位处理
    if(1 == pFlag->flag_b.Gear_R_Sts)
    {
        Logical_Control_Gear_R_Process();
    }
    if(1 == pFlag->flag_b.Gear_DN_Sts)
    {
        Logical_Control_Gear_DN_Process();
    }
    if(1 == pFlag->flag_b.Gear_P_Sts)
    {
        Logical_Control_Gear_P_Process();
    }
    //车速和转向处理
    //车速低于30km/h&&转向开启全景使能
    if((1u ==pFlag->flag_b.VehicleSpeed_Low_Sts) && 
        (1u == pIndicator->indicator_bits->flag_b.Steering_AutoOpen_GblView_Enable_Sts))
    {
        //转向数据有变化
        if(1u == pFlag->flag_b.Steering_Changed_Sts)
        {
            Logical_Control_Steering_Process();
        }
        //转向灯信号消失
        else if(1u == pFlag->flag_b.Steering_End_Sts)
        {
            //当前正处于左转或者右转视图，才可以切回原车视图，否则维持现有视图不变
            if(curModeChannelTracker.channel == Global_Left_View ||
            curModeChannelTracker.channel == Global_Right_View)
            {
                //返回上一模式
                Logical_Control_Back_To_PreWorkMode_Process();
            }
        }
        else
        {
            //转向信息已经处理
        }
    }
    //车速高于30km/h或转向关闭全景
    else if((0u == pFlag->flag_b.VehicleSpeed_Low_Sts) ||
        (0u == pIndicator->indicator_bits->flag_b.Steering_AutoOpen_GblView_Enable_Sts))
    {
        //手动进入全景
        if(1u == pFlag->flag_b.Manual_ShowMode_Sts)
        {
            //左转
            if(1u == pFlag->flag_b.Turn_Left_Sts)
            {
                Logical_Control_Switch_To_New_Work_Mode_Process(Global_Left_View, Manual_Enter_Way);
            }
            //右转
            else if(1u == pFlag->flag_b.Turn_Right_Sts)
            {
                Logical_Control_Switch_To_New_Work_Mode_Process(Global_Right_View, Manual_Enter_Way);
            }
            //转向灯关闭
            else
            {
                //返回上一模式
                Logical_Control_Back_To_PreWorkMode_Process();
            }
        }
        else if(0u == pFlag->flag_b.VehicleSpeed_Low_Sts)
        {
            Logical_Control_Vehicle_Speed_High_Process();
        }
    }
    //方向盘转角变化，发送方向盘信息
    if(1u == pFlag->flag_b.SteerWheelAngle_Change_Sts)
    {
        pIndicator->indicator_bits->flag_b.Send_YawRateInfo_Sts = 1;
    }
    //按键处理
    if(1u == pFlag->flag_b.Key_List_PressedLong_Sts)
    {
        Logical_Control_List_PressedLong_Process();
    }
    if(1u == pFlag->flag_b.Key_List_Pressed_Sts)
    {
        Logical_Control_List_Pressed_Process();
    }
    if(1u == pFlag->flag_b.Key_Any_Pressed_Sts)
    {
        Logical_Control_AnyKey_Pressed_Process();
    }
    if(1u == pFlag->flag_b.TouchScreen_Change_Sts)
    {
        Logical_Control_TouchScreenInfo_Process();
    }
}
STATIC void Logical_Control_CAN_Signal_Process(void)
{
    #ifdef DEBUG_MODE
    STATIC u32_t counter = 0u;
    #endif
    CAN_Signal_t *CANSignal = pCan_signal_process_instance->Get_CAN_Signal();

    //获取CAN信号flag
    pFlag = CANSignal->flag;

    //获取Vehicle信息
    pIndicator->vehicleInfo = CANSignal->vehicleInfo;
    //刷新时间信息
    pIndicator->vehicleTime = CANSignal->vehicleTime;
    //刷新触摸屏信息
    pIndicator->ts_oprt = CANSignal->ts_oprt;

    //分析信号
    Logical_Control_Vehicle_Signal_Process();

    //判断D当倒计时标记(只有等待状态可以被强制切换)
    if(logical_control_state_machine > 127u)
    {
        if(gearDDone)
        {
            Logical_Control_Back_To_PreWorkMode_Process();

            gearDDone = FALSE;
        }
    }

    //根据状态进行处理
    switch(logical_control_state_machine)
    {
        case WaitPowerOn:
            //判断上电标志
            if(1u == pFlag->flag_b.PowerOn_Sts)
            {
                //切换到开始上电的标志
                logical_control_state_machine = StartPowerOn;
            }
        break;
        case StartPowerOn:
            //开始上电完成倒计时
            lastOperationDone = FALSE;

            if(powerOnTimerID == TIMER_ID_INVALID)
            {
                powerOnTimerID = pLCITimeMgr_Instance->CreateTimer(5000, Timer_Once, 
                    Logical_Control_PowerOn_Done_Callback, Ptr_NULL);

                TIMER_ASSERT(powerOnTimerID, TIMER_ID_INVALID);
                
            }

            pLCITimeMgr_Instance->StartTimer(powerOnTimerID);

            logical_control_state_machine = WaitPowerOnDone;
        break;
        case WaitPowerOnDone:
            //上电完成，转入上电环绕
            if(lastOperationDone)
            {
                //删除上电倒计时定时器
                pLCITimeMgr_Instance->DeleteTimer(powerOnTimerID);

                powerOnTimerID = TIMER_ID_INVALID;

                //上电环绕使能
                if(1u == pIndicator->indicator_bits->flag_b.PowerOn_Startup_Sts)
                {
                    logical_control_state_machine = StartSurround;

                    //将当前视图切换为环绕视图
                    curModeChannelTracker.channel = Surround_View;
                }
                //原车模式
                else
                {
                    logical_control_state_machine = BackToPreWorkMode;
                }
            }
        break;
        case StartSurround:
            //开始上电完成倒计时
            lastOperationDone = FALSE;

            if(surroundTimerID == TIMER_ID_INVALID)
            {
                surroundTimerID = pLCITimeMgr_Instance->CreateTimer(5000, Timer_Once, 
                    Logical_Control_Surround_Done_Callback, Ptr_NULL);

                TIMER_ASSERT(surroundTimerID, TIMER_ID_INVALID);

            }
            
            pLCITimeMgr_Instance->StartTimer(surroundTimerID);

            //切换到上电环绕视图
            Logical_Control_Change_WorkMode_Process(curModeChannelTracker.channel);

            //等待结果
            logical_control_state_machine = WaitSurroundDone;
        break;
        case WaitSurroundDone:
            //lastOperationDone |= pIndicator->indicator_bits->flag_b.Last_Operation_Done_Sts;
            //pIndicator->indicator_bits->flag_b.Last_Operation_Done_Sts = 0u;

            //上电环绕命令发送完成,等待新的操作
            if(lastOperationDone)
            {
                //删除上电环绕定时器
                pLCITimeMgr_Instance->DeleteTimer(surroundTimerID);

                surroundTimerID = TIMER_ID_INVALID;

                logical_control_state_machine = BackToPreWorkMode;
            }
        break;
        case BackToPreWorkMode:
            lastOperationDone = FALSE;

            //发送返回上一模式的命令
            Logical_Control_Change_WorkMode_Process(preModeChannelTracker.channel);

            //等待结果
            logical_control_state_machine = WaitBackToPreWorkModeDone;
        break;
        case WaitBackToPreWorkModeDone:
            lastOperationDone |= pIndicator->indicator_bits->flag_b.Last_Operation_Done_Sts;
            pIndicator->indicator_bits->flag_b.Last_Operation_Done_Sts = 0u;

            //上电环绕命令发送完成,等待新的操作
            if(lastOperationDone)
            {
                //current channel is changed to last one
                curModeChannelTracker = preModeChannelTracker;

                logical_control_state_machine = WaitForNewOperation;
            }
        break;
        case SwitchToNewWorkMode:
            lastOperationDone = FALSE;

            //发送切换到新模式的命令
            Logical_Control_Change_WorkMode_Process(curModeChannelTracker.channel);

            //等待结果
            logical_control_state_machine = WaitSwitchToNewWorkModeDone;
        break;
        case WaitSwitchToNewWorkModeDone:
            lastOperationDone |= pIndicator->indicator_bits->flag_b.Last_Operation_Done_Sts;
            pIndicator->indicator_bits->flag_b.Last_Operation_Done_Sts = 0u;

            //上电环绕命令发送完成,等待新的操作
            if(lastOperationDone)
            {
                logical_control_state_machine = WaitForNewOperation;
            }
        break;
        case WaitForNewOperation:
            //Acc off, then back to "wait power on" state
            if(sysStatus.accOffFlag == TRUE)
            {
                logical_control_state_machine = WaitPowerOn;
            }
        break;
        default:

        break;
    }

    #ifdef DEBUG_MODE
    {
        counter++;

        if(counter > 50u)
        {
            counter = 0u;
            pLCITimeMgr_Instance->Callback_5ms();
        }
    }
    #endif
}
STATIC void Logical_Control_External_Device_Process(void)
{
    //No process is accessable when Acc is off
    if(sysStatus.accOffFlag != FALSE)
        return;

    //如果与360设备握手成功，开始500ms倒计时用于发送系统时间
    if(1u == pIndicator->indicator_bits->flag_b.Handshake_OK_Sts)
    {
        pIndicator->indicator_bits->flag_b.Handshake_OK_Sts = 0u;

        if(delayTimerID == TIMER_ID_INVALID)
        {
            delayTimerID = pLCITimeMgr_Instance->CreateTimer(500, Timer_Once, 
                Logical_Control_Delay_Done_Callback, Ptr_NULL);

            TIMER_ASSERT(delayTimerID, TIMER_ID_INVALID);
            
        }

        pLCITimeMgr_Instance->StartTimer(delayTimerID);
    }

    //处理
    pExternal_device_process_instance->Process();
}
STATIC void Logical_Control_Screen_Brightness_Process(void)
{
    //TODO
}
/*
逻辑控制单元处理函数
*/
STATIC void Logical_Control_Process(void)
{
    //根据CAN信号进行处理
    Logical_Control_CAN_Signal_Process();

    //调用外部设备操作模块的处理函数
    Logical_Control_External_Device_Process();

    //亮度调节
    Logical_Control_Screen_Brightness_Process();
}