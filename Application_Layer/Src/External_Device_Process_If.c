#include "External_Device_Process_If.h"

STATIC bool_t External_Device_Process_Init(void);
STATIC void External_Device_Process_Register(void *dev);
STATIC External_Device_Indicator_t* External_Device_Process_Indicator(void);
STATIC void External_Device_Process_Process(void);

//外部设备注册表定义
External_Device_t* external_device_register_table[EXTERNAL_DEVICE_REGISTER_TABLE_SIZE];
//注册表可用位置定义
u8_t external_device_register_table_free_posistion;
//当前活动的外部设备指针
External_Device_t* pActiveExternalDevice;

//外部设备操作模块状态机定义
enum _External_Device_State_Machine
{
    Enumeration,
    Interaction,
    Pend,
}state_machine;

/*
instance定义
*/
External_Device_Process_t external_device_process_instance = 
{
    External_Device_Process_Init,
    External_Device_Process_Register,
    External_Device_Process_Indicator,
    External_Device_Process_Process,
};
/*
时间管理模块instance
*/
TimeMgr_t* External_Device_Process_TimeMgr_Instance;
/*
用于执行挂起操作的定时器
*/
Timer_ID_t pendTimeoutTimerID;
/*
挂起flag
*/
bool_t pendStartFlag;
bool_t pendEndFlag;

/*
外部设备需要进行处理的状态标志
*/
External_Device_Indicator_t externalDeviceIndicator;
External_Device_Interaction_Indicator_Bit_t indicator_bit;

/*
NVM中存储的菜单项
*/
NVM_Info_t* external_device_process_nvmInfo;
/*
局部函数声明
*/
STATIC void PendTimeout(void *arg);

/*
获取instance的接口函数
*/
External_Device_Process_t* External_Device_Process_Instance(void)
{
    return &external_device_process_instance;
}
/*
获取indicator的地址
*/
STATIC External_Device_Indicator_t* External_Device_Process_Indicator(void)
{
    return &externalDeviceIndicator;
}
/*
初始化注册表
*/
STATIC void External_Device_Process_Register_Table_Init(void)
{
    u8_t i = 0;

    for(; i < EXTERNAL_DEVICE_REGISTER_TABLE_SIZE; i ++)
    {
        external_device_register_table[i] = Ptr_NULL;
    }

    external_device_register_table_free_posistion = 0;
}

/*
函数具体实现
*/
STATIC bool_t External_Device_Process_Init(void)
{
    bool_t res = TRUE;

    //初始化注册表
    External_Device_Process_Register_Table_Init();

    //状态机初始化
    //初始化逻辑控制模块状态机
    state_machine = Enumeration;

    //初始化外部活动设备指针
    pActiveExternalDevice = Ptr_NULL;

    //获取时间管理模块instance
    External_Device_Process_TimeMgr_Instance = TimeMgr_Instance();

    //初始化indicator
    externalDeviceIndicator.vehicleInfo = Ptr_NULL;
    externalDeviceIndicator.indicator_bits = &indicator_bit;

    //初始化菜单设置选项
    external_device_process_nvmInfo = NVM_GetNvMInfo();
    externalDeviceIndicator.indicator_bits->flag_b.PowerOn_Startup_Sts = external_device_process_nvmInfo->menuSetting->settings_b.Poweron_Startup_Sts;
    externalDeviceIndicator.indicator_bits->flag_b.Steering_AutoOpen_GblView_Enable_Sts = external_device_process_nvmInfo->menuSetting->settings_b.Steering_Open_Sts;
    externalDeviceIndicator.indicator_bits->flag_b.Radar_Obstacle_Startup_Sts = external_device_process_nvmInfo->menuSetting->settings_b.Radar_Obstacle_Open_Sts;
    externalDeviceIndicator.standby_time = external_device_process_nvmInfo->menuSetting->settings_b.Standby_Time;
    externalDeviceIndicator.vehicleSpeed_threshold = external_device_process_nvmInfo->menuSetting->settings_b.vehicleSpeed_threshold;

    externalDeviceIndicator.indicator_bits->flag_b.Handshake_OK_Sts = 1u;
    return res;
}
STATIC void External_Device_Process_Register(void *dev)
{
    External_Device_t* pdev = (External_Device_t*)dev;

    //外部设备指针有效
    if(pdev != Ptr_NULL)
    {
        //初始化外部设备
        pdev->Initialization();
        
        //外部设备注册表还有空间
        if(external_device_register_table_free_posistion < EXTERNAL_DEVICE_REGISTER_TABLE_SIZE)
        {
            //添加到注册表中
            external_device_register_table[external_device_register_table_free_posistion] = pdev;

            //可用位置后移1位
            external_device_register_table_free_posistion++;
        }
    }
}
STATIC void External_Device_Process_Process(void)
{
    Handshake_Result_t handshakeResult;

    //状态机轮转
    switch(state_machine)
    {
        case Enumeration:
            {
                u8_t i = 0;

                pActiveExternalDevice = Ptr_NULL;

                for(; i < external_device_register_table_free_posistion; i++)
                {
                    handshakeResult = external_device_register_table[i]->Handshake();

                    //握手成功
                    if(Handshake_OK == handshakeResult)
                    {
                        //设置当前活动设备指针
                        pActiveExternalDevice = external_device_register_table[i];

                        //状态机切换，转入交互阶段
                        state_machine = Interaction;

                        break;
                    }
                    //握手未完成
                    else if(Handshake_Unfinished == handshakeResult)
                    {
                        break;
                    }
                    else
                    {
                        //TODO
                    }
                }

                //枚举未成功
                if(external_device_register_table_free_posistion == i)
                {
                    pendStartFlag = TRUE;
                    pendEndFlag = FALSE;

                    //状态机切换，转入挂起阶段
                    state_machine = Pend;
                }            
            }
        break;
        case Interaction:

            if(FALSE == pActiveExternalDevice->Process(&externalDeviceIndicator))
            {
                pendStartFlag = TRUE;
                pendEndFlag = FALSE;

                //状态机切换，转入挂起阶段
                state_machine = Pend;
            }

        break;
        case Pend:
            //挂起2秒
            if(TRUE == pendStartFlag)
            {
                //创建一个挂起计时定时器
                pendTimeoutTimerID = External_Device_Process_TimeMgr_Instance->CreateTimer(2000, Timer_Once, PendTimeout, Ptr_NULL);

                External_Device_Process_TimeMgr_Instance->StartTimer(pendTimeoutTimerID);

                pendStartFlag = FALSE;
            }
            else if(TRUE == pendEndFlag)
            {
                //释放定时器资源
                External_Device_Process_TimeMgr_Instance->DeleteTimer(pendTimeoutTimerID);

                //状态机切换，转入枚举状态
                state_machine = Enumeration;

                pendEndFlag = FALSE;
            }
            else
            {
                //we just wait for timeout
            }
        break;
    }
}
STATIC void PendTimeout(void *arg)
{
    pendEndFlag = TRUE;
}