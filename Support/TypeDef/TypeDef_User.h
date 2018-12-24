/*
有关用户自定义的数据类型的类型定义
*/
#ifndef __TYPE_DEF_USER__
#define __TYPE_DEF_USER__

#include "TypeDef_Std.h"
/*
错误码定义
*/
#define Error_None          (0)
#define Error_Ptr_Null      (-1)
#define Error_UART_UnInit   (-2)

/*
开关值定义
*/
#define STD_ON              (1)
#define STD_OFF             (0)

//禁止握手
#define HAND_SHAKE_ENABLE   STD_OFF
//不检查结果
#define CHECK_ACK_ENABLE    STD_OFF
/*
定义CAN信号状态类型，可以通过这个类型定义修改状态位的bit数
*/
typedef u32_t    CAN_Signal_Sts_t;

#define CAN_Signal_Sts_Size     (sizeof(CAN_Signal_Sts_t)*8) 

typedef union
{
    CAN_Signal_Sts_t flag;

    struct
    {
        CAN_Signal_Sts_t Gear_R_Sts      : 1;//进入R档标志
        CAN_Signal_Sts_t Gear_DN_Sts     : 1;//进入D/N档标志
        CAN_Signal_Sts_t VehicleSpeed_Low_Sts : 1;//车速低于30Km/h标志
        CAN_Signal_Sts_t Gear_P_Sts : 1;//进入P档标志
        CAN_Signal_Sts_t SteerWheelAngle_Change_Sts : 1;//方向盘转角变化标志
        CAN_Signal_Sts_t TurnLight_Change_Sts : 1;//转向灯变化标志
        CAN_Signal_Sts_t Turn_Right_Sts : 1;//右转标志
        CAN_Signal_Sts_t Turn_Left_Sts : 1;//左转标志
        CAN_Signal_Sts_t Steering_End_Sts : 1;//转向消失标志
        CAN_Signal_Sts_t Double_Flash_Sts : 1;//双闪标志
        
        CAN_Signal_Sts_t Gear_Switch_Sts : 1;//档位切换标志
        CAN_Signal_Sts_t Steering_Changed_Sts : 1;//转向开始标志
        CAN_Signal_Sts_t _360ShowMode_Sts : 1;//360显示模式标志
        CAN_Signal_Sts_t Manual_ShowMode_Sts : 1;//手动显示全景模式标志

        CAN_Signal_Sts_t PowerOn_Sts : 1;//上电标志
        CAN_Signal_Sts_t ACC_OUT_Sts : 1;//车钥匙拔出标志
        CAN_Signal_Sts_t ACC_OFF_Sts : 1;//ACC处于OFF标志
        CAN_Signal_Sts_t ACC_Start_Sts : 1;//ACC处于Start标志
        CAN_Signal_Sts_t ACC_ON_Sts : 1;//ACC处于ON标志

        CAN_Signal_Sts_t Lamp_1_Sts : 1;//小灯状态
        CAN_Signal_Sts_t Key_List_PressedLong_Sts : 1;//List键长按标志
        CAN_Signal_Sts_t Key_List_Pressed_Sts : 1;//List键按下标志（短按）
        CAN_Signal_Sts_t Key_Any_Pressed_Sts : 1;//任意键按下标志（短按）-任意键不包括List键
        CAN_Signal_Sts_t TouchScreen_Change_Sts : 1;//触摸屏信息变化标志
 				
        CAN_Signal_Sts_t ReservedBits : (CAN_Signal_Sts_Size - 24);//保留位
    }flag_b;
}CAN_Signal_Flag_t;

/*
逻辑控制操作状态标志类型定义
*/
typedef u32_t    Logical_Control_Sts_t;

#define Logical_Control_Sts_Size     (sizeof(Logical_Control_Sts_t)*8) 

typedef union
{
    Logical_Control_Sts_t flag;

    struct
    {   
        //传递给逻辑控制单元的状态，
        Logical_Control_Sts_t Steering_AutoOpen_GblView_Enable_Sts      : 1;//转向时自动开启全景标志,逻辑控制模块不能修改这位的值
        Logical_Control_Sts_t Handshake_OK_Sts : 1;//握手成功标志，逻辑控制模块需要在使用之后清空这位的值
        Logical_Control_Sts_t Project_Mode_Sts : 1;//工程模式标志，1-有效，0-无效
        Logical_Control_Sts_t PowerOn_Startup_Sts : 1;//开机启动环绕标志，1-有效，0-无效
        Logical_Control_Sts_t Radar_Obstacle_Startup_Sts : 1;//雷达障碍物启动
        Logical_Control_Sts_t Last_Operation_Done_Sts : 1;//上一操作完成，逻辑控制模块需要在使用之后清空这位的值
        Logical_Control_Sts_t ReservedBit : 2;//保留位，
        //传递给外部设备的状态，外部设备不能修改这些位的值
        Logical_Control_Sts_t Send_PowerOnSurround_Sts : 1;//发送上电环绕命令标志        
        Logical_Control_Sts_t Send_OriginalView_Sts     : 1;//发送原车模式标志        
        Logical_Control_Sts_t Send_Reversing_Sts : 1;//发送倒车命令        
        Logical_Control_Sts_t Send_GearInfo_Sts : 1;//发送档位信息
        Logical_Control_Sts_t Send_YawRateInfo_Sts : 1;//发送方向盘信息
        Logical_Control_Sts_t Send_ShowRightView_Sts : 1;//发送显示右视命令
        Logical_Control_Sts_t Send_ShowSplitView_Sts : 1;//发送显示4分屏命令
        Logical_Control_Sts_t Send_ShowFrontLRView_Sts : 1;//发送显示左前+右前命令
        Logical_Control_Sts_t Send_ShowRearLRView_Sts : 1;//发送显示左后+右后命令
        Logical_Control_Sts_t Send_ShowLeftView_Sts : 1;//发送显示左视命令
        Logical_Control_Sts_t Send_ShowFrontView_Sts :  1;//发送显示前视命令
        Logical_Control_Sts_t Send_ShowFrontZoomOutView_Sts :  1;//发送显示前放大命令
        Logical_Control_Sts_t Send_ShowRearZoomOutView_Sts :  1;//发送显示后放大命令
        Logical_Control_Sts_t Send_SteeringInfo_Sts :  1;//发送转向信息
        Logical_Control_Sts_t Send_FrontFullScreenView_Sts : 1;//发送前视全屏命令
        Logical_Control_Sts_t Send_RearFullScreenView_Sts : 1;//手动显示标志
        Logical_Control_Sts_t Send_ShowGblFullScreenView_Sts : 1;//发送显示全景全屏命令
        Logical_Control_Sts_t Send_3DFullScreenView_Sts : 1;//发送3D全屏命令
        Logical_Control_Sts_t Send_TimeInfo_Sts : 1;//发送时间信息命令
        Logical_Control_Sts_t Send_TSInfo_Sts : 1;//发送TS信息命令
				
		Logical_Control_Sts_t ReservedBits : 3;//保留位
				
		Logical_Control_Sts_t Send_Sts : 1;//Need to send Flag
        
    }flag_b;
}External_Device_Interaction_Indicator_Bit_t;

/*
车身信息结构体类型定义
*/
/*
车身基本信息
*/
typedef union _vehicle_basic_info_union
{
    u8_t basicInfos;//车身基本信息

    struct 
    {
        u8_t ACC_Sts : 2;//bit0-1: ACC状态，0-钥匙拔出，1-ACC off，2-ACC，3-ACC on
        u8_t ILL_Sts : 1;//bit2: ILL状态，0-关闭，1-开启
        u8_t FBK_Sts : 1;//bit3:脚刹状态，0-正常，1-刹车
        u8_t GER_Sts : 4;//bit4-7: 档位状态，0-P，1-R，2-N，3-D
    }basicInfos_b;
}Vehicle_Basic_Info_Bit_t;
/*
车灯油门信息
*/
typedef union _vehicle_light_accelerator_info_union
{
    u8_t lightAndAcceleratorInfos;//车灯油门信息

    struct 
    {
        u8_t leftLight_Sts : 1;//bit0: 左转向灯状态，0-正常，1-开启
        u8_t rightLight_Sts : 1;//bit1: 右转向灯状态，0-正常，1-开启
        u8_t DFLight_Sts : 1;//bit2: 双闪灯状态，0-正常，1-开启
        u8_t HBLight_Sts : 1;//bit3: 远光灯状态，0-正常，1-开启

        u8_t Accelerator_Sts : 4;//bit4-7: 油门状态,0-正常，0x01-0x0F-踩下（由轻到重）
    }lightAcceleratorInfo_b;
}Vehicle_Light_Accelerator_Info_Bit_t;
/*
车速
*/
typedef u8_t Vehicle_Speed_Info_t;
/*
方向盘转角
*/
typedef u8_t Vehicle_SteeringAngle_Info_t;
/*
雷达状态信息
*/
typedef union _vehicle_radar_info_union
{
    u8_t radarInfos;//车灯油门信息

    struct 
    {
        u8_t LR_Sts : 2;//bit0-1: 左雷达状态，0-最远，不显示，1-较远，绿色，2-较近，黄色，3-最近，红色
        u8_t LCR_Sts : 2;//bit2-3: 左中雷达状态，0-最远，不显示，1-较远，绿色，2-较近，黄色，3-最近，红色
        u8_t RR_Sts : 2;//bit4-5: 右雷达状态，0-最远，不显示，1-较远，绿色，2-较近，黄色，3-最近，红色
        u8_t RCR_Sts : 2;//bit6-7: 右中雷达状态，0-最远，不显示，1-较远，绿色，2-较近，黄色，3-最近，红色
    }radarInfo_b;
}Vehicle_Radar_Info_Bit_t;
/*
车门/P键状态信息
*/
typedef union _vehicle_door_p_info_union
{
    u8_t doorPInfos;

    struct 
    {
        u8_t LFD_Sts : 1;//bit0: 左前门状态，0-关闭，1-打开
        u8_t RFD_Sts : 1;//bit1: 右前门状态，0-关闭，1-打开
        u8_t LRD_Sts : 1;//bit2: 左后门状态，0-关闭，1-打开
        u8_t RRD_Sts : 1;//bit3: 右后门状态，0-关闭，1-打开
        
        u8_t BBX_Sts : 1;//bit4：后备箱状态，0-关闭，1-打开
        u8_t HOOD_Sts: 1;//bit5: 引擎盖状态，0-关闭，1-打开
        u8_t PKEY_Sts: 1;//bit6: P键状态，0-off，1-on
        u8_t Reserved: 1;//bit7： 保留
    }doorPInfo_b;
}Vehicle_Door_P_Info_t;
/*
发动机转速信息
*/
typedef u8_t Vehicle_Engine_Speed_t;
/*
车身信息
*/
typedef struct _vehicle_info_struct
{
    Vehicle_Basic_Info_Bit_t basicInfo;
    Vehicle_Light_Accelerator_Info_Bit_t lightAcceleratorInfo;
    Vehicle_Speed_Info_t speedInfo;
    Vehicle_SteeringAngle_Info_t steeringAngleInfo;
    Vehicle_Radar_Info_Bit_t frontRadarInfo;
    Vehicle_Radar_Info_Bit_t rearRadarInfo;
    Vehicle_Door_P_Info_t doorPInfo;
    Vehicle_Engine_Speed_t engineSpeedInfo;
}Vehicle_Info_t;

/*
视频通道类型定义
*/
typedef enum _video_channel_enum
{
    Original_View = 0x00u,//原车界面
    
    Global_Front_View,//全景+前视
    Global_Rear_View,//全景+后视
    Global_Left_View,//全景+左视
    Global_Right_View,//全景+右视
    
    
    Four_Splits_View,//四分屏视图
    Left_Right_Front_View,//左前+右前
    Left_Right_Rear_View,//左后+右后
    Global_Front_ZoomOut_View,//全景+全景前放大
    Global_Rear_ZoomOut_View,//全景+全景后放大
    Front_FullScreen_View,//前视全屏
    Rear_FullScreen_View,//后视全屏
    Global_FullScreen_View,//全景全屏
    _3D_FullScreen_View,//3D全屏

    Surround_View,//开机旋转

    Invalid_View = 0xFF,//无效的视图
}Video_Channle_t;
/*
按键按下标志：0-无效，1-有效
*/
typedef u8_t Key_Flag_t;
#define Key_Pressed_Flag_Size (sizeof(Key_Flag_t)*8)
typedef union _key_pressed_status
{
    Key_Flag_t keyFlags;

    struct
    {
        Key_Flag_t ListKey_Pressed_Flag : 1;//bit0-list键按下标志
        Key_Flag_t AnyKey_Pressed_Flag : 1;//bit1-任意键按下标志
        Key_Flag_t ReservedBits : (Key_Pressed_Flag_Size - 2);//保留位
    }keyFlags_b;
}Key_Pressed_Flag_t;
/*
车灯标志：0-关闭，1-开启
*/
typedef u8_t Lamp_Flag_t;
#define Lamp_Flag_Size (sizeof(Lamp_Flag_t)*8)
typedef union _lamp_status
{
    Lamp_Flag_t lampFlags;

    struct
    {
        Lamp_Flag_t Lamp1_Flag : 1;//bit0-小灯标志
        Lamp_Flag_t ReservedBits : (Key_Pressed_Flag_Size - 1);//保留位
    }LampFlags_b;
}Lamp_Sts_Flag_t;
/*
亮度信息
*/
typedef u8_t Brightness_t;
/*
时间信息
*/
typedef struct _vehicle_time_t
{
    u8_t year;
    u8_t month;
    u8_t day;

    u8_t hour;
    u8_t minute;
    u8_t second;
}Vehicle_Time_t;
/*
坐标类型定义：16-bit，0x0000--0x0FFF
*/
typedef struct _coordinate_value_struct
{
    u8_t high;
    u8_t low;
}Coordinate_Value_t;
/*
触摸屏状态
*/
typedef enum _ts_oprt_enum
{
    Non_Pressed = 0x00u,
    Short_Pressed = 0x01u,
    Long_Pressed = 0x02u,
}TS_Oprt_Status_t;
/*
触摸屏操作
*/
typedef struct _touch_screen_oprt_struct
{
    Coordinate_Value_t x;
    Coordinate_Value_t y;
    TS_Oprt_Status_t status;
}TS_Oprt_t;
/*
CAN驱动层处理结果结构体类型定义
*/
typedef struct _CAN_Driver_signal_strcut
{
    Vehicle_Info_t *vehicleInfo;
    Key_Pressed_Flag_t *keyFlag;
    Lamp_Sts_Flag_t *lampFlag;
    Brightness_t *brightness;
    Vehicle_Time_t *vehicleTime;
    TS_Oprt_t *ts_oprt;
}CAN_Driver_Signal_t;
/*
CAN信号处理结果结构体类型定义
*/
typedef struct _CAN_signal_strcut
{
    Vehicle_Info_t *vehicleInfo;
    CAN_Signal_Flag_t *flag;
    Vehicle_Time_t *vehicleTime;
    TS_Oprt_t *ts_oprt;
}CAN_Signal_t;
/*
外部设备处理模块indicator类型定义
*/
typedef struct _external_device_process_indicator_strcut
{
    /*
    待机时间，单位是s，0表示一直显示
    */
    u8_t standby_time;
    /*
    车速阈值，超过一定速度要关闭全景，如果是0，则不关闭全景
    */
    u8_t vehicleSpeed_threshold;
    Vehicle_Info_t *vehicleInfo;
    Vehicle_Time_t *vehicleTime;
    External_Device_Interaction_Indicator_Bit_t *indicator_bits;
    TS_Oprt_t *ts_oprt;
}External_Device_Indicator_t;
/*
握手结果
*/
typedef enum _handshake_result_enum
{
    Handshake_Unfinished = 0,
    Handshake_OK = 1,
    Handshake_Failure = 2,
}Handshake_Result_t;
/*
360设备菜单设置信息结构体
*/
typedef union _360_menu_setting_union
{
    u8_t settings[6];

    struct 
    {
        u8_t Poweron_Startup_Sts : 1;//开机环绕使能
        u8_t Steering_Open_Sts : 1;//转向开启
        u8_t Radar_Obstacle_Open_Sts : 1;//雷达障碍物启动
        u8_t Reserved_Bits1 : 5;//保留位

        u8_t Standby_Time : 8;//待机时间
        u8_t vehicleSpeed_threshold : 8;//车速阈值
        u8_t Reserved_Bits2 : 8;//保留位
        u8_t Reserved_Bits3 : 8;//保留位
        u8_t Reserved_Bits4 : 8;//保留位
    }settings_b;
}Menu_Setting_t;
/*
NVM中存储的信息结构体
*/
typedef struct _nvm_info_struct
{
    Menu_Setting_t *menuSetting;
}NVM_Info_t;

typedef enum _canbox_mode_enum
{
    Original_Mode,//原车模式
    _360_Canbox_Mode,//加装360模式
}CANBox_Mode_t;


typedef u32_t (*Transmitter)(u8_t *pbuf, u8_t length);
typedef void (*Processer)(void);
typedef CAN_Signal_t* (*GetCANSignal)(void);
typedef External_Device_Indicator_t* (*GetIndicator)(void);
typedef bool_t (*Register)(void* dev);
typedef Handshake_Result_t (*Handshaker)(void);
typedef void (*Callback)(void *arg);
typedef void (*Initizalizator)(void);
typedef bool_t (*Handler)(void);
typedef bool_t (*DeviceProcesser)(External_Device_Indicator_t *flag);
typedef CANBox_Mode_t (*CanBoxModeGetter)(void);
#endif