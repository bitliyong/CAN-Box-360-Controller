#include "CAN_If_Mock.h"

CAN_Driver_Signal_t signal;
/*
Vehicle_Info_t *vehicleInfo;
    Key_Pressed_Flag_t *keyFlag;
    Lamp_Sts_Flag_t *lampFlag;
    Brightness_t *brightness;
    Vehicle_Time_t *vehicleTime;
    TS_Oprt_t *ts_oprt;
*/
Vehicle_Info_t vehicleInfo;
Key_Pressed_Flag_t keyFlag;
Lamp_Sts_Flag_t lampFlag;
Brightness_t brightness;
Vehicle_Time_t vehicleTime;
TS_Oprt_t ts_oprt;
extern u8_t Current_Time_Val[],keypushflag,keybackflag,Lamp_state_Flag,Dimmval,Can_LFDoor_State,Current_Steering_Val,Can_Scm_state,Gear_Val;
extern u16_t Vehicle_speed;
extern u8_t Radar_F_dis,Radar_R_dis,Jeacar_Touch_Val[],LftTrnLmpAtv,RtTrnLmpAtv;
/*
获取车身状态flag
*/
void CAN_Driver_Init(void)
{
    signal.vehicleInfo = &vehicleInfo;
    signal.keyFlag = &keyFlag;
    signal.lampFlag = &lampFlag;
    signal.brightness = &brightness;
    signal.vehicleTime = &vehicleTime;
    signal.ts_oprt = &ts_oprt;
}
/*
获取车身状态flag
*/
CAN_Driver_Signal_t* CAN_Driver_GetVehicleSignal(void)
{
    //这些变量均是为了表示赋值操作的示例而定义的
    Coordinate_Value_t X;
    Coordinate_Value_t Y;
    Vehicle_Basic_Info_Bit_t basicInfo;
    Vehicle_Door_P_Info_t doorPInfo;
    Vehicle_Engine_Speed_t engineSpeedInfo;
    Vehicle_Light_Accelerator_Info_Bit_t lightAcceleratorInfo;
    Vehicle_Radar_Info_Bit_t frontRadarInfo;
    Vehicle_Radar_Info_Bit_t rearRadarInfo;
    Vehicle_Speed_Info_t speedInfo;
    Vehicle_SteeringAngle_Info_t steeringAngleInfo;

    /****************************************************************************
     * 如下为对CAN驱动层传递过来的信号的整个集合的大结构体的各个字段进行赋值操作的示例，
     * 示例中的这些临时变量或具体数字在实际的程序中可以被一个全局变量或者其他类似的定义
     * 所替换，CAN驱动在收到这些数据时只要可以刷新对应的值即可。
     ***************************************************************************/
    //对车身时间结构体赋值示例
		vehicleTime.year = Current_Time_Val[2];//18u;
    vehicleTime.month = Current_Time_Val[3];//7u;
    vehicleTime.day = Current_Time_Val[4];//28u;
    vehicleTime.hour = Current_Time_Val[5];//21u;
    vehicleTime.minute = Current_Time_Val[6];//31u;
    vehicleTime.second = Current_Time_Val[7];//59u;
    //对按键标志结构体赋值示例
    keyFlag.keyFlags_b.ListKey_Pressed_Flag = keypushflag;//0u;//List键未按下，如果按下则赋值为1u
    keyFlag.keyFlags_b.AnyKey_Pressed_Flag = keybackflag;//0u;//任意键未按下，如果按下则赋值为1u
    //对车灯标志结构体赋值示例
    lampFlag.LampFlags_b.Lamp1_Flag = Lamp_state_Flag;//0u;//小灯未打开，如果打开则赋值为1u
    //对亮度进行赋值示例
    brightness = Dimmval;//0x80u;//赋值为解码器背光亮度的具体数值
    //对触摸屏操作结构体赋值示例
    X.high = Jeacar_Touch_Val[2];//0x35u;
    X.low = Jeacar_Touch_Val[3];//0x43u;
    Y.high = Jeacar_Touch_Val[4];//0x26u;
    Y.low = Jeacar_Touch_Val[5];//0x29u;
    ts_oprt.x = X;//设置触摸屏的x坐标为0x3543u
    ts_oprt.y = Y;//设置触摸屏的y坐标为0x2629u
    ts_oprt.status = Jeacar_Touch_Val[5];//Short_Pressed;//设置触摸屏的触摸状态为短按
    //对车身信息结构体赋值示例
    vehicleInfo.basicInfo.basicInfos_b.ACC_Sts = Can_Scm_state;//basicInfo; //basicInfo.
		if(Gear_Val == 0x0E)
			vehicleInfo.basicInfo.basicInfos_b.GER_Sts = 1;//basicInfo; //basicInfo.
		else if(Gear_Val == 0x0F)
			vehicleInfo.basicInfo.basicInfos_b.GER_Sts = 0;//basicInfo; //basicInfo.
		else
			vehicleInfo.basicInfo.basicInfos_b.GER_Sts = 3;
		vehicleInfo.basicInfo.basicInfos_b.ILL_Sts = Lamp_state_Flag;
		vehicleInfo.doorPInfo.doorPInfo_b.LFD_Sts = Can_LFDoor_State;//doorPInfo;
    //vehicleInfo.engineSpeedInfo = engineSpeedInfo;
		//frontRadarInfo = (Vehicle_Radar_Info_Bit_t)Radar_F_dis;//
    vehicleInfo.frontRadarInfo = frontRadarInfo;

    //先清空现有值
    vehicleInfo.lightAcceleratorInfo.lightAndAcceleratorInfos = 0u;

		if(LftTrnLmpAtv != 0  && RtTrnLmpAtv != 0)	//双闪
		{
			vehicleInfo.lightAcceleratorInfo.lightAcceleratorInfo_b.DFLight_Sts = 1;
		}
		else if(LftTrnLmpAtv)
		{
			vehicleInfo.lightAcceleratorInfo.lightAcceleratorInfo_b.leftLight_Sts = 1;
		}
		else if(RtTrnLmpAtv)
		{
			vehicleInfo.lightAcceleratorInfo.lightAcceleratorInfo_b.rightLight_Sts = 1;
		}
		else					//无转向
		{
		}
    
    //vehicleInfo.rearRadarInfo = Radar_R_dis;//rearRadarInfo;
    vehicleInfo.speedInfo = Vehicle_speed*0.01625;//speedInfo;
    vehicleInfo.steeringAngleInfo = Current_Steering_Val;//teeringAngleInfo;

    return &signal;
}