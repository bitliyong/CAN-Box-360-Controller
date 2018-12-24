#include "_360_Device.h"
#include "eeprom.h"

STATIC bool_t _360_Device_Init(void);
STATIC Handshake_Result_t _360_Device_Handshake(void);
STATIC bool_t _360_Device_Process(External_Device_Indicator_t *idr);
STATIC void _360_Device_Receive_Process(void);

u8_t buffer[16];
Stream_Device_t *streamDevice;
TimeMgr_t *_360_Device_TimeMgr_Instance;
Timer_ID_t timerID;
/*
外部设备状态
*/
enum _external_device_state
{
    Handshake_State = 0u,
    Process_State = 1u,
}device_state;
/*
外部设备握手状态机
*/
enum _handshake_state
{
    RecvHandshakeMsg = 0u,
    SendHandshakeAck = 1u,
    CheckHandshake = 2u,
    HandshakeTimeout = 3u,
    HandshakeFailure = 4u,
}handshake_state_machine;
/*
握手失败标志
*/
bool_t doHandshakeFlag;
/*
外部设备Process状态机
*/
enum _process_state
{
    SendProcessMsg = 0,
    ReceiveProcessAck = 1,
    CheckProcessAck = 2,
    ProcessTimeout = 3,
    ProcessFailure = 4,
    ProcessIdle = 5,
}process_state_machine;


//外部设备交互处理指示
External_Device_Indicator_t *indicator;

//360外部设备交互数据帧格式定义
union _360_frame_union
{
    u8_t frame[MAX_SEND_FRAME_LENGTH + 4];

    struct
    {
        u8_t head_code;
        u8_t data_type;
        u8_t data_length;
        u8_t frameData[MAX_SEND_FRAME_LENGTH];
        u8_t checkSum;
        u8_t data_count;//用于记录已经收到了多少个字节，当data_count==data_length时，数据域接收完成
    }frame_byte;
}_360_send_frame, _360_recv_frame, _360_recv_frame_tmp;
struct _360_ack_struct
{
    u8_t ackCode;
}_360_recv_ack, _360_send_ack;
/* 
接收数据帧的状态机
*/
enum _360_recv_frame_state{
    Head_Code_Sta = 0u,
    Data_Type_Sta,
    Data_Length,
    Data_Sta,
    CheckSum_Sta,
}_360_recv_state;

/*
接收消息类型
*/
typedef enum _360_msg_type_enum
{
    ACK_Type = 0u,
    Frame_Type = 1u,
}_360_Msg_Type_t;
/*
接收消息结构体封装
*/
typedef struct _360_msg_struct
{
    _360_Msg_Type_t msgType;//消息类型
    bool_t validFlag;//有效标志
    void *msg;//消息内容
}_360_Msg_t;

/*
定义消息队列-目前已知的有两个消息（360设备的应答和360设备主动发送的数据）
目前不考虑消息堆积的问题，假定所有的消息都会在收齐下一帧消息之前处理完毕
*/
#define RECV_MSG_COUNT      (2u)
_360_Msg_t recvMsgs[RECV_MSG_COUNT];

External_Device_t _360_device = 
{
    "360_Device",
    0x2E,
    0x00,
    0x00,
    buffer,

    _360_Device_Init,
    _360_Device_Handshake,
    _360_Device_Process,
};

STATIC void _360_Device_Timeout_Callback(void* arg)
{
    if(Handshake_State == device_state)
    {
        //握手超时
        handshake_state_machine = HandshakeTimeout;
    }
    else if(Process_State == device_state)
    {
        //Process超时
        process_state_machine = ProcessTimeout;
    }
}
STATIC u8_t _360_Device_Calculate_Checksum(union _360_frame_union* frame)
{
    u8_t checkSum = 0x00;
    u8_t i = 0;

    checkSum += frame->frame_byte.data_type;
    checkSum += frame->frame_byte.data_length;
    
    for(; i < frame->frame_byte.data_length; i++)
    {
        checkSum += frame->frame_byte.frameData[i];
    }
    
    checkSum ^= 0xFF;

    return checkSum;
}
STATIC void _360_Device_Frame_Init(void)
{
    //发送数据帧
    _360_send_frame.frame_byte.head_code = 0x2E;
    _360_send_frame.frame_byte.data_type = 0x00;
    _360_send_frame.frame_byte.data_length = 0x00;
    _360_send_frame.frame_byte.checkSum = 0x00;

    //接收数据帧
    #if (CHECK_ACK_ENABLE)
    _360_recv_ack.ackCode = 0x00;
    #else
    _360_recv_ack.ackCode = NORMAL_ACK;
    #endif
    _360_send_ack.ackCode = 0x00;
}
STATIC bool_t _360_Device_Init(void)
{
    //获取流设备，作为实际的数据传输媒介
    streamDevice = Stream_Device_Instance();
    //创建超时检测定时器
    _360_Device_TimeMgr_Instance = TimeMgr_Instance();
    timerID = _360_Device_TimeMgr_Instance->CreateTimer(200, Timer_Once, _360_Device_Timeout_Callback, Ptr_NULL);
	
		//初始化流设备
	  streamDevice->Initialization();

    //初始化交互数据帧
    _360_Device_Frame_Init();

    //初始化接收状态机
    _360_recv_state = Head_Code_Sta;

    //初始化接收消息队列
    recvMsgs[0].msgType = ACK_Type;
    #if (CHECK_ACK_ENABLE)
    recvMsgs[0].validFlag = FALSE;
    #else
    recvMsgs[0].validFlag = TRUE;
    #endif
    recvMsgs[0].msg = (void*)(&_360_recv_ack);
    recvMsgs[1].msgType = Frame_Type;
    recvMsgs[1].validFlag = FALSE;
    recvMsgs[1].msg = (void*)(&_360_recv_frame);

    //初始化握手状态机
    handshake_state_machine = RecvHandshakeMsg;
    //初始化交互状态机
    process_state_machine = ProcessIdle;
}
STATIC void _360_Device_SendACK(struct _360_ack_struct *ack)
{
    streamDevice->Write(&(ack->ackCode), 1);
}
STATIC union _360_frame_union* _360_Device_ReceiveProcessFrameMessage(void)
{
    union _360_frame_union *frame = Ptr_NULL;

    //如果ack消息有效，返回ACK消息的指针
    if(TRUE == recvMsgs[1].validFlag)
    {
        frame = (union _360_frame_union*)recvMsgs[1].msg;
        recvMsgs[1].validFlag = FALSE;
    }

    return frame;
}
STATIC bool_t _360_Device_CheckProcessFrameMessage(union _360_frame_union *pmsg)
{
    return (_360_Device_Calculate_Checksum(pmsg) == pmsg->frame_byte.checkSum);
}
STATIC struct _360_ack_struct* _360_Device_ReceiveProcessAckMessage(void)
{
    struct _360_ack_struct *ack = Ptr_NULL;

    //如果ack消息有效，返回ACK消息的指针
    if(TRUE == recvMsgs[0].validFlag)
    {
        ack = (struct _360_ack_struct*)recvMsgs[0].msg;

        #if (CHECK_ACK_ENABLE)
        recvMsgs[0].validFlag = FALSE;
        #endif
    }

    return ack;
}
STATIC bool_t _360_Device_CheckProcessAckMessage(struct _360_ack_struct *pmsg)
{
    return (NORMAL_ACK == pmsg->ackCode);
}
STATIC union _360_frame_union* _360_Device_RecvHandshakeMessage(void)
{
    return _360_Device_ReceiveProcessFrameMessage();
}
STATIC void _360_Device_SendHandshakeAckMessage(void)
{
    _360_send_ack.ackCode = NORMAL_ACK;

    _360_Device_SendACK(&_360_send_ack);
}
STATIC bool_t _360_Device_CheckHandshakeMessage(union _360_frame_union *pmsg)
{
    return _360_Device_CheckProcessFrameMessage(pmsg);
}
STATIC Handshake_Result_t _360_Device_Handshake(void)
{
    Handshake_Result_t handshakeResult = Handshake_OK;

    #if (HAND_SHAKE_ENABLE == STD_ON)
    u8_t failureCounter = 0;    
    union _360_frame_union *pmsg = Ptr_NULL;

    handshakeResult = Handshake_Unfinished;

    //设备处于handshake状态
    device_state = Handshake_State;
    
    //设置握手执行标志
    doHandshakeFlag = TRUE;

    //开启定时器
    _360_Device_TimeMgr_Instance->StartTimer(timerID);

    while(TRUE == doHandshakeFlag)
    {
        doHandshakeFlag = FALSE;
        
        //从流设备读取数据
        _360_Device_Receive_Process();

        //handshake状态机轮转
        switch(handshake_state_machine)
        {
            case RecvHandshakeMsg:
                pmsg = _360_Device_RecvHandshakeMessage();

                if(pmsg != Ptr_NULL)
                {
                    //收到了握手，停定时器
                    _360_Device_TimeMgr_Instance->StopTimer(timerID);

                    handshake_state_machine = CheckHandshake;
                }                
                else
                {
                    break;
                }                
            case CheckHandshake:
                if(TRUE == _360_Device_CheckHandshakeMessage(pmsg))
                {
                    handshake_state_machine = SendHandshakeAck;
                }
                else
                {
                    handshake_state_machine = HandshakeFailure;

                    break;
                }            
            case SendHandshakeAck:
                _360_Device_SendHandshakeAckMessage();
                
                indicator->indicator_bits->flag_b.Handshake_OK_Sts = 1u;

                handshakeResult = Handshake_OK;
            break;            
            case HandshakeTimeout:
                //超时，将状态机切换到握手失败
                handshake_state_machine = HandshakeFailure;
            break;
            case HandshakeFailure:
                //failureCounter++,
                failureCounter++;

                //如果达到指定错误次数，说明handshake失败
                if(MAX_FAILURE_COUNT == failureCounter)
                {
                    handshakeResult = Handshake_Failure;
                    failureCounter = 0u;
                }
                else
                {
                    handshake_state_machine = RecvHandshakeMsg;
                }
            break;
            default:
            break;
        }
    }
    #else
    //indicator->indicator_bits->flag_b.Handshake_OK_Sts = 1u;
    #endif

    return handshakeResult;
}
STATIC void _360_Device_Send_Version_Info(void)
{
    //发送版本信息
    _360_send_frame.frame_byte.data_type = VERSION_INFO;
    _360_send_frame.frame_byte.data_length = VERSION_INFO_LENGTH;
    _360_send_frame.frame_byte.frameData[0] = 0x00;
    _360_send_frame.frame_byte.frameData[1] = 0x00;
    _360_send_frame.frame_byte.frameData[2] = 0x00;
    _360_send_frame.frame_byte.frameData[3] = 0x00;
    _360_send_frame.frame_byte.frameData[4] = 0x00;
    _360_send_frame.frame_byte.frameData[5] = 0x00;
    _360_send_frame.frame_byte.frameData[6] = 0x00;
    _360_send_frame.frame_byte.frameData[7] = 0x00;
    _360_send_frame.frame_byte.frameData[8] = 0x00;
    _360_send_frame.frame_byte.frameData[9] = 0x00;
    _360_send_frame.frame_byte.frameData[10] = 0x00;
    _360_send_frame.frame_byte.frameData[11] = 0x00;
    _360_send_frame.frame_byte.frameData[12] = 0x00;
    _360_send_frame.frame_byte.frameData[13] = 0x00;
    _360_send_frame.frame_byte.frameData[14] = 0x00;
    _360_send_frame.frame_byte.frameData[15] = 0x00;
    _360_send_frame.frame_byte.checkSum = _360_Device_Calculate_Checksum(&_360_send_frame);

    streamDevice->Write(_360_send_frame.frame, _360_send_frame.frame_byte.data_length + 4);
}
STATIC void _360_Device_Send_TS_CP(void)
{
    //发送触摸屏操作
    _360_send_frame.frame_byte.data_type = TS_INFO;
    _360_send_frame.frame_byte.data_length = TS_INFO_LENGTH;
    _360_send_frame.frame_byte.frameData[0] = indicator->ts_oprt->x.high;
    _360_send_frame.frame_byte.frameData[1] = indicator->ts_oprt->x.low;
    _360_send_frame.frame_byte.frameData[2] = indicator->ts_oprt->y.high;
    _360_send_frame.frame_byte.frameData[3] = indicator->ts_oprt->y.low;
    _360_send_frame.frame_byte.frameData[4] = indicator->ts_oprt->status;
    _360_send_frame.frame_byte.frameData[5] = _360_send_frame.frame_byte.checkSum = _360_Device_Calculate_Checksum(&_360_send_frame);

    streamDevice->Write(_360_send_frame.frame, _360_send_frame.frame_byte.data_length + 4);
}
STATIC void _360_Device_Send_Vehicle_Adapter(void)
{
    //发送车辆适配信息
    _360_send_frame.frame_byte.data_type = VEHICLE_ADAPTER;
    _360_send_frame.frame_byte.data_length = VEHICLE_ADAPTER_LENGTH;
    _360_send_frame.frame_byte.frameData[0] = 0x00;
    _360_send_frame.frame_byte.frameData[1] = 0x00;
    _360_send_frame.frame_byte.frameData[2] = _360_send_frame.frame_byte.checkSum = _360_Device_Calculate_Checksum(&_360_send_frame);

    streamDevice->Write(_360_send_frame.frame, _360_send_frame.frame_byte.data_length + 4);
}
STATIC void _360_Device_Send_Action_Info(void)
{
    //发送版本信息
    _360_send_frame.frame_byte.data_type = ACTION_INFO;
    _360_send_frame.frame_byte.data_length = ACTION_INFO_LENGTH;
    _360_send_frame.frame_byte.frameData[0] = 0x00;
    _360_send_frame.frame_byte.frameData[1] = 0x00;
    _360_send_frame.frame_byte.frameData[2] = _360_send_frame.frame_byte.checkSum = _360_Device_Calculate_Checksum(&_360_send_frame);

    streamDevice->Write(_360_send_frame.frame, _360_send_frame.frame_byte.data_length + 4);
}
STATIC void _360_Device_Update_Vehicle_Info_To_Send(void)
{
    _360_send_frame.frame_byte.data_type = VEHICLE_INFO;
    _360_send_frame.frame_byte.data_length = VEHICLE_INFO_LENGTH;
    _360_send_frame.frame_byte.frameData[0] = indicator->vehicleInfo->basicInfo.basicInfos;//基本信息
    _360_send_frame.frame_byte.frameData[1] = indicator->vehicleInfo->lightAcceleratorInfo.lightAndAcceleratorInfos;//灯光油门信息
    _360_send_frame.frame_byte.frameData[2] = indicator->vehicleInfo->speedInfo;//车速信息
    _360_send_frame.frame_byte.frameData[3] = indicator->vehicleInfo->steeringAngleInfo;//方向盘转角信息
    _360_send_frame.frame_byte.frameData[4] = indicator->vehicleInfo->frontRadarInfo.radarInfos;//前雷达信息
    _360_send_frame.frame_byte.frameData[5] = indicator->vehicleInfo->rearRadarInfo.radarInfos;//后雷达信息
    _360_send_frame.frame_byte.frameData[6] = indicator->vehicleInfo->doorPInfo.doorPInfos;//车门P键信息
    _360_send_frame.frame_byte.frameData[7] = indicator->vehicleInfo->engineSpeedInfo;//发送机转速信息
    _360_send_frame.frame_byte.frameData[8] =_360_send_frame.frame_byte.checkSum = _360_Device_Calculate_Checksum(&_360_send_frame);
}
STATIC void _360_Device_Update_Video_Channel_To_Send(Video_Channle_t view)
{
    _360_send_frame.frame_byte.data_type = VIDEO_CHANNEL;
    _360_send_frame.frame_byte.data_length = VIDEO_CHANNEL_LENGTH;
    _360_send_frame.frame_byte.frameData[0] = view;//视频通道
    _360_send_frame.frame_byte.frameData[1] =_360_send_frame.frame_byte.checkSum = _360_Device_Calculate_Checksum(&_360_send_frame);
}
STATIC void _360_Device_SendReversingMessage(void)
{
    //倒车视图
    _360_Device_Update_Video_Channel_To_Send(Global_Rear_View);
}
STATIC void _360_Device_Send3DFullScreenViewMessage(void)
{
    //3D全屏视图
    _360_Device_Update_Video_Channel_To_Send(_3D_FullScreen_View);
}
STATIC void _360_Device_SendFrontFullScreenViewMessage(void)
{
    //前视全屏视图
    _360_Device_Update_Video_Channel_To_Send(Front_FullScreen_View);
}
STATIC void _360_Device_SendGearInfoMessage(void)
{
    //填写车身信息
    _360_Device_Update_Vehicle_Info_To_Send();
}
STATIC void _360_Device_SendSteeringInfoMessage(void)
{
    //填写车身信息
    _360_Device_Update_Vehicle_Info_To_Send();
}
STATIC void _360_Device_SendYawRateInfoMessage(void)
{
    //填写车身信息
    _360_Device_Update_Vehicle_Info_To_Send();
}
STATIC void _360_Device_SendOriginalViewMessage(void)
{
    //原车视图
    _360_Device_Update_Video_Channel_To_Send(Original_View);
}
STATIC void _360_Device_SendPowerOnSurroundMessage(void)
{
    //上电环绕视图
    _360_Device_Update_Video_Channel_To_Send(Surround_View);
}
STATIC void _360_Device_SendRearFullScreenViewMessage(void)
{
    //后视全屏视图
    _360_Device_Update_Video_Channel_To_Send(Rear_FullScreen_View);
}
STATIC void _360_Device_SendFrontLRViewMessage(void)
{
    //左前右前视图
    _360_Device_Update_Video_Channel_To_Send(Left_Right_Front_View);
}
STATIC void _360_Device_SendFrontViewMessage(void)
{
    //前视图
    _360_Device_Update_Video_Channel_To_Send(Global_Front_View);
}
STATIC void _360_Device_SendFrontZoomOutMessage(void)
{
    //前放大视图
    _360_Device_Update_Video_Channel_To_Send(Global_Front_ZoomOut_View);
}
STATIC void _360_Device_SendGblFullScreenMessage(void)
{
    //全景全屏视图
    _360_Device_Update_Video_Channel_To_Send(Global_FullScreen_View);
}
STATIC void _360_Device_SendLeftViewMessage(void)
{
    //全景左视图
    _360_Device_Update_Video_Channel_To_Send(Global_Left_View);
}
STATIC void _360_Device_SendRearLRMessage(void)
{
    //左右后视图
    _360_Device_Update_Video_Channel_To_Send(Left_Right_Rear_View);
}
STATIC void _360_Device_SendRearZoomOutMessage(void)
{
    //后放大视图
    _360_Device_Update_Video_Channel_To_Send(Global_Rear_ZoomOut_View);
}
STATIC void _360_Device_SendRightViewMessage(void)
{
    //全景右视图
    _360_Device_Update_Video_Channel_To_Send(Global_Right_View);
}
STATIC void _360_Device_SendSplitViewMessage(void)
{
    //4分屏视图
    _360_Device_Update_Video_Channel_To_Send(Four_Splits_View);
}

STATIC void _360_Device_SendTimeInfoMessage(void)
{
    _360_send_frame.frame_byte.data_type = TIME_INFO;
    _360_send_frame.frame_byte.data_length = TIME_INFO_LENGTH;
    _360_send_frame.frame_byte.frameData[0] = indicator->vehicleTime->year;//年
    _360_send_frame.frame_byte.frameData[1] = indicator->vehicleTime->month;//月
    _360_send_frame.frame_byte.frameData[2] = indicator->vehicleTime->day;//日
    _360_send_frame.frame_byte.frameData[3] = indicator->vehicleTime->hour;//时
    _360_send_frame.frame_byte.frameData[4] = indicator->vehicleTime->minute;//分
    _360_send_frame.frame_byte.frameData[5] = indicator->vehicleTime->second;//秒
    _360_send_frame.frame_byte.frameData[6] =_360_send_frame.frame_byte.checkSum = _360_Device_Calculate_Checksum(&_360_send_frame);
}
STATIC void _360_Device_UpdateSendMessageProcess(void)
{
	if((indicator->indicator_bits->flag & 0x07FFFF00u) != 0u)
	{
		indicator->indicator_bits->flag_b.Send_Sts = 1u;
	}

    //依次判断哪些命令需要发送
    if(1u == indicator->indicator_bits->flag_b.Send_Reversing_Sts)
    {
        _360_Device_SendReversingMessage();
			
				indicator->indicator_bits->flag_b.Send_Reversing_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_3DFullScreenView_Sts)
    {
        _360_Device_Send3DFullScreenViewMessage();
			
			  indicator->indicator_bits->flag_b.Send_3DFullScreenView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_FrontFullScreenView_Sts)
    {
        _360_Device_SendFrontFullScreenViewMessage();
			
			  indicator->indicator_bits->flag_b.Send_FrontFullScreenView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_GearInfo_Sts)
    {
        _360_Device_SendGearInfoMessage();
			
				indicator->indicator_bits->flag_b.Send_GearInfo_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_OriginalView_Sts)
    {
        _360_Device_SendOriginalViewMessage();
			
				indicator->indicator_bits->flag_b.Send_OriginalView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_PowerOnSurround_Sts)
    {
        _360_Device_SendPowerOnSurroundMessage();
			
				indicator->indicator_bits->flag_b.Send_PowerOnSurround_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_RearFullScreenView_Sts)
    {
        _360_Device_SendRearFullScreenViewMessage();
			
				indicator->indicator_bits->flag_b.Send_RearFullScreenView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_ShowFrontLRView_Sts)
    {
        _360_Device_SendFrontLRViewMessage();
			
				indicator->indicator_bits->flag_b.Send_ShowFrontLRView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_ShowFrontView_Sts)
    {
        _360_Device_SendFrontViewMessage();
			
				indicator->indicator_bits->flag_b.Send_ShowFrontView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_ShowFrontZoomOutView_Sts)
    {
        _360_Device_SendFrontZoomOutMessage();
			
				indicator->indicator_bits->flag_b.Send_ShowFrontZoomOutView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_ShowGblFullScreenView_Sts)
    {
        _360_Device_SendGblFullScreenMessage();
			
				indicator->indicator_bits->flag_b.Send_ShowGblFullScreenView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_ShowLeftView_Sts)
    {
        _360_Device_SendLeftViewMessage();
			
				indicator->indicator_bits->flag_b.Send_ShowLeftView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_ShowRearLRView_Sts)
    {
        _360_Device_SendRearLRMessage();
			
				indicator->indicator_bits->flag_b.Send_ShowRearLRView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_ShowRearZoomOutView_Sts)
    {
        _360_Device_SendRearZoomOutMessage();
			
				indicator->indicator_bits->flag_b.Send_ShowRearZoomOutView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_ShowRightView_Sts)
    {
        _360_Device_SendRightViewMessage();
			
				indicator->indicator_bits->flag_b.Send_ShowRightView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_ShowSplitView_Sts)
    {
        _360_Device_SendSplitViewMessage();
			
				indicator->indicator_bits->flag_b.Send_ShowSplitView_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_SteeringInfo_Sts)
    {
        _360_Device_SendSteeringInfoMessage();
			
				indicator->indicator_bits->flag_b.Send_SteeringInfo_Sts = 0u;
    }
    else if(1u == indicator->indicator_bits->flag_b.Send_YawRateInfo_Sts)
    {
        _360_Device_SendYawRateInfoMessage();
			
				indicator->indicator_bits->flag_b.Send_YawRateInfo_Sts = 0u;
    }
    //时间信息
    else if(indicator->indicator_bits->flag_b.Send_TimeInfo_Sts)
    {
        _360_Device_SendTimeInfoMessage();
			
				indicator->indicator_bits->flag_b.Send_TimeInfo_Sts = 0u;
    }
    //TS信息
    else if(indicator->indicator_bits->flag_b.Send_TSInfo_Sts)
    {
        _360_Device_Send_TS_CP();
			
				indicator->indicator_bits->flag_b.Send_TSInfo_Sts = 0u;
    }
}
STATIC void _360_Device_SendMessage(void)
{
    //发送数据
    streamDevice->Write(_360_send_frame.frame, _360_send_frame.frame_byte.data_length + 4);
}
STATIC void _360_Device_RestoreData(u8_t *src, u8_t *des, u8_t length)
{
    while(length-- > 0)
    {
        *des++ = *src++;
    }
}
STATIC void _360_Device_Receive_Process(void)
{
    static u8_t tmp[16u];
    static u8_t offset = 0u;
    u8_t index = 0u;
    u8_t count = 0u;
    u8_t totalCount = 0u;
    u8_t data = 0u;

    //从流设备获取数据
    count = streamDevice->Read((u8_t*)(tmp+offset), 1);

    //总字节数
    totalCount = offset + count;

    while(index < totalCount)
    {
        data = tmp[index++];

        switch(_360_recv_state)
        {
            case Head_Code_Sta:
                //首先判断是不是ACK
                if(IS_ACK(data))
                {
                    _360_recv_ack.ackCode = data;

                    //ACK消息有效！！！
                    recvMsgs[0].validFlag = TRUE;
                }
                //是合法的数据头
                else if(data == _360_device.classCode)
                {
                    _360_recv_frame_tmp.frame_byte.head_code = data;
                    _360_recv_state = Data_Type_Sta;
                }
                //非法数据
                else
                {
                    //
                }
            break;
            case Data_Type_Sta:
                _360_recv_frame_tmp.frame_byte.data_type = data;
                _360_recv_state = Data_Length;
            break;
            case Data_Length:
                _360_recv_frame_tmp.frame_byte.data_length = data;
                _360_recv_frame_tmp.frame_byte.data_count = 0;
                _360_recv_state = Data_Sta;
            break;
            case Data_Sta:
                _360_recv_frame_tmp.frame_byte.frameData[_360_recv_frame_tmp.frame_byte.data_count++] = data;

                if(_360_recv_frame_tmp.frame_byte.data_count >= _360_recv_frame_tmp.frame_byte.data_length)
                {
                    _360_recv_state = CheckSum_Sta;
                }
            break;
            case CheckSum_Sta:
                _360_recv_frame_tmp.frame_byte.checkSum = data;
                
                //复制数据
                _360_Device_RestoreData(_360_recv_frame_tmp.frame, _360_recv_frame.frame, _360_recv_frame_tmp.frame_byte.data_length + 3);
                
                _360_recv_frame.frame_byte.checkSum = _360_recv_frame_tmp.frame_byte.checkSum;
                //将消息设置为有效，可以被处理
                recvMsgs[1].validFlag = TRUE;

                _360_recv_state = Head_Code_Sta;
            break;
            default:

            break;
        }
    }
}
STATIC void _360_Device_RecvdChannelAckFrame_Process(union _360_frame_union *pframe)
{
    //TODO
}
STATIC void _360_Device_RecvdProjectModeFrame_Process(union _360_frame_union *pframe)
{
    indicator->indicator_bits->flag_b.Project_Mode_Sts = pframe->frame_byte.frameData[0];
}
STATIC void _360_Device_RecvdTimeQueryFrame_Process(union _360_frame_union *pframe)
{
    indicator->indicator_bits->flag_b.Send_TimeInfo_Sts = 1;
}
STATIC void _360_Device_RecvdMenuSettingFrame_Process(union _360_frame_union *pframe)
{
    indicator->indicator_bits->flag_b.PowerOn_Startup_Sts = POWERON_STARTUP_FLAG(pframe->frame_byte.frameData[0]);
		SaveFullViewPowerOnSelectionEE(indicator->indicator_bits->flag_b.PowerOn_Startup_Sts);
    indicator->indicator_bits->flag_b.Steering_AutoOpen_GblView_Enable_Sts = STEERING_OPEN_FLAG(pframe->frame_byte.frameData[0]);
		SaveFullViewTurnSelectionEE(indicator->indicator_bits->flag_b.Steering_AutoOpen_GblView_Enable_Sts);
    indicator->indicator_bits->flag_b.Radar_Obstacle_Startup_Sts = RADAR_OBSTACLE_OPEN_FLAG(pframe->frame_byte.frameData[0]);
		SaveFullViewRadarSelectionEE(indicator->indicator_bits->flag_b.Radar_Obstacle_Startup_Sts);
    indicator->standby_time = pframe->frame_byte.frameData[1];
    indicator->vehicleSpeed_threshold = pframe->frame_byte.frameData[2];
		SaveFullViewSpeedSelectionEE(indicator->vehicleSpeed_threshold);
}
STATIC void _360_Device_RecvdFrame_Process(union _360_frame_union *pframe)
{
    //校验
    if(_360_Device_Calculate_Checksum(pframe) == pframe->frame_byte.checkSum)
    {
        //回复ack
        _360_send_ack.ackCode = NORMAL_ACK;
        _360_Device_SendACK(&_360_send_ack);

        switch(pframe->frame_byte.data_type)
        {
            case HAND_SHAKE:
                
            break;
            case CHANNEL_ACK:
                _360_Device_RecvdChannelAckFrame_Process(pframe);
            break;
            case TIME_QUERY:
                //发送时间信息
                _360_Device_RecvdTimeQueryFrame_Process(pframe);
            break;
            case PROJ_MODE:
                _360_Device_RecvdProjectModeFrame_Process(pframe);
            break;
            case MENU_SETTING_INFO:
                _360_Device_RecvdMenuSettingFrame_Process(pframe);
            break;
        }
    }
}
STATIC bool_t _360_Device_Process(External_Device_Indicator_t *idr)
{
    u8_t failureCounter = 0;
    bool_t processResult = TRUE;
    struct _360_ack_struct *pAck = Ptr_NULL;
    union _360_frame_union *pframe = Ptr_NULL;
    bool_t doProcessFlag = TRUE;

    indicator = idr;

    //设备处在Process状态
    device_state = Process_State;

    //刷新要发送的命令
    _360_Device_UpdateSendMessageProcess();

    while(TRUE == doProcessFlag)
    {
        //从流设备读取数据
        _360_Device_Receive_Process();

        //看是不是收到了360设备的数据帧
        pframe = _360_Device_ReceiveProcessFrameMessage();

        if(Ptr_NULL != pframe)
        {
            _360_Device_RecvdFrame_Process(pframe);
        }

        //handshake状态机轮转
        switch(process_state_machine)
        {
            case SendProcessMsg:
                _360_Device_SendMessage();

                //开启定时器
                _360_Device_TimeMgr_Instance->StartTimer(timerID);

                process_state_machine = ReceiveProcessAck;
            break;
            case ReceiveProcessAck:
                pAck = _360_Device_ReceiveProcessAckMessage();

                if(pAck != Ptr_NULL)
                {
                    //关闭定时器
                    _360_Device_TimeMgr_Instance->StopTimer(timerID);

                    process_state_machine = CheckProcessAck;
                }
                else
                {
                    break;
                }            
            case CheckProcessAck:
                if(TRUE == _360_Device_CheckProcessAckMessage(pAck))
                {
                    //收到了应答，上一操作完成
                    indicator->indicator_bits->flag_b.Last_Operation_Done_Sts = TRUE;
                    indicator->indicator_bits->flag_b.Send_Sts = 0u;
                    
                    processResult = TRUE;

                    process_state_machine = ProcessIdle;
									
					doProcessFlag = FALSE;
                }
                else
                {
                    process_state_machine = ProcessFailure;
                }
            break;
            case ProcessTimeout:
                //超时，将状态机切换到握手失败
                process_state_machine = ProcessFailure;
            break;
            case ProcessFailure:
                failureCounter++;

                //如果达到指定错误次数，说明handshake失败
                if(MAX_FAILURE_COUNT == failureCounter)
                {
                    doProcessFlag = FALSE;
                    processResult = FALSE;
                }
                else
                {
                    process_state_machine = ProcessIdle;
                }
            break;
            case ProcessIdle:
                //有标志被置起来，转入消息发送
                if(indicator->indicator_bits->flag_b.Send_Sts != 0)
                {
                    process_state_machine = SendProcessMsg;
                }
                else
                {
                    doProcessFlag = FALSE;
                }
            break;
            default:

            break;
        }
    }

    return processResult;
}
