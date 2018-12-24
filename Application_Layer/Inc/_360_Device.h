#ifndef __360_DEVICE__
#define __360_DEVICE__

#include "External_Device_Descriptor.h"
#include "Stream_Device_If.h"
#include "TimeMgr.h"

#ifndef STATIC
#define STATIC              static
#endif

//最大失败次数
#define MAX_FAILURE_COUNT           (3)
//最大发送帧长度
#define MAX_SEND_FRAME_LENGTH       (16)
//数据类型
//HOST-->SLAVE
#define HAND_SHAKE                  (0x80)
#define CHANNEL_ACK                 (0x81)
#define TIME_QUERY                  (0x82)
#define PROJ_MODE                   (0x83)
#define MENU_SETTING_INFO           (0x84)
//SLAVE-->HOST
#define VERSION_INFO                (0x00)
#define VIDEO_CHANNEL               (0x01)
#define TS_INFO                     (0x02)
#define VEHICLE_INFO                (0x03)
#define VEHICLE_ADAPTER             (0x04)
#define ACTION_INFO                 (0x05)
#define TIME_INFO                   (0x06)
//数据长度
#define VERSION_INFO_LENGTH         (0x10)
#define VIDEO_CHANNEL_LENGTH        (0x01)
#define TS_INFO_LENGTH              (0x05)
#define VEHICLE_INFO_LENGTH         (0x08)
#define VEHICLE_ADAPTER_LENGTH      (0x02)
#define ACTION_INFO_LENGTH          (0x02)
#define TIME_INFO_LENGTH            (0x06)
//应答码
#define NORMAL_ACK                  (0xFF)
#define CHECKSUM_ERR                (0xF0)
#define NOT_SUPPORT_ERR             (0xF3)
#define BUSY_ERR                    (0xFC)
//开机启动标志
#define POWERON_STARTUP_FLAG(x)     (((x)&0x01) == 0? 0 : 1)
#define STEERING_OPEN_FLAG(x)       (((x)&0x02) == 0? 0 : 1)
#define RADAR_OBSTACLE_OPEN_FLAG(x) (((x)&0x04) == 0? 0 : 1)
//判断是不是ACK帧
#define IS_ACK(x)                   ((NORMAL_ACK == (x)) || (CHECKSUM_ERR == (x)) || (NOT_SUPPORT_ERR == (x)) || (BUSY_ERR == (x)))

extern External_Device_t _360_device;

#endif