#ifndef __TIMEMGR__
#define __TIMEMGR__

#include "TypeDef_Std.h"
#include "TypeDef_User.h"

#ifndef STATIC
#define STATIC              static
#endif

//时间节拍-5ms
#define TIMER_TICK          (5)
//软件定时器个数-8
#define TIMER_COUNT         (16)

/*
定时器类型
*/
typedef enum __timer_type
{
    Timer_Once = 0,
    Timer_Repeat = 1,
}Timer_Type_t;

/*
定时器类型
*/
typedef u8_t Timer_ID_t;

typedef Timer_ID_t (*TimerCreator)(u32_t period, Timer_Type_t type, Callback callback, void *arg);
typedef void (*TimerDeletor)(Timer_ID_t timerID);
typedef void (*TimerActor)(Timer_ID_t timerID);

/*
TimeManager结构体定义
*/
typedef struct __time_mgr
{
    Initizalizator Initialization;
    TimerCreator CreateTimer;
    TimerDeletor DeleteTimer;
    TimerActor StartTimer;
    TimerActor StopTimer;
    Processer Callback_5ms;
}TimeMgr_t;
/*
接口函数
*/
TimeMgr_t* TimeMgr_Instance(void);
void TimeMgr_Callback_5ms(void);
#endif