#include "TimeMgr.h"

/*
声明函数
*/
STATIC void TimeMgr_Initialization(void);
STATIC Timer_ID_t TimeMgr_CreateTimer(u32_t period, Timer_Type_t type, Callback callback, void *arg);
STATIC void TimeMgr_DeleteTimer(Timer_ID_t timerID);
STATIC void TimeMgr_StartTimer(Timer_ID_t timerID);
STATIC void TimeMgr_StopTimer(Timer_ID_t timerID);
/*
Timer结构体定义
*/
typedef struct __timer_struct
{
    Timer_ID_t timerID;
    u32_t period;
    u32_t periodBackup;

    Timer_Type_t type;
    Callback callback;
    void *arg;
    bool_t activeFlag;
}Timer_Instance_t;

TimeMgr_t timeMgr_Instance = 
{
    TimeMgr_Initialization,
    TimeMgr_CreateTimer,
    TimeMgr_DeleteTimer,
    TimeMgr_StartTimer,
    TimeMgr_StopTimer,
    TimeMgr_Callback_5ms,
};

/*
定时器链表结构体定义
*/
typedef struct _timer_node
{
    struct _timer_node * next;
    Timer_Instance_t * timer;
}Timer_Node_t;

//定时器数组定义
Timer_Instance_t timers[TIMER_COUNT];
Timer_Node_t timer_nodes[TIMER_COUNT];

Timer_Node_t *freeTimerListHead;
Timer_Node_t *activeTimerListHead;

TimeMgr_t* TimeMgr_Instance(void)
{
    return &timeMgr_Instance;
}
/*
初始化节点，使每一个节点指向一个定时器
*/
STATIC void TimeMgr_TimerNode_Init(void)
{
    u8_t i = 0;

    for(; i < TIMER_COUNT; i++)
    {
        timer_nodes[i].next = Ptr_NULL;
        timer_nodes[i].timer = &timers[i];

        //分配timerID
        timers[i].timerID = i;
    }
}
/*
初始化可用定时器链表
*/
STATIC void TimeMgr_FreeTimerList_Init(void)
{
    u8_t i = 1;
    Timer_Node_t *ptimer = Ptr_NULL;

    freeTimerListHead = ptimer = &timer_nodes[0];

    for(; i < TIMER_COUNT; i++)
    {
        ptimer->next = &timer_nodes[i];
        ptimer = &timer_nodes[i];
    }
}

/*
正在使用的定时器的链表
*/
STATIC void TimeMgr_ActiveTimerList_Init(void)
{
    activeTimerListHead = Ptr_NULL;
}
/*
时间管理模块初始化
*/
STATIC void TimeMgr_Initialization(void)
{
    //初始化定时器节点数组
    TimeMgr_TimerNode_Init();

    //初始化可用定时器链表
    TimeMgr_FreeTimerList_Init();
    
    //初始化正在使用的定时器链表
    TimeMgr_ActiveTimerList_Init();
}
/*
将节点从可用链表移动到正在使用链表
*/
STATIC void TimeMgr_FromFree2Active(void)
{
    Timer_Node_t *ptimer = Ptr_NULL;

    //从链表头部取一个定时器
    ptimer = freeTimerListHead;

    //移除取出的节点
    freeTimerListHead = ptimer->next;
        
    //断开与当前链表的连接
    ptimer->next = Ptr_NULL;

    //将取出的节点添加到正在使用的定时器链表头部
    ptimer->next = activeTimerListHead;
    activeTimerListHead = ptimer;
}
/*
将节点从正在使用链表移动到可用链表
*/
STATIC void TimeMgr_FromActive2Free(Timer_Instance_t* timer)
{
    Timer_Node_t *ptimer = Ptr_NULL;
    Timer_Node_t *ppreTimer = Ptr_NULL; 

    //在正在使用的定时器链表中找出timer
    ptimer = activeTimerListHead;

    while(ptimer != Ptr_NULL)
    {
        //找到了
        if(ptimer->timer == timer)
        {
            break;
        }

        //继续往下找，并记录好上一个节点的位置
        ppreTimer = ptimer;
        ptimer = ptimer->next;
    }

    //确实找到了timer
    if(ptimer != Ptr_NULL)
    {
        //timer位于正在使用链表的表头位置
        if(ppreTimer == Ptr_NULL)
        {
            //更新表头的位置
            activeTimerListHead = ptimer->next;
        }
        else
        {
            //上一节点直接连接到下一节点
            ppreTimer->next = ptimer->next;
        }

        //将取出的节点添加到可用的定时器链表头部
        ptimer->next = freeTimerListHead;
        freeTimerListHead = ptimer;
    }
}
/*
创建定时器实例
*/
STATIC Timer_ID_t TimeMgr_CreateTimer(u32_t period, Timer_Type_t type, Callback callback, void *arg)
{
    u32_t period_tick = 0;
    Timer_Instance_t *ptimer = Ptr_NULL;

    //可用定时器链表中有定时器
    if(freeTimerListHead != Ptr_NULL)
    {
        //移动节点
        TimeMgr_FromFree2Active();

        //获取当前申请到的定时器对象
        ptimer = activeTimerListHead->timer;

        //计算定时节拍，四舍五入
        period_tick = (period + TIMER_TICK/2)/TIMER_TICK;

        //设置参数值
        ptimer->period = period_tick;
        ptimer->periodBackup = period_tick;
        ptimer->type = type;
        ptimer->callback = callback;
        ptimer->arg = arg;
        ptimer->activeFlag = FALSE;
    }

    return ptimer->timerID;
}
STATIC Timer_Instance_t* FindTimerByTimerID(Timer_ID_t timerID)
{
    Timer_Node_t *timer = activeTimerListHead;
    Timer_Node_t *ptimer = Ptr_NULL;

    while(timer != Ptr_NULL)
    {
        if(timerID == timer->timer->timerID)
        {
            ptimer = timer;

            break;
        }
        else
        {
            timer = timer->next;
        }
    }

    return ptimer->timer;
}
/*
删除定时器
*/
STATIC void TimeMgr_DeleteTimer(Timer_ID_t timerID)
{
    Timer_Instance_t *timer = Ptr_NULL;

    timer = FindTimerByTimerID(timerID);

    if(Ptr_NULL != timer)
    {
        //复位timer的参数
        timer->activeFlag = FALSE;
        timer->period = 0;
        timer->periodBackup = 0;
        timer->type = 0;
        timer->callback = Ptr_NULL;
        timer->arg = Ptr_NULL;

        //将定时器放回到可用定时器链表中
        TimeMgr_FromActive2Free(timer);
    }
}
/*
启动定时器
*/
STATIC void TimeMgr_StartTimer(Timer_ID_t timerID)
{
    Timer_Instance_t *timer = Ptr_NULL;

    timer = FindTimerByTimerID(timerID);

    if(Ptr_NULL != timer)
    {
        if(timer->activeFlag == FALSE)
        {
            timer->period = timer->periodBackup;
            timer->activeFlag = TRUE;
        }
    }
}
/*
停止定时器
*/
STATIC void TimeMgr_StopTimer(Timer_ID_t timerID)
{
    Timer_Instance_t *timer = Ptr_NULL;

    timer = FindTimerByTimerID(timerID);
    
    if(Ptr_NULL != timer)
    {
        timer->activeFlag = FALSE;
    }
}
void TimeMgr_Callback_5ms(void)
{
    Timer_Node_t *ptimer = activeTimerListHead;
    Timer_Instance_t *timer = Ptr_NULL;

    //遍历当前正在使用的定时器，并进行处理
    while(ptimer != Ptr_NULL)
    {
        timer = ptimer->timer;

        //当前定时器处于active状态
        if(TRUE == timer->activeFlag)
        {
            timer->period--;

            //定时时间到？
            if(0 == timer->period)
            {
                //回调函数不为空？
                if(timer->callback != Ptr_NULL)
                {
                    //调用回调函数，处理超时事件
                    timer->callback(timer->arg);

                    //定时器类型为Once？
                    if(Timer_Once == timer->type)
                    {
                        //定时器不再处于active状态
                        timer->activeFlag = FALSE;
                    }
                    //定时器类型为Repeat？
                    else if(Timer_Repeat == timer->type)
                    {
                        //reload period
                        timer->period = timer->periodBackup;
                    }
                    else
                    {
                        //TODO
                    }
                }
            }
        }

        //指向下一个
        ptimer = ptimer->next;
    }
}