1. Appl为应用层代码，不需要修改
2. Middle为中间层代码；需要修改：将Stream_Device_If.h中包含的头文件UART_If_Mock.h替换为你实际的UART驱动的头文件
	将各个宏定义替换为你实际定义的UART驱动函数的名字，如果没有，则直接将该宏定义为空白即可
3. 在5ms定时中断中调用timeMgr_Instance的Callback_5ms回调函数， 具体的调用方式可以借鉴main.c中对logical_control的使用
4. Drv为驱动层，需要用你的具体实现替换，文件名和函数名最好保持不变，否则你还要修改应用层的代码
	CAN_If_Mock为CAN的驱动实现
	NVM_Mock_If为EEPROM驱动实现
	UART_IF_Mock为UART驱动实现