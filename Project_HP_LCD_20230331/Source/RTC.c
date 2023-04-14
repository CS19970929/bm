#include "main.h"

struct RTC_ELEMENT RTC_element;

RTC_InitTypeDef RTC_InitStructure;
RTC_TimeTypeDef RTC_TimeStructure;
RTC_DateTypeDef RTC_DateStructure;
RTC_AlarmTypeDef RTC_AlarmStructure;

UINT8 RTC_Faultcnt = 0;

void App_RTC(void)
{
	static UINT8 u8RTCcnt = 0;
	if (1 == g_st_SysTimeFlag.bits.b1Sys200msFlag && u8RTCcnt < 5)
	{
		++u8RTCcnt;
		return;
	}
	u8RTCcnt = 0;
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	RTC_GetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);
}

INT8 RTC_ClockConfig(void)
{

	__IO UINT16 StartUpCounter = 0, HSEStatus = 0;
	INT8 result = 0;
	PWR_BackupAccessCmd(ENABLE); // 允许访问RTC
	RCC_LSEConfig(RCC_LSE_ON);	 // 使能外部LSE晶振，RCC_LSE_Bypass旁路的意思应该是使能这个LSE时钟，但是单片机不用，外围电路用?
	do
	{
		HSEStatus = RCC_GetFlagStatus(RCC_FLAG_LSERDY);
		StartUpCounter++;
	} while ((HSEStatus == RESET) && (StartUpCounter < LSE_START_TIMEOUT)); // 等待到 LSE 预备

	if (StartUpCounter < LSE_START_TIMEOUT)
	{
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // 把RTC 时钟源配置为LSE
		RCC_RTCCLKCmd(ENABLE);					// 使能RTC时钟
		RTC_WaitForSynchro();					// 等待 RTC APB 寄存器同步
		// 分频设置，Flsi/((AsynchPrediv+1)(SynchPrediv+1)) = 1Hz = 1s
		RTC_InitStructure.RTC_AsynchPrediv = 99; // 异步分频，0x7F最大，拉满可以降低功耗
		RTC_InitStructure.RTC_SynchPrediv = 327; // 同步分频，LSE = 32768Hz
		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
		result = 0;
	}
	else
	{
		++RTC_Faultcnt;		// RTC错误单数为LSE出错
		RCC_LSICmd(ENABLE); // 使能 LSI 振荡
		while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
			;									 // 等待到 LSI 预备
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);	 // 把RTC 时钟源配置为LSI
		RCC_RTCCLKCmd(ENABLE);					 // 使能RTC时钟
		RTC_WaitForSynchro();					 // 等待 RTC APB 寄存器同步
		RTC_InitStructure.RTC_AsynchPrediv = 99; // 异步分频，0x7F最大，拉满可以降低功耗
		RTC_InitStructure.RTC_SynchPrediv = 399; // 同步分频，LSI = 40KHz
		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
		result = -1;
	}

	if (RTC_Init(&RTC_InitStructure) == ERROR)
	{
		++RTC_Faultcnt;
		++RTC_Faultcnt;
		result = -1;
	}
	return result;
}

INT8 RTC_TimeConfig(void)
{
	INT8 result = 0;

	RTC_DateStructure.RTC_Year = RTC_element.RTC_Time_Year;
	RTC_DateStructure.RTC_Month = RTC_element.RTC_Time_Month;
	RTC_DateStructure.RTC_Date = RTC_element.RTC_Time_Day;
	RTC_TimeStructure.RTC_H12 = RTC_H12_AM; // 24小时制
	RTC_TimeStructure.RTC_Hours = RTC_element.RTC_Time_Hour;
	RTC_TimeStructure.RTC_Minutes = RTC_element.RTC_Time_Minute;
	RTC_TimeStructure.RTC_Seconds = RTC_element.RTC_Time_Second;

	if (RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) == ERROR ||
		RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure) == ERROR)
	{
		// printf("\n\r>> !! RTC Set Time failed. !! <<\n\r");
		++RTC_Faultcnt;
		++RTC_Faultcnt;
		result = -1;
	}
	return result;
}

void RTC_AlarmConfig(void)
{

	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RTC_AlarmCmd(RTC_Alarm_A, DISABLE); // Disable the Alarm A
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = RTC_element.RTC_Alarm_Hour;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = RTC_element.RTC_Alarm_Minute;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = RTC_element.RTC_Alarm_Second;
	// Set the Alarm A
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;							   // 0011 0001，相关寄存器的24-31位
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date; // 选择0-31天，不是星期数
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;			   // 0-31天对Alarm A 无影响
	RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);			   // 这个有问题，按datasheet应该是BCD格式，但是库函数要BIN才能变为BCD
																	// RTC_Alarm_A必须要有才行，不知道有什么用
	// RTC_AlarmShow();
	RTC_ITConfig(RTC_IT_ALRA, ENABLE); // Enable the RTC Alarm A Interrupt
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE); // Enable the alarm	A

	EXTI_ClearITPendingBit(EXTI_Line17); // 外部上升沿中断，能从standby模式唤醒
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn; // Enable the RTC Alarm Interrupt
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// 回来思考一下这个函数是否可以优化，然后RTC中断之类的搞搞
void InitRTC(void)
{

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 使能PWR外设时钟，待机模式，RTC，看门狗
	RTC_ClockConfig();									// RTC时钟配置

	if (RTC_ReadBackupRegister(RTC_BKP_DR0) != BKP_VALUE)
	{ // 读取备份里面的值是否被写过。
		if (0 == RTC_TimeConfig())
		{
			RTC_WriteBackupRegister(RTC_BKP_DR0, BKP_VALUE);
		}
		else
		{
			// 配置失败操作
		}
		RTC_AlarmConfig();
	}
	else
	{ // 以下这段话需要吗？
		if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
		{ // 这是啥
			//("\r\n Power On Reset occurred....\n\r");
			//++RTC_Faultcnt;
		}
		else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
		{ // 这是啥
			// printf("\r\n External Reset occurred....\n\r");
		}
		RTC_ClearFlag(RTC_FLAG_ALRAF); // Clear the RTC Alarm Flag
		EXTI_ClearITPendingBit(EXTI_Line17);
		// RTC_TimeShow();				//Display the RTC Time and Alarm，这个后面会用到
		// RTC_AlarmShow();
	}
}

void RTC_IRQHandler(void)
{
	if (RTC_GetITStatus(RTC_IT_ALRA) != RESET)
	{
		RTC_ClearITPendingBit(RTC_IT_ALRA);
	}
	EXTI_ClearITPendingBit(EXTI_Line17);
}
