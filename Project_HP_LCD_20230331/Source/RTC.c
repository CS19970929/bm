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
	PWR_BackupAccessCmd(ENABLE); // �������RTC
	RCC_LSEConfig(RCC_LSE_ON);	 // ʹ���ⲿLSE����RCC_LSE_Bypass��·����˼Ӧ����ʹ�����LSEʱ�ӣ����ǵ�Ƭ�����ã���Χ��·��?
	do
	{
		HSEStatus = RCC_GetFlagStatus(RCC_FLAG_LSERDY);
		StartUpCounter++;
	} while ((HSEStatus == RESET) && (StartUpCounter < LSE_START_TIMEOUT)); // �ȴ��� LSE Ԥ��

	if (StartUpCounter < LSE_START_TIMEOUT)
	{
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // ��RTC ʱ��Դ����ΪLSE
		RCC_RTCCLKCmd(ENABLE);					// ʹ��RTCʱ��
		RTC_WaitForSynchro();					// �ȴ� RTC APB �Ĵ���ͬ��
		// ��Ƶ���ã�Flsi/((AsynchPrediv+1)(SynchPrediv+1)) = 1Hz = 1s
		RTC_InitStructure.RTC_AsynchPrediv = 99; // �첽��Ƶ��0x7F����������Խ��͹���
		RTC_InitStructure.RTC_SynchPrediv = 327; // ͬ����Ƶ��LSE = 32768Hz
		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
		result = 0;
	}
	else
	{
		++RTC_Faultcnt;		// RTC������ΪLSE����
		RCC_LSICmd(ENABLE); // ʹ�� LSI ��
		while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
			;									 // �ȴ��� LSI Ԥ��
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);	 // ��RTC ʱ��Դ����ΪLSI
		RCC_RTCCLKCmd(ENABLE);					 // ʹ��RTCʱ��
		RTC_WaitForSynchro();					 // �ȴ� RTC APB �Ĵ���ͬ��
		RTC_InitStructure.RTC_AsynchPrediv = 99; // �첽��Ƶ��0x7F����������Խ��͹���
		RTC_InitStructure.RTC_SynchPrediv = 399; // ͬ����Ƶ��LSI = 40KHz
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
	RTC_TimeStructure.RTC_H12 = RTC_H12_AM; // 24Сʱ��
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
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;							   // 0011 0001����ؼĴ�����24-31λ
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date; // ѡ��0-31�죬����������
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;			   // 0-31���Alarm A ��Ӱ��
	RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);			   // ��������⣬��datasheetӦ����BCD��ʽ�����ǿ⺯��ҪBIN���ܱ�ΪBCD
																	// RTC_Alarm_A����Ҫ�в��У���֪����ʲô��
	// RTC_AlarmShow();
	RTC_ITConfig(RTC_IT_ALRA, ENABLE); // Enable the RTC Alarm A Interrupt
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE); // Enable the alarm	A

	EXTI_ClearITPendingBit(EXTI_Line17); // �ⲿ�������жϣ��ܴ�standbyģʽ����
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

// ����˼��һ����������Ƿ�����Ż���Ȼ��RTC�ж�֮��ĸ��
void InitRTC(void)
{

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // ʹ��PWR����ʱ�ӣ�����ģʽ��RTC�����Ź�
	RTC_ClockConfig();									// RTCʱ������

	if (RTC_ReadBackupRegister(RTC_BKP_DR0) != BKP_VALUE)
	{ // ��ȡ���������ֵ�Ƿ�д����
		if (0 == RTC_TimeConfig())
		{
			RTC_WriteBackupRegister(RTC_BKP_DR0, BKP_VALUE);
		}
		else
		{
			// ����ʧ�ܲ���
		}
		RTC_AlarmConfig();
	}
	else
	{ // ������λ���Ҫ��
		if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
		{ // ����ɶ
			//("\r\n Power On Reset occurred....\n\r");
			//++RTC_Faultcnt;
		}
		else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
		{ // ����ɶ
			// printf("\r\n External Reset occurred....\n\r");
		}
		RTC_ClearFlag(RTC_FLAG_ALRAF); // Clear the RTC Alarm Flag
		EXTI_ClearITPendingBit(EXTI_Line17);
		// RTC_TimeShow();				//Display the RTC Time and Alarm�����������õ�
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
