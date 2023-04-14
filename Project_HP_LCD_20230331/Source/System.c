#include "main.h"
#include "bsp.h"

volatile union SYS_TIME g_st_SysTimeFlag;
volatile union On_OFF_func On_OFF_funcflag;

static INT8 fac_us = 0;	 // us
static INT16 fac_ms = 0; // ms
UINT8 g_u81msCnt = 0;
UINT8 g_u810msClockCnt = 0;
UINT8 g_u81msClockCnt = 0;

UINT8 MasterSlave_Select = 0;

// һ�㲻��Ҫ����������IO�ڴ���

//#define MCUO_DEBUG_LED1     	(PORT_OUT_GPIOB->bit2)       	//LED
void InitIO(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // ����GPIOA������ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // ����GPIOB������ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // ����GPIOC������ʱ��
	
	//RCC_AHBPeriphClockCmd(RCC_ALL_KEY, ENABLE);
	

	GPIO_WriteBit(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_6, Bit_SET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);			//�͵�ƽ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void InitDelay(void)
{
	SysTick->CTRL &= ~(1 << 2);			// ʹ���ⲿʱ��
	fac_us = SystemCoreClock / 8000000; // SysTickʱ����SYSCLK 8��Ƶ����SysTickʱ��Ƶ��=SYSCLK/8��1usҪ�Ƶĸ���Ϊ����/1MHz
	fac_ms = (INT16)fac_us * 1000;		// ÿ��ms��Ҫ��systickʱ����
}

// ʹ��LSI
void Init_IWDG(void)
{

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // ʹ��PWR����ʱ�ӣ�����ģʽ��RTC�����Ź�
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);		// �򿪶������Ź��Ĵ�������Ȩ��
	IWDG_SetPrescaler(IWDG_Prescaler_64);				// Ԥ��Ƶϵ��
	IWDG_SetReload(250);									// �������ؼ���ֵ��k = Xms / (1 / (40KHz/64)) = X/64*40; 4096���
						// 800����1.28s��80����128ms
	IWDG_ReloadCounter(); // ι��
	IWDG_Enable();		  // ʹ��IWDG
}

// û�ã�ʵ�ʾ���PA0��������ģʽ
void InitWKUP_Key(void)
{

	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // ʹ��PWR����ʱ�ӣ�����ģʽ��RTC�����Ź�
	// PA0_WKUP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // ѡ��Ҫ�õ�GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ��������ģʽΪ��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// �����ж���0��EXTI0��PA0�ҹ�
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	// ����PA0_WKUP�ⲿ�������ж�
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising; // �������ж�
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	// �ж�Ƕ�����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn; // ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00; // ��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	   // ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);
}

// 030����ʱ�Ӿ�Ϊ���ϼ���
void InitTimer(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);		//ʱ��3ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	// ��ʱ����ʼ��
	TIM_TimeBaseStructure.TIM_Period = 500;						// ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 48;					// ����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ����������Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// ����ʱ�ӷָ�:TDTS = Tck_tim����ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);			// ����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);					// ʹ��ָ���ж�,��������ж�

	/*�ж�Ƕ�����*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0; // ��ռ���ȼ�0����û��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM17, ENABLE); // ʹ��TIMx
}

void __delay_us(UINT32 nus)
{
	UINT32 temp;
	SysTick->LOAD = nus * fac_us;			  // ʱ�����
	SysTick->VAL = 0x00;					  // ��ռ�����
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // ��ʼ����
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16))); // �ȴ�ʱ�䵽��
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;		// �رռ�����
	SysTick->VAL = 0X00;							// ��ռ�����
}

// ����Ƿ��жϷ�ʽ����ʱ������ʹ���ж�ʽ��ʱ�����ж���ʹ����ʱ������ж�Ƕ�����⣬�����׳���
void __delay_ms(UINT16 ms)
{
	// UINT32 temp;
	// SysTick->LOAD = (UINT32)ms * fac_ms;	  // ʱ�����(SysTick->LOADΪ24bit)
	// SysTick->VAL = 0x00;					  // ��ռ�����
	// SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // ��ʼ����
	// do
	// {
	// 	temp = SysTick->CTRL;
	// } while (temp & 0x01 && !(temp & (1 << 16))); // �ȴ�ʱ�䵽��

	// SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // �رռ�����
	// SysTick->VAL = 0X00;					   // ��ռ�����

	bsp_DelayMS(ms);
}

void IWDG_Feed(void)
{

	IWDG_ReloadCounter();
}

void Sys_Standby(void)
{
	PWR_WakeUpPinCmd(PWR_WakeUpPin_1, ENABLE); // ʹ�ܻ��ѹܽŹ��ܣ�PWR_CSR��
											   // �����Żᱻǿ������Ϊ�������룬��ζ�Ų���Ҫ�����ˣ�
	// PWR_BackupAccessCmd(ENABLE);				//PWR_CR����λ��RTC �ͺ󱸼Ĵ�������д��λ

	PWR_ClearFlag(PWR_FLAG_WU); // Clear WUF bit in Power Control/Status register (PWR_CSR)
								// ��PWR_CR��ر������PWR_CSR
	PWR_EnterSTANDBYMode(); // ���������STANDBY��ģʽ��PWR_CR    _PDDS
							// SCB->SCR����ΪSLEEPDEEP = 1
}

void App_SysTime(void)
{
	static UINT8 u8LEDcnt = 0;
	static UINT16 s_u16Cnt10ms = 0;
	static UINT16 s_u16Cnt20ms = 0;
	static UINT16 s_u16Cnt200ms = 0;
	static UINT16 s_u16Cnt200ms2 = 10;

	g_st_SysTimeFlag.bits.b1Sys10msFlag = 0;
	g_st_SysTimeFlag.bits.b1Sys10ms1Flag = 0;
	g_st_SysTimeFlag.bits.b1Sys10ms2Flag = 0;
	g_st_SysTimeFlag.bits.b1Sys10ms3Flag = 0;
	g_st_SysTimeFlag.bits.b1Sys10ms4Flag = 0;
	if (s_u16Cnt10ms != g_u810msClockCnt) // 10ms��ʱ��־
	{
		s_u16Cnt10ms = g_u810msClockCnt;
		switch (g_u810msClockCnt)
		{
		case 0:
			MCUO_LED_MOD1 = ~MCUO_LED_MOD1;
			s_u16Cnt20ms++;
			g_st_SysTimeFlag.bits.b1Sys10msFlag = 1;
			break;

		case 1:
			s_u16Cnt200ms++;
			g_st_SysTimeFlag.bits.b1Sys10ms1Flag = 1;
			break;

		case 2:
			s_u16Cnt200ms2++;
			g_st_SysTimeFlag.bits.b1Sys10ms2Flag = 1;
			break;

		case 3:
			g_st_SysTimeFlag.bits.b1Sys10ms3Flag = 1;
			break;

		case 4:
			g_st_SysTimeFlag.bits.b1Sys10ms4Flag = 1;
			break;

		default:
			break;
		}
	}

	g_st_SysTimeFlag.bits.b1Sys20msFlag = 0;
	if (s_u16Cnt20ms >= 2)
	{
		s_u16Cnt20ms = 0;
		g_st_SysTimeFlag.bits.b1Sys20msFlag = 1; // 20ms��ʱ��־
	}

	g_st_SysTimeFlag.bits.b1Sys200msFlag = 0; // ���ε���ʹͨѶ�쳣
	g_st_SysTimeFlag.bits.b1Sys200ms1Flag = 0;
	if (s_u16Cnt200ms2 == 20)
	{
		s_u16Cnt200ms2 = 0;
		g_st_SysTimeFlag.bits.b1Sys200ms1Flag = 1; // 200ms��ʱ��־
	}
	else if (s_u16Cnt200ms == 20)
	{
		s_u16Cnt200ms = 0;
		//MCUO_LED_MOD2 = ~MCUO_LED_MOD2;
		g_st_SysTimeFlag.bits.b1Sys200msFlag = 1; // 200ms��ʱ��־

		//MCUO_DEBUG_LED1 = ~MCUO_DEBUG_LED1;
	}

	if (1 == g_st_SysTimeFlag.bits.b1Sys200msFlag)
	{
		++u8LEDcnt;
		if (u8LEDcnt >= 5)
		{
			u8LEDcnt = 0;
		}
	}
}

// �ⲿ�ж�0�������û��
void EXTI0_1_IRQHandler(void)
{
	// delay_ms(10);//����
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		MasterSlave_Select = ~MasterSlave_Select; // ����ͨѶģʽ�л�
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

// // ��ʱ��3�жϷ������
// void TIM17_IRQHandler(void) // TIM3�ж�
// {
// 	if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET)
// 	{												 // ���TIM3�����жϷ������
// 		TIM_ClearITPendingBit(TIM17, TIM_IT_Update); // ���TIMx�����жϱ�־
		
// 		if ((++g_u81msCnt) >= 2)					 // 1ms
// 		{
// 			g_u81msCnt = 0;
			
// 			g_u81msClockCnt++;

// 			//TimCnt();

// 			if (g_u81msClockCnt >= 2) // 2ms
// 			{
// 				g_u81msClockCnt = 0;

// 				g_u810msClockCnt++;

// 				if (g_u810msClockCnt >= 5) // 10ms
// 				{
// 					g_u810msClockCnt = 0;
// 				}
// 			}
// 		}
// 	}
// }
