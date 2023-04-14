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

// 一般不需要另立函数的IO口处理

//#define MCUO_DEBUG_LED1     	(PORT_OUT_GPIOB->bit2)       	//LED
void InitIO(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // 开启GPIOA的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // 开启GPIOB的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // 开启GPIOC的外设时钟
	
	//RCC_AHBPeriphClockCmd(RCC_ALL_KEY, ENABLE);
	

	GPIO_WriteBit(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_6, Bit_SET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);			//低电平亮

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
	SysTick->CTRL &= ~(1 << 2);			// 使用外部时钟
	fac_us = SystemCoreClock / 8000000; // SysTick时钟是SYSCLK 8分频，即SysTick时钟频率=SYSCLK/8，1us要计的个数为还得/1MHz
	fac_ms = (INT16)fac_us * 1000;		// 每个ms需要的systick时钟数
}

// 使用LSI
void Init_IWDG(void)
{

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 使能PWR外设时钟，待机模式，RTC，看门狗
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);		// 打开独立看门狗寄存器操作权限
	IWDG_SetPrescaler(IWDG_Prescaler_64);				// 预分频系数
	IWDG_SetReload(250);									// 设置重载计数值，k = Xms / (1 / (40KHz/64)) = X/64*40; 4096最高
						// 800——1.28s，80——128ms
	IWDG_ReloadCounter(); // 喂狗
	IWDG_Enable();		  // 使能IWDG
}

// 没用，实际就是PA0上拉输入模式
void InitWKUP_Key(void)
{

	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 使能PWR外设时钟，待机模式，RTC，看门狗
	// PA0_WKUP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // 选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 设置引脚模式为上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 设置中断线0，EXTI0和PA0挂钩
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	// 配置PA0_WKUP外部上升沿中断
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising; // 上升沿中断
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	// 中断嵌套设计
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn; // 使能按键WK_UP所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00; // 抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	   // 使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);
}

// 030所有时钟均为向上计数
void InitTimer(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);		//时钟3使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	// 定时器初始化
	TIM_TimeBaseStructure.TIM_Period = 500;						// 设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = 48;					// 设置用来作为TIMx时钟频率除数的预分频值——计数分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 设置时钟分割:TDTS = Tck_tim——时钟分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);			// 根据指定的参数初始化TIMx的时间基数单位
	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);					// 使能指定中断,允许更新中断

	/*中断嵌套设计*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0; // 抢占优先级0级，没响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM17, ENABLE); // 使能TIMx
}

void __delay_us(UINT32 nus)
{
	UINT32 temp;
	SysTick->LOAD = nus * fac_us;			  // 时间加载
	SysTick->VAL = 0x00;					  // 清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // 开始倒数
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16))); // 等待时间到达
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;		// 关闭计数器
	SysTick->VAL = 0X00;							// 清空计数器
}

// 这个是非中断方式的延时，倘若使用中断式延时，在中断中使用延时会出现中断嵌套问题，很容易出错
void __delay_ms(UINT16 ms)
{
	// UINT32 temp;
	// SysTick->LOAD = (UINT32)ms * fac_ms;	  // 时间加载(SysTick->LOAD为24bit)
	// SysTick->VAL = 0x00;					  // 清空计数器
	// SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // 开始倒数
	// do
	// {
	// 	temp = SysTick->CTRL;
	// } while (temp & 0x01 && !(temp & (1 << 16))); // 等待时间到达

	// SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // 关闭计数器
	// SysTick->VAL = 0X00;					   // 清空计数器

	bsp_DelayMS(ms);
}

void IWDG_Feed(void)
{

	IWDG_ReloadCounter();
}

void Sys_Standby(void)
{
	PWR_WakeUpPinCmd(PWR_WakeUpPin_1, ENABLE); // 使能唤醒管脚功能，PWR_CSR。
											   // 该引脚会被强制配置为下拉输入，意味着不需要配置了？
	// PWR_BackupAccessCmd(ENABLE);				//PWR_CR。复位后，RTC 和后备寄存器允许写入位

	PWR_ClearFlag(PWR_FLAG_WU); // Clear WUF bit in Power Control/Status register (PWR_CSR)
								// 清PWR_CR相关便能清除PWR_CSR
	PWR_EnterSTANDBYMode(); // 进入待命（STANDBY）模式，PWR_CR    _PDDS
							// SCB->SCR设置为SLEEPDEEP = 1
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
	if (s_u16Cnt10ms != g_u810msClockCnt) // 10ms定时标志
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
		g_st_SysTimeFlag.bits.b1Sys20msFlag = 1; // 20ms定时标志
	}

	g_st_SysTimeFlag.bits.b1Sys200msFlag = 0; // 屏蔽掉能使通讯异常
	g_st_SysTimeFlag.bits.b1Sys200ms1Flag = 0;
	if (s_u16Cnt200ms2 == 20)
	{
		s_u16Cnt200ms2 = 0;
		g_st_SysTimeFlag.bits.b1Sys200ms1Flag = 1; // 200ms定时标志
	}
	else if (s_u16Cnt200ms == 20)
	{
		s_u16Cnt200ms = 0;
		//MCUO_LED_MOD2 = ~MCUO_LED_MOD2;
		g_st_SysTimeFlag.bits.b1Sys200msFlag = 1; // 200ms定时标志

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

// 外部中断0服务程序，没用
void EXTI0_1_IRQHandler(void)
{
	// delay_ms(10);//消抖
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		MasterSlave_Select = ~MasterSlave_Select; // 两个通讯模式切换
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

// // 定时器3中断服务程序
// void TIM17_IRQHandler(void) // TIM3中断
// {
// 	if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET)
// 	{												 // 检查TIM3更新中断发生与否
// 		TIM_ClearITPendingBit(TIM17, TIM_IT_Update); // 清除TIMx更新中断标志
		
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
