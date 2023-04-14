#include "main.h"

void LCD_Show_Vatl(void);
void LCD_Show_SOC(void);

uint8_t g_u8Rs485FunCmd;
uint16_t g_u16Rs485RegAddr;

uint8_t g_u8DataRXBuff[RXBUFLEN];
uint8_t g_u8DataArry[TXBUFNUM]; // 发送给副边数据缓存				应该可以优化为局部变量

UINT8 g_u8Rs485State = RS485_STATE_IDLE;

UINT16 g_u16MasterSendComCnt1ms = 0;

uint8_t g_u8SciRxErrFlag = 0;
uint8_t g_u8ReceiveCnt = 0;

uint8_t g_u8ReceiveBeginFlag = 0;

struct structDelayFlag g_st_DelayFlag;
STATUS_Comm_TypeDef g_st_SysCommFlag;

struct LCD_DISPLAY g_stLcdDispData;

uint16_t SuspendFlag1 = 0;
uint16_t SuspendFlag2 = 0;
uint16_t voll = 0;

uint8_t Tab_Num1[12] = {0xF5, 0x05, 0xB6, 0x97, 0x47, 0xD3, 0xF3, 0x85, 0xF7, 0xD7, 0x00, 0xFF}; // 分别对应0~9，全灭，全亮
uint8_t Tab_Num2[12] = {0xFA, 0x0A, 0xD6, 0x9E, 0x2E, 0xBC, 0xFC, 0x1A, 0xFE, 0xBE, 0x00, 0xFF}; // 后三个数字对应段的排列与前三个高低交换

void InitUSART1_Master(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 开启USART1外设时钟
	// RCC->AHBENR |= 1<<17;										//开启GPIOA的外设时钟

	// Enable the USART1 Interrupt(使能USART1中断)
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// USART1_TX -> PA9 , USART1_RX -> PA10
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1); // 030的AF表格在非reg的datasheet里
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 串口初始化
	USART_InitStructure.USART_BaudRate = 19200;										// 设置串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 设置数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 设置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 设置效验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 设置流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 设置工作模式
	USART_Init(USART1, &USART_InitStructure);										// 配置入结构体

	USART1->CR3 |= 1 << 0;	// EIE，开帧错误中断，同时开启噪声中断
	USART1->CR3 |= 1 << 11; // 未被使能前改写，禁止噪声中断

	USART_Cmd(USART1, ENABLE);					   // 使能串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 使能接收中断
}

void InitUSART2_Master(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	// RCC->AHBENR |= 1<<17;										//开启GPIOA的外设时钟

	// Enable the USART2 Interrupt(使能USART2中断)
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// USART2_TX -> PA9 , USART2_RX -> PA3
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1); // 030的AF表格在非reg的datasheet里
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 串口初始化
	USART_InitStructure.USART_BaudRate = 19200;										// 设置串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 设置数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 设置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 设置效验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 设置流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 设置工作模式
	USART_Init(USART2, &USART_InitStructure);										// 配置入结构体

	USART2->CR3 |= 1 << 0;	// EIE，开帧错误中断，同时开启噪声中断
	USART2->CR3 |= 1 << 11; // 未被使能前改写，禁止噪声中断

	USART_Cmd(USART2, ENABLE);					   // 使能串口1
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 使能接收中断

	// Sci_DataInit(&g_stCurrentMsgPtr_SCI2);
}

void TimCnt(void)
{
	static UINT16 s_u16Delay200msCnt = 0;
	static UINT8 s_u8RecDelayCnt = 0;

	g_u16MasterSendComCnt1ms++;

	// 以下逻辑应该没用，可以优化掉
	if (g_st_DelayFlag.b1PowerStDelay == FALSE)
	{
		s_u16Delay200msCnt++;
		if (s_u16Delay200msCnt >= 200)
		{
			s_u16Delay200msCnt = 200; // why 200,会出问题吗？

			g_st_DelayFlag.b1PowerStDelay = TRUE;
		}
	}

	if (1 == g_u8ReceiveBeginFlag)
	{
		s_u8RecDelayCnt++;
		if (s_u8RecDelayCnt > SciReceiveTime) // SciReceiveTime	需不需要改大点
		{
			s_u8RecDelayCnt = 0;
			g_u8ReceiveBeginFlag = 0;

			g_st_SysCommFlag.bits.bReceiveFinishFlag = TRUE; // 10ms
		}
	}
}

void Sci_ReadRegsDecode_Master(void)
{
	static uint8_t test = 0;

	uint16_t t_u16Temp;
	// int j;int i = 17;
	switch (g_u16Rs485RegAddr)
	{
	case RS485_ADDR_YS_LCD3: // c003
		t_u16Temp = (uint16_t)g_u8DataRXBuff[3];
		t_u16Temp = t_u16Temp << 8;
		t_u16Temp += g_u8DataRXBuff[4];
		g_stLcdDispData.g_u16RunState = t_u16Temp;

		g_stLcdDispData.g_u16VoutDisp = ((((uint16_t)g_u8DataRXBuff[5]) << 8) | g_u8DataRXBuff[6]); //
		g_stLcdDispData.g_u16IoutDisp = ((((uint16_t)g_u8DataRXBuff[7]) << 8) | g_u8DataRXBuff[8]);
		g_stLcdDispData.g_u16Temp1 = ((((uint16_t)g_u8DataRXBuff[9]) << 8) | g_u8DataRXBuff[10]);
		g_stLcdDispData.g_u16Soc = ((((uint16_t)g_u8DataRXBuff[11]) << 8) | g_u8DataRXBuff[12]); //

		voll = (((((uint16_t)g_u8DataRXBuff[43]) << 8) | g_u8DataRXBuff[44])) - 4;
		SuspendFlag1 = SuspendFlag2;
		SuspendFlag2 = ((((uint16_t)g_u8DataRXBuff[13]) << 8) | g_u8DataRXBuff[14]);
		// 蓝牙
		if ((SuspendFlag2 != 0) || (SuspendFlag1 != SuspendFlag2))
		{
			BlueToothFlag = 1;
		}
		else
		{
			BlueToothFlag = 0;
		}

		// 加热
		if ((((((uint16_t)g_u8DataRXBuff[15]) << 8) | g_u8DataRXBuff[16])) != 0)
		{
			HeatedFlag = 1;
		}
		else
		{
			HeatedFlag = 0;
		}

		LCD_Show_Vatl();
		LCD_Show_SOC();
		LcdShow1_9(); // 对三级保护状态判断并显示

		if (LcdshowFlag)
		{
			if ((++test) % 4 == 0)
			{
				LCD_Show_SOC();
			}
			else
			{
				LcdShow1_9(); // 对三级保护状态判断并显示
			}
		}

		break;

	case RS485_ADDR_RO_LCD:

		break;
	default:
		break;
	}
}

void Sci_WrRegDecode_Master(void)
{
}

void Sci_WrRegsDecode_Master(void)
{
}

void Sci_Transmit_Master(void)
{
	uint16_t u16CRCTemp = 0;
	uint16_t t_u16Rs485RegData;
	uint16_t t_u16Rs485Addr;

	uint16_t i = 0;

	static uint8_t s_u8TxCnt = 0;

	if ((g_u16MasterSendComCnt1ms >= 50) && (TRUE == g_st_DelayFlag.b1PowerStDelay)) // 50ms延时  上电延时时间
	{
		g_u16MasterSendComCnt1ms = 50;

		s_u8TxCnt = 0;

		t_u16Rs485Addr = RS485_SLAVE_ADDR;		// 01
		g_u8Rs485FunCmd = RS485_CMD_READ_REGS;	// 03
		g_u16Rs485RegAddr = RS485_ADDR_YS_LCD3; // c0 RS485_ADDR_RO_LCD RS485_ADDR_YS_LCD4

		t_u16Rs485RegData = 21;

		g_u8DataArry[s_u8TxCnt++] = t_u16Rs485Addr;						 // 地址
		g_u8DataArry[s_u8TxCnt++] = g_u8Rs485FunCmd;					 // 功能码
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr >> 8);	 // 寄存器地址H
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr & 0xFF); // 寄存器地址L
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(t_u16Rs485RegData >> 8);
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(t_u16Rs485RegData & 0xFF);

		u16CRCTemp = CRCChk(g_u8DataArry, s_u8TxCnt);
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(u16CRCTemp & 0xFF);
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(u16CRCTemp >> 8);

		for (i = 0; i < s_u8TxCnt; i++)
		{
			while (!((USART2->ISR) & (1 << 7)))
				;
			USART2->TDR = g_u8DataArry[i];
		}
		g_u16MasterSendComCnt1ms = 0; // 发送完这次200ms任务完成
	}
}

void Sci_Receive_Master(void)
{

	uint16_t u16SciVerify;
	uint16_t t_u16FrameLenth;

	if ((g_u8DataRXBuff[0] == RS485_BROADCAST_ADDR) // 广播地址
		|| (g_u8DataRXBuff[0] == RS485_SLAVE_ADDR)) // 从机地址
	{
		if (g_u8ReceiveCnt < 2)
		{
			return;
		}

		t_u16FrameLenth = g_u8ReceiveCnt - 2;
		u16SciVerify = g_u8DataRXBuff[t_u16FrameLenth] + (g_u8DataRXBuff[t_u16FrameLenth + 1] << 8);
		if (u16SciVerify == CRCChk(g_u8DataRXBuff, t_u16FrameLenth))
		{ // CRC正确
			if (g_u8DataRXBuff[1] == g_u8Rs485FunCmd)
			{
				switch (g_u8Rs485FunCmd)
				{
				case RS485_CMD_READ_COILS:
				case RS485_CMD_WRITE_COIL:
					break;
				case RS485_CMD_READ_REGS:
					Sci_ReadRegsDecode_Master();
					break;
				case RS485_CMD_WRITE_REG:
					Sci_WrRegDecode_Master();
					break;
				case RS485_CMD_WRITE_REGS:
					Sci_WrRegsDecode_Master();
					break;
				default:
					break;
				}
			}
		}
	}
}
void Sci_DataRx_Master(void)
{

	g_u8DataRXBuff[g_u8ReceiveCnt++] = USART2->RDR;

	g_u8ReceiveBeginFlag = 1;

	if (g_u8ReceiveCnt >= (RXBUFLEN - 1)) // Buf限幅保护
	{
		g_u8ReceiveCnt = 0;
		g_u8SciRxErrFlag = 1;
	}
}

void _0x06_sleep_L3_sleep(void)
{
	uint16_t u16CRCTemp = 0;
	uint16_t t_u16Rs485RegData;
	uint16_t t_u16Rs485Addr;

	uint16_t i = 0;

	static uint8_t s_u8TxCnt = 0;
	{
		g_u16MasterSendComCnt1ms = 50;

		t_u16Rs485Addr = RS485_SLAVE_ADDR;					   // 01
		g_u8Rs485FunCmd = RS485_CMD_WRITE_REG;				   // 03
		g_u16Rs485RegAddr = RS485_CMD_ADDR_SYSTEM_FUNCTION_ON; // c0 RS485_ADDR_RO_LCD RS485_ADDR_YS_LCD4

		t_u16Rs485RegData = 0x000A;

		g_u8DataArry[s_u8TxCnt++] = t_u16Rs485Addr;						 // 地址
		g_u8DataArry[s_u8TxCnt++] = g_u8Rs485FunCmd;					 // 功能码
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr >> 8);	 // 寄存器地址H
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr & 0xFF); // 寄存器地址L
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(t_u16Rs485RegData >> 8);
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(t_u16Rs485RegData & 0xFF);

		u16CRCTemp = CRCChk(g_u8DataArry, s_u8TxCnt);
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(u16CRCTemp & 0xFF);
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(u16CRCTemp >> 8);

		for (i = 0; i < s_u8TxCnt; i++)
		{
			while (!((USART2->ISR) & (1 << 7)))
				;
			USART2->TDR = g_u8DataArry[i];
		}
		g_u16MasterSendComCnt1ms = 0; // 发送完这次200ms任务完成
	}

	{
		g_u8ReceiveCnt = 0;
		g_u8SciRxErrFlag = 0;
		g_u8Rs485State = RS485_STATE_WAIT;
	}
}

void _0x06_sleep_stopBMS(void)
{
	uint16_t u16CRCTemp = 0;
	uint16_t t_u16Rs485RegData;
	uint16_t t_u16Rs485Addr;

	uint16_t i = 0;

	static uint8_t s_u8TxCnt = 0;
	{
		g_u16MasterSendComCnt1ms = 50;

		t_u16Rs485Addr = RS485_SLAVE_ADDR;					   // 01
		g_u8Rs485FunCmd = RS485_CMD_WRITE_REG;				   // 03
		g_u16Rs485RegAddr = RS485_CMD_ADDR_SYSTEM_FUNCTION_ON; // c0 RS485_ADDR_RO_LCD RS485_ADDR_YS_LCD4


		t_u16Rs485RegData = 0x000B;

		g_u8DataArry[s_u8TxCnt++] = t_u16Rs485Addr;						 // 地址
		g_u8DataArry[s_u8TxCnt++] = g_u8Rs485FunCmd;					 // 功能码
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr >> 8);	 // 寄存器地址H
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr & 0xFF); // 寄存器地址L
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(t_u16Rs485RegData >> 8);
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(t_u16Rs485RegData & 0xFF);

		u16CRCTemp = CRCChk(g_u8DataArry, s_u8TxCnt);
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(u16CRCTemp & 0xFF);
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(u16CRCTemp >> 8);

		for (i = 0; i < s_u8TxCnt; i++)
		{
			while (!((USART2->ISR) & (1 << 7)))
				;
			USART2->TDR = g_u8DataArry[i];
		}
		g_u16MasterSendComCnt1ms = 0; // 发送完这次200ms任务完成
	}

	// {
	// 	g_u8ReceiveCnt = 0;
	// 	g_u8SciRxErrFlag = 0;
	// 	g_u8Rs485State = RS485_STATE_WAIT;
	// }
}

void App_SciMasterStation(void)
{
	if (MasterSlave_Select == 1)
	{
		return;
	}

	switch (g_u8Rs485State)
	{
	case RS485_STATE_IDLE: // 0
		// if ((g_u16MasterSendComCnt1ms >= 200) && (TRUE == g_st_DelayFlag.b1PowerStDelay))
		if ((g_u16MasterSendComCnt1ms >= 50))
		{ // 每200ms更新一次原副边通讯数据   计数变量
			g_u16MasterSendComCnt1ms = 50;
			// LCD_Display();
			g_u8Rs485State = RS485_STATE_TX; // 1
		}
		break;
	case RS485_STATE_TX:	   // 1
		Sci_Transmit_Master(); // 发指令

		g_u8ReceiveCnt = 0;
		g_u8SciRxErrFlag = 0;

		g_u8Rs485State = RS485_STATE_WAIT; // 2
		break;
	case RS485_STATE_WAIT:
		if (TRUE == g_st_SysCommFlag.bits.bReceiveFinishFlag)
		{
			g_st_SysCommFlag.bits.bReceiveFinishFlag = FALSE;

			g_u8ReceiveBeginFlag = 0;
			g_u16MasterSendComCnt1ms = 0;

			if (0 == g_u8SciRxErrFlag)
			{
				g_u8Rs485State = RS485_STATE_RXOK;
			}
			else
			{
				g_u8Rs485State = RS485_STATE_IDLE;
			}
		}
		else
		{
			if (g_u16MasterSendComCnt1ms > 1000)
			{ // 超时处理，有问题看前面两个函数什么情况
				g_u8ReceiveBeginFlag = 0;
				g_u16MasterSendComCnt1ms = 0;

				g_u8Rs485State = RS485_STATE_IDLE;
			}

			// g_u8ReceiveBeginFlag = 0;
			// g_u16MasterSendComCnt1ms = 0;
			// g_u8Rs485State = RS485_STATE_IDLE;
		}
		break;
	case RS485_STATE_RXOK: // 3
		Sci_Receive_Master();

		g_u16MasterSendComCnt1ms = 0;
		g_u8Rs485State = RS485_STATE_IDLE;
		break;
	default:
		g_u16MasterSendComCnt1ms = 0;
		g_u8Rs485State = RS485_STATE_IDLE;
		break;
	}
}

#if (defined _RS232_1) || (defined _RS485)
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		if (0 == MasterSlave_Select)
		{
			Sci_DataRx_Master();
		}
		else
		{
			Sci_DataRx_Slave(&g_stSCI1CurrentMsgPtr); // 这
		}
	}

	if (USART1->ISR & 0x08)
	{						   // 接收溢出错误，RXNEIE或EIE使能产生中断，开
		USART1->ICR |= 1 << 3; // 清除
							   // RCSTA1bits.CREN = 1;  //使能接收器
							   // USART1->RE = 1;			//使能接收器
	}

	if (USART1->ISR & 0x02)
	{						   // 帧错误，USART_CR3的EIE使能中断，开
		USART1->ICR |= 1 << 1; // 清除
							   // USART1->RE = 1;
	}
}
#endif
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		if (0 == MasterSlave_Select)
		{
			Sci_DataRx_Master();
		}
		else
		{
			Sci_DataRx_Slave(&g_stSCI1CurrentMsgPtr); // 这
		}
	}

	if (USART2->ISR & 0x08)
	{						   // 接收溢出错误，RXNEIE或EIE使能产生中断，开
		USART2->ICR |= 1 << 3; // 清除
							   // RCSTA1bits.CREN = 1;  //使能接收器
							   // USART1->RE = 1;			//使能接收器
	}

	if (USART2->ISR & 0x02)
	{						   // 帧错误，USART_CR3的EIE使能中断，开
		USART2->ICR |= 1 << 1; // 清除
							   // USART1->RE = 1;
	}
}

#ifdef _RS232_2
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		// Sci2_Rx_Deal(&g_stSCI2CurrentMsgPtr);
	}

	if (USART2->ISR & 0x08)
	{						   // 接收溢出错误，RXNEIE或EIE使能产生中断，开
		USART2->ICR |= 1 << 3; // 清除
	}

	if (USART2->ISR & 0x02)
	{						   // 帧错误，USART_CR3的EIE使能中断，开
		USART2->ICR |= 1 << 1; // 清除
	}
}
#endif
void LCDshow5(int num)
{ // 加热和错误时
	switch (num)
	{
	case 0:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0xFA); // 显示	0
		break;
	case 1:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0x0A); // 显示	1
		break;
	case 2:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0xD6); // 显示	2
		break;
	case 3:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0x9E); // 显示	3
		break;
	case 4:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0x2E); // 显示	4
		break;
	case 5:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0xBC); // 显示	5
		break;
	case 6:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0xFC); // 显示	6
		break;
	case 7:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0x1A); // 显示	7
		break;
	case 8:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0xFE); // 显示	8
		break;
	case 9:
		lcd_display(6, 0xF4); // 显示	E
		if (HeatedFlag == 1)
		{						  // 如果加热显示就
			lcd_display(8, 0x7F); // 显示	R
		}
		else
		{
			lcd_display(8, 0x7E); // 显示	R
		}
		lcd_display(10, 0xBE); // 显示	9
		break;
	default:
		break;
	}
}

void LcdShow1_9(void)
{
	uint8_t _delay = 0;

	__delay_ms(1);

	LcdshowFlag = 0;

	if (g_u8DataRXBuff[17] != 0 || g_u8DataRXBuff[20] != 0 || g_u8DataRXBuff[29] != 0 || g_u8DataRXBuff[37] != 0)
	{ // 放电短路37、AFE1通讯17、E2P通讯错误20	、E2P存储错误29
		LCDshow5(0);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}

	if ((((uint16_t)g_u8DataRXBuff[42]) & 0x01) > 0)
	{ // 单节过压		显示ERR：1
		LCDshow5(1);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 1) & 0x01) > 0)
	{ // 单节低压		显示ERR：2
		LCDshow5(2);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 2) & 0x01) > 0)
	{ // 总压过压		显示ERR：1
		LCDshow5(1);
		//__delay_ms(500);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 3) & 0x01) > 0)
	{ // 总压低压		显示ERR：2
		LCDshow5(2);
		//__delay_ms(500);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 4) & 0x01) > 0)
	{ // 充电过流		显示ERR：5
		LCDshow5(5);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 5) & 0x01) > 0)
	{ // 放电过流		显示ERR：6
		LCDshow5(6);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 6) & 0x01) > 0)
	{ // 充电过温		显示ERR：3
		LCDshow5(3);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 7) & 0x01) > 0)
	{ // 放电过温		显示ERR：3
		LCDshow5(3);
		//__delay_ms(500);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[41])) & 0x01) > 0)
	{ // 放电低温		显示ERR：4
		LCDshow5(4);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[41]) >> 1) & 0x01) > 0)
	{ // 充电低温		显示ERR：4
		LCDshow5(4);
		//__delay_ms(500);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[41]) >> 2) & 0x01) > 0)
	{ // 压差过大		显示ERR：7
		LCDshow5(7);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[41]) >> 5) & 0x01) > 0)
	{ // MOS过温		显示ERR：8
		LCDshow5(8);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
}

void LCD_Show_SOC(void)
{
	if (g_stLcdDispData.g_u16Soc / 10 >= 10)
	{
		if (HeatedFlag == 1)
		{
			lcd_display(6, Tab_Num2[1]);

			lcd_display(8, Tab_Num2[0] + 1); // 加热

			lcd_display(10, Tab_Num2[g_stLcdDispData.g_u16Soc % 10] + 1);
		}
		else
		{
			lcd_display(6, Tab_Num2[1]);

			lcd_display(8, Tab_Num2[0]); // 加热

			lcd_display(10, Tab_Num2[g_stLcdDispData.g_u16Soc % 10] + 1);
		}
	}
	else
	{
		// 如果加热开启
		if (HeatedFlag == 1)
		{
			lcd_display(6, Tab_Num2[10]);

			lcd_display(8, Tab_Num2[g_stLcdDispData.g_u16Soc / 10] + 1); // 加热

			lcd_display(10, Tab_Num2[g_stLcdDispData.g_u16Soc % 10] + 1);
		}
		else
		{

			lcd_display(6, Tab_Num2[10]);

			lcd_display(8, Tab_Num2[g_stLcdDispData.g_u16Soc / 10]);

			lcd_display(10, Tab_Num2[g_stLcdDispData.g_u16Soc % 10] + 1); // 加%
		}
	}
}

void LCD_Show_Vatl(void)
{

	if ((voll / 100) == 0)
	{

		if (BlueToothFlag == 1)
		{ // 如果蓝牙开启

			// lcd_display(0, Tab_Num1[(int)g_stLcdDispData.g_u16VoutDisp / 10] + 8);//加蓝牙
			lcd_display(0, 0x08);
		}
		else
		{
			lcd_display(0, 0x00);
		}
	}
	else
	{

		if (BlueToothFlag == 1)
		{ // 如果蓝牙开启

			lcd_display(0, Tab_Num1[(int)voll / 100] + 8); // 加蓝牙
		}
		else
		{

			lcd_display(0, Tab_Num1[voll / 100]);
		}
	}

	lcd_display(2, Tab_Num1[(voll / 10) % 10] + 8); // 加，

	lcd_display(4, Tab_Num1[(voll % 10)] + 8); // 加V
}

/*	问题A：

		1、在出现AFE1通讯错误时连接蓝牙、图标不显示，蓝牙在连接时打开加热蓝牙图标会消失
		2、在出现ERR时会有可能导致通信中断，要重新上电才能恢复
		3、现在测试发现热耦ERR不显示 --- 后续是不需要该报错、需要AFE1通讯错误、E2P通讯错误、E2P存储错误、放电短路

*/

/*	问题B：

		1、现在测试报错类：单节过压、总压过压、单节低压、总压低压、AFE1通讯报错没有问题
		2、蓝牙、加热一起显示没有问题（在连接蓝牙时打开加热）、

*/

int fputc(int ch, FILE *f)
{
#if 0 /* 将需要printf的字符通过串口中断FIFO发送出去，printf函数会立即返回 */
	comSendChar(COM1, ch);

	return ch;
#else /* 采用阻塞方式发送每个字符,等待数据发送完毕 */
	/* 写一个字节到USART1 */
	USART_SendData(USART1, (uint8_t)ch);

	/* 等待发送结束 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}

	return ch;
#endif
}
