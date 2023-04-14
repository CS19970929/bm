#include "main.h"

void LCD_Show_Vatl(void);
void LCD_Show_SOC(void);

uint8_t g_u8Rs485FunCmd;
uint16_t g_u16Rs485RegAddr;

uint8_t g_u8DataRXBuff[RXBUFLEN];
uint8_t g_u8DataArry[TXBUFNUM]; // ���͸��������ݻ���				Ӧ�ÿ����Ż�Ϊ�ֲ�����

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

uint8_t Tab_Num1[12] = {0xF5, 0x05, 0xB6, 0x97, 0x47, 0xD3, 0xF3, 0x85, 0xF7, 0xD7, 0x00, 0xFF}; // �ֱ��Ӧ0~9��ȫ��ȫ��
uint8_t Tab_Num2[12] = {0xFA, 0x0A, 0xD6, 0x9E, 0x2E, 0xBC, 0xFC, 0x1A, 0xFE, 0xBE, 0x00, 0xFF}; // ���������ֶ�Ӧ�ε�������ǰ�����ߵͽ���

void InitUSART1_Master(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // ����USART1����ʱ��
	// RCC->AHBENR |= 1<<17;										//����GPIOA������ʱ��

	// Enable the USART1 Interrupt(ʹ��USART1�ж�)
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// USART1_TX -> PA9 , USART1_RX -> PA10
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1); // 030��AF����ڷ�reg��datasheet��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ���ڳ�ʼ��
	USART_InitStructure.USART_BaudRate = 19200;										// ���ô��ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// ��������λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// ����ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����Ч��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ����������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// ���ù���ģʽ
	USART_Init(USART1, &USART_InitStructure);										// ������ṹ��

	USART1->CR3 |= 1 << 0;	// EIE����֡�����жϣ�ͬʱ���������ж�
	USART1->CR3 |= 1 << 11; // δ��ʹ��ǰ��д����ֹ�����ж�

	USART_Cmd(USART1, ENABLE);					   // ʹ�ܴ���1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ʹ�ܽ����ж�
}

void InitUSART2_Master(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	// RCC->AHBENR |= 1<<17;										//����GPIOA������ʱ��

	// Enable the USART2 Interrupt(ʹ��USART2�ж�)
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// USART2_TX -> PA9 , USART2_RX -> PA3
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1); // 030��AF����ڷ�reg��datasheet��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ���ڳ�ʼ��
	USART_InitStructure.USART_BaudRate = 19200;										// ���ô��ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// ��������λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// ����ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����Ч��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ����������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// ���ù���ģʽ
	USART_Init(USART2, &USART_InitStructure);										// ������ṹ��

	USART2->CR3 |= 1 << 0;	// EIE����֡�����жϣ�ͬʱ���������ж�
	USART2->CR3 |= 1 << 11; // δ��ʹ��ǰ��д����ֹ�����ж�

	USART_Cmd(USART2, ENABLE);					   // ʹ�ܴ���1
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // ʹ�ܽ����ж�

	// Sci_DataInit(&g_stCurrentMsgPtr_SCI2);
}

void TimCnt(void)
{
	static UINT16 s_u16Delay200msCnt = 0;
	static UINT8 s_u8RecDelayCnt = 0;

	g_u16MasterSendComCnt1ms++;

	// �����߼�Ӧ��û�ã������Ż���
	if (g_st_DelayFlag.b1PowerStDelay == FALSE)
	{
		s_u16Delay200msCnt++;
		if (s_u16Delay200msCnt >= 200)
		{
			s_u16Delay200msCnt = 200; // why 200,���������

			g_st_DelayFlag.b1PowerStDelay = TRUE;
		}
	}

	if (1 == g_u8ReceiveBeginFlag)
	{
		s_u8RecDelayCnt++;
		if (s_u8RecDelayCnt > SciReceiveTime) // SciReceiveTime	�費��Ҫ�Ĵ��
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
		// ����
		if ((SuspendFlag2 != 0) || (SuspendFlag1 != SuspendFlag2))
		{
			BlueToothFlag = 1;
		}
		else
		{
			BlueToothFlag = 0;
		}

		// ����
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
		LcdShow1_9(); // ����������״̬�жϲ���ʾ

		if (LcdshowFlag)
		{
			if ((++test) % 4 == 0)
			{
				LCD_Show_SOC();
			}
			else
			{
				LcdShow1_9(); // ����������״̬�жϲ���ʾ
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

	if ((g_u16MasterSendComCnt1ms >= 50) && (TRUE == g_st_DelayFlag.b1PowerStDelay)) // 50ms��ʱ  �ϵ���ʱʱ��
	{
		g_u16MasterSendComCnt1ms = 50;

		s_u8TxCnt = 0;

		t_u16Rs485Addr = RS485_SLAVE_ADDR;		// 01
		g_u8Rs485FunCmd = RS485_CMD_READ_REGS;	// 03
		g_u16Rs485RegAddr = RS485_ADDR_YS_LCD3; // c0 RS485_ADDR_RO_LCD RS485_ADDR_YS_LCD4

		t_u16Rs485RegData = 21;

		g_u8DataArry[s_u8TxCnt++] = t_u16Rs485Addr;						 // ��ַ
		g_u8DataArry[s_u8TxCnt++] = g_u8Rs485FunCmd;					 // ������
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr >> 8);	 // �Ĵ�����ַH
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr & 0xFF); // �Ĵ�����ַL
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
		g_u16MasterSendComCnt1ms = 0; // ���������200ms�������
	}
}

void Sci_Receive_Master(void)
{

	uint16_t u16SciVerify;
	uint16_t t_u16FrameLenth;

	if ((g_u8DataRXBuff[0] == RS485_BROADCAST_ADDR) // �㲥��ַ
		|| (g_u8DataRXBuff[0] == RS485_SLAVE_ADDR)) // �ӻ���ַ
	{
		if (g_u8ReceiveCnt < 2)
		{
			return;
		}

		t_u16FrameLenth = g_u8ReceiveCnt - 2;
		u16SciVerify = g_u8DataRXBuff[t_u16FrameLenth] + (g_u8DataRXBuff[t_u16FrameLenth + 1] << 8);
		if (u16SciVerify == CRCChk(g_u8DataRXBuff, t_u16FrameLenth))
		{ // CRC��ȷ
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

	if (g_u8ReceiveCnt >= (RXBUFLEN - 1)) // Buf�޷�����
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

		g_u8DataArry[s_u8TxCnt++] = t_u16Rs485Addr;						 // ��ַ
		g_u8DataArry[s_u8TxCnt++] = g_u8Rs485FunCmd;					 // ������
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr >> 8);	 // �Ĵ�����ַH
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr & 0xFF); // �Ĵ�����ַL
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
		g_u16MasterSendComCnt1ms = 0; // ���������200ms�������
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

		g_u8DataArry[s_u8TxCnt++] = t_u16Rs485Addr;						 // ��ַ
		g_u8DataArry[s_u8TxCnt++] = g_u8Rs485FunCmd;					 // ������
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr >> 8);	 // �Ĵ�����ַH
		g_u8DataArry[s_u8TxCnt++] = (uint8_t)(g_u16Rs485RegAddr & 0xFF); // �Ĵ�����ַL
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
		g_u16MasterSendComCnt1ms = 0; // ���������200ms�������
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
		{ // ÿ200ms����һ��ԭ����ͨѶ����   ��������
			g_u16MasterSendComCnt1ms = 50;
			// LCD_Display();
			g_u8Rs485State = RS485_STATE_TX; // 1
		}
		break;
	case RS485_STATE_TX:	   // 1
		Sci_Transmit_Master(); // ��ָ��

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
			{ // ��ʱ���������⿴ǰ����������ʲô���
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
			Sci_DataRx_Slave(&g_stSCI1CurrentMsgPtr); // ��
		}
	}

	if (USART1->ISR & 0x08)
	{						   // �����������RXNEIE��EIEʹ�ܲ����жϣ���
		USART1->ICR |= 1 << 3; // ���
							   // RCSTA1bits.CREN = 1;  //ʹ�ܽ�����
							   // USART1->RE = 1;			//ʹ�ܽ�����
	}

	if (USART1->ISR & 0x02)
	{						   // ֡����USART_CR3��EIEʹ���жϣ���
		USART1->ICR |= 1 << 1; // ���
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
			Sci_DataRx_Slave(&g_stSCI1CurrentMsgPtr); // ��
		}
	}

	if (USART2->ISR & 0x08)
	{						   // �����������RXNEIE��EIEʹ�ܲ����жϣ���
		USART2->ICR |= 1 << 3; // ���
							   // RCSTA1bits.CREN = 1;  //ʹ�ܽ�����
							   // USART1->RE = 1;			//ʹ�ܽ�����
	}

	if (USART2->ISR & 0x02)
	{						   // ֡����USART_CR3��EIEʹ���жϣ���
		USART2->ICR |= 1 << 1; // ���
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
	{						   // �����������RXNEIE��EIEʹ�ܲ����жϣ���
		USART2->ICR |= 1 << 3; // ���
	}

	if (USART2->ISR & 0x02)
	{						   // ֡����USART_CR3��EIEʹ���жϣ���
		USART2->ICR |= 1 << 1; // ���
	}
}
#endif
void LCDshow5(int num)
{ // ���Ⱥʹ���ʱ
	switch (num)
	{
	case 0:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0xFA); // ��ʾ	0
		break;
	case 1:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0x0A); // ��ʾ	1
		break;
	case 2:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0xD6); // ��ʾ	2
		break;
	case 3:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0x9E); // ��ʾ	3
		break;
	case 4:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0x2E); // ��ʾ	4
		break;
	case 5:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0xBC); // ��ʾ	5
		break;
	case 6:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0xFC); // ��ʾ	6
		break;
	case 7:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0x1A); // ��ʾ	7
		break;
	case 8:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0xFE); // ��ʾ	8
		break;
	case 9:
		lcd_display(6, 0xF4); // ��ʾ	E
		if (HeatedFlag == 1)
		{						  // ���������ʾ��
			lcd_display(8, 0x7F); // ��ʾ	R
		}
		else
		{
			lcd_display(8, 0x7E); // ��ʾ	R
		}
		lcd_display(10, 0xBE); // ��ʾ	9
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
	{ // �ŵ��·37��AFE1ͨѶ17��E2PͨѶ����20	��E2P�洢����29
		LCDshow5(0);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}

	if ((((uint16_t)g_u8DataRXBuff[42]) & 0x01) > 0)
	{ // ���ڹ�ѹ		��ʾERR��1
		LCDshow5(1);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 1) & 0x01) > 0)
	{ // ���ڵ�ѹ		��ʾERR��2
		LCDshow5(2);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 2) & 0x01) > 0)
	{ // ��ѹ��ѹ		��ʾERR��1
		LCDshow5(1);
		//__delay_ms(500);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 3) & 0x01) > 0)
	{ // ��ѹ��ѹ		��ʾERR��2
		LCDshow5(2);
		//__delay_ms(500);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 4) & 0x01) > 0)
	{ // ������		��ʾERR��5
		LCDshow5(5);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 5) & 0x01) > 0)
	{ // �ŵ����		��ʾERR��6
		LCDshow5(6);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 6) & 0x01) > 0)
	{ // ������		��ʾERR��3
		LCDshow5(3);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[42]) >> 7) & 0x01) > 0)
	{ // �ŵ����		��ʾERR��3
		LCDshow5(3);
		//__delay_ms(500);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[41])) & 0x01) > 0)
	{ // �ŵ����		��ʾERR��4
		LCDshow5(4);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[41]) >> 1) & 0x01) > 0)
	{ // ������		��ʾERR��4
		LCDshow5(4);
		//__delay_ms(500);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[41]) >> 2) & 0x01) > 0)
	{ // ѹ�����		��ʾERR��7
		LCDshow5(7);
		__delay_ms(_delay);
		LcdshowFlag = 1;
	}
	if (((((uint16_t)g_u8DataRXBuff[41]) >> 5) & 0x01) > 0)
	{ // MOS����		��ʾERR��8
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

			lcd_display(8, Tab_Num2[0] + 1); // ����

			lcd_display(10, Tab_Num2[g_stLcdDispData.g_u16Soc % 10] + 1);
		}
		else
		{
			lcd_display(6, Tab_Num2[1]);

			lcd_display(8, Tab_Num2[0]); // ����

			lcd_display(10, Tab_Num2[g_stLcdDispData.g_u16Soc % 10] + 1);
		}
	}
	else
	{
		// ������ȿ���
		if (HeatedFlag == 1)
		{
			lcd_display(6, Tab_Num2[10]);

			lcd_display(8, Tab_Num2[g_stLcdDispData.g_u16Soc / 10] + 1); // ����

			lcd_display(10, Tab_Num2[g_stLcdDispData.g_u16Soc % 10] + 1);
		}
		else
		{

			lcd_display(6, Tab_Num2[10]);

			lcd_display(8, Tab_Num2[g_stLcdDispData.g_u16Soc / 10]);

			lcd_display(10, Tab_Num2[g_stLcdDispData.g_u16Soc % 10] + 1); // ��%
		}
	}
}

void LCD_Show_Vatl(void)
{

	if ((voll / 100) == 0)
	{

		if (BlueToothFlag == 1)
		{ // �����������

			// lcd_display(0, Tab_Num1[(int)g_stLcdDispData.g_u16VoutDisp / 10] + 8);//������
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
		{ // �����������

			lcd_display(0, Tab_Num1[(int)voll / 100] + 8); // ������
		}
		else
		{

			lcd_display(0, Tab_Num1[voll / 100]);
		}
	}

	lcd_display(2, Tab_Num1[(voll / 10) % 10] + 8); // �ӣ�

	lcd_display(4, Tab_Num1[(voll % 10)] + 8); // ��V
}

/*	����A��

		1���ڳ���AFE1ͨѶ����ʱ����������ͼ�겻��ʾ������������ʱ�򿪼�������ͼ�����ʧ
		2���ڳ���ERRʱ���п��ܵ���ͨ���жϣ�Ҫ�����ϵ���ָܻ�
		3�����ڲ��Է�������ERR����ʾ --- �����ǲ���Ҫ�ñ�����ҪAFE1ͨѶ����E2PͨѶ����E2P�洢���󡢷ŵ��·

*/

/*	����B��

		1�����ڲ��Ա����ࣺ���ڹ�ѹ����ѹ��ѹ�����ڵ�ѹ����ѹ��ѹ��AFE1ͨѶ����û������
		2������������һ����ʾû�����⣨����������ʱ�򿪼��ȣ���

*/

int fputc(int ch, FILE *f)
{
#if 0 /* ����Ҫprintf���ַ�ͨ�������ж�FIFO���ͳ�ȥ��printf�������������� */
	comSendChar(COM1, ch);

	return ch;
#else /* ����������ʽ����ÿ���ַ�,�ȴ����ݷ������ */
	/* дһ���ֽڵ�USART1 */
	USART_SendData(USART1, (uint8_t)ch);

	/* �ȴ����ͽ��� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}

	return ch;
#endif
}
