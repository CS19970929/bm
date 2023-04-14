#include "main.h"

/*************************SCI1 Global Variables****************************/
UINT16 g_u16SCI1CommuErrCnt = 0;         //SCIͨ���쳣����
UINT8  g_u8SCI1TxEnable = 0;
UINT8  g_u8SCI1TxFinishFlag = 0;
struct RS485MSG g_stSCI1CurrentMsgPtr;

volatile union SysStatusFlag g_st_SysStatusFlag;

UINT8 g_u8SCITxBuff[SCI_TX_BUF_LEN];
UINT8 u8FlashUpdateFlag = 0;
UINT8 u8FlashUpdateE2PROM = 0;

UINT8  LcdshowFlag = 0;//�����ж��Ƿ�����ʾ
UINT8  LcdshowFlag1 = 0;//�����ж��Ƿ�����ʾ
UINT8  LcdshowFlag2 = 0;//�����ж��Ƿ�����ʾ
UINT8  LcdshowFlag3 = 0;//�����ж��Ƿ�����ʾ
UINT8  LcdshowFlag4 = 0;//�����ж��Ƿ�����ʾ
UINT8  LcdshowFlag5 = 0;//�����ж��Ƿ�����ʾ
UINT8  LcdshowFlag6 = 0;//�����ж��Ƿ�����ʾ
UINT8  LcdshowFlag7 = 0;//�����ж��Ƿ�����ʾ
UINT8  LcdshowFlag8 = 0;//�����ж��Ƿ�����ʾ
UINT8  LcdshowFlag9 = 0;//�����ж��Ƿ�����ʾ
UINT8  BlueToothFlag = 0;//�����ж������Ƿ�����ʾ
UINT8		HeatedFlag = 0;	//����
UINT8		PintFlag = 0;
UINT16 vol;UINT16 soc;
UINT8 g_u16RdSel = 0;
UINT8 	u16Buffer_TX[U16BUFFER_TX];//���buff�����ʾ���ݵ�ָ��

void Sci_DataInit( struct RS485MSG *s ){
	UINT16 i;

	s->ptr_no = 0;
	s->csr = RS485_STA_IDLE;
	s->enRs485CmdType = RS485_CMD_READ_REGS;
	for(i = 0; i < RS485_MAX_BUFFER_SIZE; i++)
	{
		s->u16Buffer[i] = 0;
	}
	
	for(i = 0; i < SCI_TX_BUF_LEN; i++) {
		g_u8SCITxBuff[i] = 0;
	}
}

void CRC_verify( struct RS485MSG *s ) {
	UINT16	u16SciVerify;
	UINT16	t_u16FrameLenth;
	
	t_u16FrameLenth = s->ptr_no - 2;
	u16SciVerify = s->u16Buffer[t_u16FrameLenth] + (s->u16Buffer[t_u16FrameLenth+1] << 8);
	if(u16SciVerify == Sci_CRC16RTU(( UINT8 * )s->u16Buffer, t_u16FrameLenth)) {		
		s ->AckType = RS485_ACK_POS;
	}
	else {
		s ->u16RdRegByteNum = 0;
		s ->AckType = RS485_ACK_NEG;
		s ->ErrorType = RS485_ERROR_CRC_ERROR;
	}
}


void Sci_ReadRegs_Data( struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {

	
}

void Sci_Tx_Fun_LCD( struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {

		switch(s->u16RdRegStartAddr) {		//��03����
			
			default:
			s->u16RdRegStartAddr = 0;
			break;
		}
		s->u16RdRegStartAddr = 0;
}


void Sci_RW_Data_Pro(struct RS485MSG *s,UINT8 t_u8BuffTemp[]){


}

void Sci_RW_Data_Cali1(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {

	

}

void Sci_RW_Data_Cali2(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {

}


void Sci_RW_Data_Other(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {



}


void Sci_ReadRegsDecode( struct RS485MSG *s ){
	UINT16 t_u16Temp;

	t_u16Temp = s->u16Buffer[3] + (s->u16Buffer[2] << 8);
	s->u16RdRegStartAddrActure = t_u16Temp;
	//t_u16Temp2 = (s->u16Buffer[5] + (s->u16Buffer[4] << 8));
    if(t_u16Temp >= (RS485_ADDR_RO_START + 0x0060)) {//9��
		t_u16Temp -= (RS485_ADDR_RO_START + 96-22-10-2-13);
    }

    else if(t_u16Temp >= (RS485_ADDR_RO_START + 0x0050)) {//13��
		t_u16Temp -= (RS485_ADDR_RO_START + 80-22-10-2);
    }
	
	else if(t_u16Temp >= (RS485_ADDR_RO_START + 0x0040)) {//2��
		t_u16Temp -= (RS485_ADDR_RO_START + 64 - 22 - 10);
	}
	
	else if(t_u16Temp >= (RS485_ADDR_RO_START + 0x0030)) {//10��
		t_u16Temp -= (RS485_ADDR_RO_START + 48 - 22);
	}
	
	else if(t_u16Temp >= RS485_ADDR_RO_START) {//22��
		t_u16Temp -= RS485_ADDR_RO_START;
	}
	else if(t_u16Temp >= RS485_ADDR_YS_LCD6){	//0XC006
		t_u16Temp -= RS485_ADDR_RO_LCD;  	//0xC000   //6		����		12��
	}
			//�¼ӽ�����
	else if(t_u16Temp >= RS485_ADDR_YS_LCD5){	//0XC005
		t_u16Temp -= RS485_ADDR_RO_LCD;  	//0xC000   //5		����������	2��
	}
			//�¼ӽ�����
	else if(t_u16Temp >= RS485_ADDR_YS_LCD4){	//0XC004
		t_u16Temp -= RS485_ADDR_RO_LCD;  	//0xC000   //4		����ѹ��SOC
	}
			//�¼ӽ�����
	else if(t_u16Temp >= RS485_ADDR_YS_LCD3){	//0XC003
		t_u16Temp -= RS485_ADDR_RO_LCD;  	//0xC000   //3		������ȫ״̬
	}
	//�¼ӽ�����
	else if(t_u16Temp >= RS485_ADDR_RO_LCD){
		t_u16Temp -= RS485_ADDR_RO_LCD;     //��ѹ��SOC��
	}
	else if(t_u16Temp >= RS485_ADDR_RW_START3){
		t_u16Temp -= RS485_ADDR_RW_START3;
	}	
	else if(t_u16Temp >= RS485_ADDR_RW_START2) {
		t_u16Temp -= RS485_ADDR_RW_START2;
	}
	else if(t_u16Temp >= RS485_ADDR_RW_Cali2) {
		t_u16Temp -= RS485_ADDR_RW_Cali2;
	}
	else if(t_u16Temp >= RS485_ADDR_RW_Cali1) {
		t_u16Temp -= RS485_ADDR_RW_Cali1;
	}

	s ->u16RdRegStartAddr = t_u16Temp;
	s ->u16RdRegByteNum = (s->u16Buffer[5] + (s->u16Buffer[4] << 8))<<1;
}

/*=================================================================
* FUNCTION: Sci_WrRegDecode
* PURPOSE : ��д�����Ĵ���06 �������н���
* INPUT:    void
*
* RETURN:   void
*
* CALLS:    void
*
* CALLED BY:Sci_Rx_Fun()
*
*=================================================================*/
void Sci_WrRegDecode(struct RS485MSG *s) {

}

/*=================================================================
* FUNCTION: Sci_WrRegsDecode
* PURPOSE : ��д�������н���
* INPUT:    void
*
* RETURN:   void
*
* CALLS:    void
*
* CALLED BY:Sci_Rx_Fun()
*
*=================================================================*/
void Sci_WrRegsDecode(struct RS485MSG *s) {

	uint16_t u16WrRegNum;
	uint16_t u16SciRegStartAddr;
	u16SciRegStartAddr = s->u16Buffer[3] + (s->u16Buffer[2] << 8);
	u16WrRegNum = s->u16Buffer[5] + (s->u16Buffer[4] << 8);
	switch(u16SciRegStartAddr) {
		case RS485_CMD_ADDR_FLASH_CONNECT:
			if(u16WrRegNum == 1) {
				u8FlashUpdateE2PROM = 1;
				WriteEEPROM_Word_NoZone(EEPROM_FLASHUPDATE_ADDR, EEPROM_FLASHUPDATE_VALUE);
			}
			else {
				s ->AckType = RS485_ACK_NEG;
				s ->ErrorType = RS485_ERROR_CMD_INVALID;
			}
			break;		//���˸�BREAK����OVER��
		default:
			s ->AckType = RS485_ACK_NEG;
			s ->ErrorType = RS485_ERROR_CMD_INVALID;
			break;
	}
}


void Sci_ReadRegs_ACK( struct RS485MSG *s ) {
	UINT8 i;
	UINT16	u16SciTemp;	
	if(s->AckType == RS485_ACK_POS) {
		if(s->u16RdRegStartAddrActure >= RS485_ADDR_RW_Cali1) {
			if(s->u16RdRegStartAddrActure >= RS485_ADDR_RO_START) {
				Sci_ReadRegs_Data(s,g_u8SCITxBuff);
			}
			else if(s->u16RdRegStartAddrActure >= RS485_ADDR_RO_LCD) {//0xC000
				Sci_Tx_Fun_LCD(s,g_u8SCITxBuff);//��
			}
			else if(s->u16RdRegStartAddrActure >= RS485_ADDR_RW_START3) {
				Sci_RW_Data_Other(s,g_u8SCITxBuff);
			}			
			else if(s->u16RdRegStartAddrActure >= RS485_ADDR_RW_START2) {
				Sci_RW_Data_Pro(s,g_u8SCITxBuff);
			}
			else if(s->u16RdRegStartAddrActure >= RS485_ADDR_RW_Cali2) {
				Sci_RW_Data_Cali2(s,g_u8SCITxBuff);
			}
			else if(s->u16RdRegStartAddrActure >= RS485_ADDR_RW_Cali1) {
				Sci_RW_Data_Cali1(s,g_u8SCITxBuff);
			}
			//ͷ�룬ǰ�����ֽڱ��ֲ���
			s->u16Buffer[0] = (s->u16Buffer[0] != 0)?RS485_SLAVE_ADDR:s->u16Buffer[0];
			s->u16Buffer[1] = s->enRs485CmdType;
			s->u16Buffer[2] = s->u16RdRegByteNum;
			//����
			for(i = 0; i < (s->u16RdRegByteNum); i++ ) {
				s->u16Buffer[i+3] = g_u8SCITxBuff[i + ((s->u16RdRegStartAddr)<<1)];
			}
			i =  s->u16RdRegByteNum + 3;
		}
	}
	else {
		i = 1;
		s->u16Buffer[i++] = s->enRs485CmdType | 0x80;
		s->u16Buffer[i++] = s->ErrorType;
	}
	u16SciTemp = Sci_CRC16RTU(( UINT8 * )s->u16Buffer, i);
	s->u16Buffer[i++] = u16SciTemp & 0x00FF;
	s->u16Buffer[i++] = u16SciTemp >> 8;
	s->AckLenth = i;
	
	s->ptr_no = 0;
	s->csr = RS485_STA_TX_COMPLETE; 
}


void Sci_WrReg_s_Decode_ACK(struct RS485MSG *s) {
    UINT8 i;
	UINT16	u16SciTemp;
	//if((u32E2PrtWriteFlag == 0)&&(u32E2WarnWriteFlag == 0)) {
		if( s->AckType == RS485_ACK_POS) {
			i = 6;
		}
		else {
			i = 1;
			s->u16Buffer[i++] = s->enRs485CmdType | 0x80;
			s->u16Buffer[i++] = s->ErrorType;
		}

		u16SciTemp = Sci_CRC16RTU(( UINT8 * )s->u16Buffer, i);
		s->u16Buffer[i++] = u16SciTemp & 0x00FF;
		s->u16Buffer[i++] = u16SciTemp >> 8;
		s->AckLenth = i;

		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
	//}
}

void InitUSART1_Slave(void) {
    
    Sci_DataInit(&g_stSCI1CurrentMsgPtr);
}	
//UINT16 g_u16SCI_ReceiveCnt = 0;         //SCIͨ�ż���

void Sci1_FaultChk(void)
{
	if(USART1->ISR&0x08) {		//�����������RXNEIE��EIEʹ�ܲ����жϣ���
		USART1->ICR |= 1<<3;	//���
		//RCSTA1bits.CREN = 1;  //ʹ�ܽ�����
		//USART1->RE = 1;			//ʹ�ܽ�����
	}

	if(USART1->ISR&0x04) {		//��⵽������Ĭ�Ͽ��������Ļ�CR3��ONEBIT��1������
								//USART_CR3��EIEʹ���ж�
		USART1->ICR |= 1<<2;	//���
		//USART1->RE = 1;
	}

	if(USART1->ISR&0x02) {		//֡����USART_CR3��EIEʹ���жϣ���
		USART1->ICR |= 1<<1;	//���
		//USART1->RE = 1;
	}

	if(USART1->ISR&0x01) {		//У������־ USART_CR1��PEIEʹ�ܸ��жϣ�����
		USART1->ICR |= 1<<0;	//���
		//USART1->RE = 1;
	}

	if(1 == g_st_SysTimeFlag.bits.b1Sys10msFlag)
	{
		g_u16SCI1CommuErrCnt++;
		if(g_u16SCI1CommuErrCnt >= DELAYB10MS_5S)
		{
			g_u16SCI1CommuErrCnt = DELAYB10MS_5S;
			g_st_SysStatusFlag.bits.b1SCI1CommuErr = 1;
		}
		else
		{
			g_st_SysStatusFlag.bits.b1SCI1CommuErr = 0;
		}             
	}
}


//���������ݽ��룬�����ж��е���
/*=================================================================
* FUNCTION: Sci2_Rx_Deal
* PURPOSE : �������ݽ��ս���
* INPUT:    void
*
* RETURN:   void
*
* CALLS:    void
*
* CALLED BY:ISR()
*
*=================================================================*/


void Sci_DataRx_Slave(struct RS485MSG *s) {

	USART1->CR2 &= ~(1<<5);
	s->u16Buffer[s->ptr_no] = USART2->RDR; 				// ��RXFIFO �ж�ȡ���յ�������
	if((s->ptr_no == 0) && (s->u16Buffer[0] != RS485_SLAVE_ADDR )&& ( s->u16Buffer[0] != RS485_BROADCAST_ADDR ))
	{
		s->ptr_no = 0;
		s->u16Buffer[0] = 0;
	}
	else
	{
		if(s->ptr_no == 1)
		{
			switch(s->u16Buffer[s->ptr_no])
			{
				case RS485_CMD_READ_REGS:
					s->enRs485CmdType = RS485_CMD_READ_REGS;
					s->ptr_no++;
					break;
				case RS485_CMD_WRITE_REG:
					s->enRs485CmdType = RS485_CMD_WRITE_REG;
					s->ptr_no++;
					break;
				case RS485_CMD_WRITE_REGS:
					s->enRs485CmdType = RS485_CMD_WRITE_REGS;
					s->ptr_no++;
					break;
				default:
					s->ptr_no = 0;
					s->u16Buffer[0] = 0;
					s->u16Buffer[1] = 0;
					break;
			}
		}
		else if(s->ptr_no > 2)
		{
			switch(s->enRs485CmdType)
			{
				case RS485_CMD_READ_REGS:
				case RS485_CMD_WRITE_REG:
				{
					if (s->ptr_no == 7)	//	receive complete
					{
						s->csr = RS485_STA_RX_COMPLETE;
						//RCSTA1bits.CREN = 0;  //��ֹ����
						//RC1IE = 0;			// ��ֹEUSART2 �����ж�
						USART2->CR1 &= ~(1<<2);
						USART2->CR1 &= ~(1<<5);	
					}
					break;
				}

				case RS485_CMD_WRITE_REGS:
				{
					if((s->ptr_no >= 7)&&(s->ptr_no == (s->u16Buffer[6] + 8)))
					{
						s->csr = RS485_STA_RX_COMPLETE;
						//disable rx TODO
						//disable rx/tx interrupt TODO
						//RCSTA1bits.CREN = 0;    //��ֹ����
						//RC1IE = 0;				// ��ֹEUSART2 �����ж�
						USART2->CR1 &= ~(1<<2);
						USART2->CR1 &= ~(1<<5);
					}
					break;
				}
				default:
				{
					s->ptr_no = 0;
					s->u16Buffer[0] = 0;
					break;
				}
			}
			
			s->ptr_no++;

			if(s->ptr_no == RS485_MAX_BUFFER_SIZE)
			{
				s->ptr_no = 0;
				s->u16Buffer[0] = 0;
			}
		}
		else
		{
			s->ptr_no++;
		}		
	}
	//ʹ��RX�ж�TODO
	//RC1IE = 1;// ʹ��EUSART2 �����ж�\
	//NVIC_EnableIRQ(USART1_IRQn);
	USART2->CR1 |= (1<<5);
}

/*=================================================================
* FUNCTION: Sci2_Tx_Deal
* PURPOSE : �������ݷ���
* INPUT:    void
*
* RETURN:   void
*
* CALLS:    void
*
* CALLED BY:SCI_TX_isr()
*
*=================================================================*/


void Sci1_Tx_Deal(struct RS485MSG *s )
{
	if(0 == g_u8SCI1TxEnable)
	{
		return;
	}
	while(!((USART2->ISR)&(1<<7)));
	if( s->ptr_no < s -> AckLenth) {
        USART2->TDR = s->u16Buffer[s->ptr_no];	//load data
		s->ptr_no++;
	}
	else {
		//g_u16SCI_ReceiveCnt++;
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8SCI1TxFinishFlag = 1;
		//pstSciRegs -> SCICTL1.bit.TXENA = 0;				//disable tx
		//pstSciRegs -> SCICTL2.all = 0x00;					//disable tx/rx interrupt
		//TXSTA2bits.TXEN = 0;    //��ֹ���ͣ����ܽ�ֹ������ͨ�ź����׶�
		g_u8SCI1TxEnable = 0;
		if(u8FlashUpdateE2PROM) {
			u8FlashUpdateE2PROM = 0;
			u8FlashUpdateFlag = 1;
		}
	}
}

/*=================================================================
* FUNCTION: Sci1_Updata
* PURPOSE : �շ�����״̬���л�
* INPUT:    void
*
* RETURN:   void
*
* CALLS:    void
*
* CALLED BY:main()
*
*=================================================================*/

void App_SciSlaveStation(struct RS485MSG *s) {
	if(MasterSlave_Select == 0) {
		return;
	}
	switch(s->csr) {
		//IDLE-����̬������50ms��ʹ�ܽ��գ�����㣩receive set
		case RS485_STA_IDLE: {
			break;
		}
		//receive complete, to deal the receive data
		case RS485_STA_RX_COMPLETE: {
			//RC1IE = 0;// ��ֹEUSART1 �����ж�
			USART2->CR1 &= ~(1<<5);
			CRC_verify(s);
			if(s ->AckType == RS485_ACK_POS) {
				switch(s->enRs485CmdType) {
					case RS485_CMD_READ_REGS:
						Sci_ReadRegsDecode(s);
						break;
					case RS485_CMD_WRITE_REG:
						Sci_WrRegDecode(s);
						break;
					case RS485_CMD_WRITE_REGS:
						Sci_WrRegsDecode(s);
						break;
					default:
						s ->u16RdRegByteNum = 0;
						s ->AckType = RS485_ACK_NEG;
						s ->ErrorType = RS485_ERROR_NULL;
						break;
				}
			}
			
			s->csr = RS485_STA_RX_OK;		//receive the correct data, switch to transmit wait 50ms
			break;              //��һ������
		}
		//receive ok, to transmit wait 50ms
		case RS485_STA_RX_OK: {
			switch(s->enRs485CmdType) {
				case RS485_CMD_READ_REGS:
					Sci_ReadRegs_ACK(s);
					//TXSTA1bits.TXEN = 1;    //ʹ�ܷ���
					USART1->CR1 |= (1<<3);
					g_u8SCI1TxEnable = 1;
					g_u16SCI1CommuErrCnt = 0;
					break;
				case RS485_CMD_WRITE_REG:
				case RS485_CMD_WRITE_REGS:
					Sci_WrReg_s_Decode_ACK(s);
					//TXSTA1bits.TXEN = 1;    //ʹ�ܷ���
					USART1->CR1 |= (1<<3);
					g_u8SCI1TxEnable = 1;
					break;
				default:
					break;
			}
		}
		//transmit complete, to switch receive wait 20ms
		case RS485_STA_TX_COMPLETE: {
			if(g_u8SCI1TxFinishFlag) {
				s->csr = RS485_STA_IDLE;
				s->u16Buffer[0] = 0;
				s->u16Buffer[1] = 0;
				s->u16Buffer[2] = 0;
				s->u16Buffer[3] = 0;
				g_u8SCI1TxFinishFlag = 0;
				s->ptr_no = 0;

				//RCSTA1bits.CREN = 1;    //ʹ�ܽ���
				//RC1IE = 1;// ʹ��EUSART2 �����ж�
				USART1->CR1 |= (1<<2);
				USART1->CR1 |= (1<<5);	
				//TXSTA1bits.TXEN = 0;    //��ֹ���ͣ����ܽ�ֹ������ͨ�ź����׶�
				g_u8SCI1TxEnable = 0;
			}
			break;
		}
		
		default: {
			s->csr = RS485_STA_IDLE;
			break;
		}
	}

	Sci1_Tx_Deal(s);
	Sci1_FaultChk();
}

void TXbuffInt(void){//��ָ��װ��buff
	
	UINT16 u16RegAddrTemp;//ָ���ַ
  UINT16 u16RegNumTemp;//���ݸ���
	UINT16	u16SciTemp;	//Ч��
	int i = 0;
	switch(g_u16RdSel++){
	
		case 0:		//������ȫ״̬
		u16RegAddrTemp = 0xC003;
		u16RegNumTemp = 1;
		break;
		
		case 1://����
		u16RegAddrTemp = 0xC006;
		u16RegNumTemp = 12;
		break;
		
		case 2://���ȡ�����
		u16RegAddrTemp = 0xC005;
		u16RegNumTemp = 2;
		break;
		
		case 4://����ѹ��SOC
		u16RegAddrTemp = 0xC004;
		u16RegNumTemp = 2;
		g_u16RdSel = 0;	
		break;
		
		default:
			break;
	}
		u16Buffer_TX[0] = 0x01;			//
		u16Buffer_TX[1] = 0x03;		//������
		u16Buffer_TX[2] = (UINT8)(u16RegAddrTemp >> 8);//��λ				��ַ
		u16Buffer_TX[3] = (UINT8)(u16RegAddrTemp & 0x00FF);//��λ		��ַ
		u16Buffer_TX[4] = (UINT8)(u16RegNumTemp >> 8);				//����
		u16Buffer_TX[5] = (UINT8)(u16RegNumTemp & 0x00FF);		//����
		//�Ӹ�CRCЧ��
		u16SciTemp = Sci_CRC16RTU(u16Buffer_TX, 6);//CRCУ��
		//u16SciTemp = CRCChk(u16Buffer_TX, 6);	//CRCУ��
		u16Buffer_TX[6] = (UINT8)(u16SciTemp & 0x00FF);//&0x00ff��Ҫ�Ͱ�λ
		u16Buffer_TX[7] = (UINT8)(u16SciTemp >> 8);//>>8��Ҫ�߰�λ
	
		//LCD_Sci_TX();//����һ��ָ��
	for(i = 0;i < 8;i++) {
            while(!((USART1->ISR)&(1<<7)));
            USART1->TDR = u16Buffer_TX[i];
		}
		Sci1_FaultChk();
}



