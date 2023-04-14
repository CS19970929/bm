/*********************************************************************
 *
 *                           TM1621D.c
 *
 *********************************************************************
 * ��    ��: TM1621D��������
 * ����ƽ̨: MDK5
 * ��    ˾:
 * ��    ַ:
 * ����			    ����			ע��
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * ������		23/03/31		ԭʼ�ļ�
 ********************************************************************/

#include "TM1621D.h"
#include "main.h"

#include "bsp.h"

void TM1621D_CS(unsigned char sta)
{
	if (sta == 1)
		//GPIO_WriteBit(GPIOB, , BitAction BitVal);
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void TM1621D_WR(unsigned char sta)
{
	if (sta == 1)
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

void TM1621D_DATA(unsigned char sta)
{
	if (sta == 1)
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}

/********************************************************************************************************
 *  ����: SendBit_1621(unsigned char date,unsigned char cnt)	  ������HT1621B����
 *  �������: Data,Cnt
 *  �������: none
 *  ����: �� Data �ĸ� Cnt λд�� TM1621D����λ��ǰ
 ********************************************************************************************************/
void SendBit_1621(unsigned char Data, unsigned char Cnt)
{
	unsigned char i;
	for (i = 0; i < Cnt; i++)
	{
		if ((Data & 0x80) == 0)
			TM1621D_DATA(0);
		else
			TM1621D_DATA(1);

		TM1621D_WR(0);
		__NOP();
		TM1621D_WR(1);

		Data <<= 1;
	}
}

/********************************************************************************************************
 *  ����: SendCmd(unsigned char command)		������HT1621��
 *  �������: command
 *  �������: none
 *  ����: ��TM1621Dд������
 ********************************************************************************************************/
void SendCmd(unsigned char command)
{
	TM1621D_CS(0);
	__NOP();

	SendBit_1621(0x80, 4);	  // д���־λ�롱100���� 9 λ command �������
	SendBit_1621(command, 8); // �������õ����������λ��Ϊ��0����Ϊ�˱�̷���
							  // ֱ�ӽ� command �����λд��0��������command����8λ������
	__NOP();
	TM1621D_CS(1);
}

/*********ָ����ַд������************/
void Write_TM1621D(unsigned char Addr, unsigned char Data)
{
	TM1621D_CS(0);
	SendBit_1621(0xa0, 3);		// д�����ݱ�־101
	SendBit_1621(Addr << 2, 6); // д���ַ����
	SendBit_1621(Data, 8);		// д������
	TM1621D_CS(1);
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

/********************************************************************************************************
 *  ����: lcd_display(unsigned char digit,unsigned char number)
 *  �������: digit,number
 *  �������: none
 *  ����: ��Һ��ָ��λ��ʾ����
 ********************************************************************************************************/
// void lcd_display(unsigned char digit,unsigned char number)
//{
//	unsigned char addr;
//
//	switch(digit)
//	{
//		/*����д���ݣ� ֻ��Ҫ��1621Dд����ʼSEG�ĵ�ַ���ɣ����д������ݣ���ַ�Զ�+1*/
//		case 1:
//			addr=0x13;	 //���λ�ġ�8����SEG19~SRG20,����������ʼ��ַΪ0x13,��SEG19
//			break;
//		case 2:
//			addr=0x11;
//			break;
//		case 3:
//			addr=0x0f;
//			break;
//		case 4:
//			addr=0x0d;
//			break;
//		case 5:
//			addr=0x0b;
//			break;
//		case 6:
//			addr=0x09;	 //���λ�ġ�8����SEG9~SEG10�������������ʼ��ַΪ0x09,��SEG9
//			break;
//		default :
//			break;
//	}

//	if(bit_col==1&&digit==5)				       //ѡ���Ƿ���ʾð��
//		Write_TM1621D(addr,Tab_Num[number]|0x08);
//	else
//		Write_TM1621D(addr,Tab_Num[number]);

//}
void lcd_display(unsigned char digit, unsigned char number)
{

	Write_TM1621D(digit, number);
}

/*****************************��ʼ������TM1621D��IO��******************************/
void TM1621D_init()
{

	TM1621D_CS(1);
	TM1621D_WR(1);
	TM1621D_DATA(1);
	//HAL_Delay(1);

	//__delay_ms(10);			//����ʱ�Ʋ����ˣ�ϵͳ����
	//bsp_DelayMS(10);


	// SendCmd(TIMER_DIS);
	// SendCmd(WDT_DIS);
	SendCmd(BIAS);	 // 1/3ƫѹ 4������
	SendCmd(RC);	 // �ڲ�RC��
	SendCmd(SYSDIS); // ��ϵͳ������LCDƫѹ������
	SendCmd(SYSEN);	 // ��ϵͳ����
	SendCmd(LCDON);	 // ��LCDƫѹ
	// SendCmd(TIMER_EN);
}
