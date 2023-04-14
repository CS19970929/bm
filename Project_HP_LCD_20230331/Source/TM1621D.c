/*********************************************************************
 *
 *                           TM1621D.c
 *
 *********************************************************************
 * 描    述: TM1621D驱动函数
 * 开发平台: MDK5
 * 公    司:
 * 网    址:
 * 作者			    日期			注释
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 汉德星		23/03/31		原始文件
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
 *  函数: SendBit_1621(unsigned char date,unsigned char cnt)	  【兼容HT1621B程序】
 *  输入参数: Data,Cnt
 *  输出参数: none
 *  功能: 将 Data 的高 Cnt 位写入 TM1621D，高位在前
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
 *  函数: SendCmd(unsigned char command)		【兼容HT1621】
 *  输入参数: command
 *  输出参数: none
 *  功能: 向TM1621D写入命令
 ********************************************************************************************************/
void SendCmd(unsigned char command)
{
	TM1621D_CS(0);
	__NOP();

	SendBit_1621(0x80, 4);	  // 写入标志位码”100”和 9 位 command 命令，由于
	SendBit_1621(command, 8); // 本程序用到的命令最高位均为“0”，为了编程方便
							  // 直接将 command 的最高位写”0”，即将command当作8位命令码
	__NOP();
	TM1621D_CS(1);
}

/*********指定地址写入数据************/
void Write_TM1621D(unsigned char Addr, unsigned char Data)
{
	TM1621D_CS(0);
	SendBit_1621(0xa0, 3);		// 写入数据标志101
	SendBit_1621(Addr << 2, 6); // 写入地址数据
	SendBit_1621(Data, 8);		// 写入数据
	TM1621D_CS(1);
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

/********************************************************************************************************
 *  函数: lcd_display(unsigned char digit,unsigned char number)
 *  输入参数: digit,number
 *  输出参数: none
 *  功能: 在液晶指定位显示内容
 ********************************************************************************************************/
// void lcd_display(unsigned char digit,unsigned char number)
//{
//	unsigned char addr;
//
//	switch(digit)
//	{
//		/*连续写数据， 只需要向1621D写入起始SEG的地址即可，随后写入的数据，地址自动+1*/
//		case 1:
//			addr=0x13;	 //最高位的“8”在SEG19~SRG20,所以这里起始地址为0x13,即SEG19
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
//			addr=0x09;	 //最低位的“8”在SEG9~SEG10，所以这里的起始地址为0x09,即SEG9
//			break;
//		default :
//			break;
//	}

//	if(bit_col==1&&digit==5)				       //选择是否显示冒号
//		Write_TM1621D(addr,Tab_Num[number]|0x08);
//	else
//		Write_TM1621D(addr,Tab_Num[number]);

//}
void lcd_display(unsigned char digit, unsigned char number)
{

	Write_TM1621D(digit, number);
}

/*****************************初始化驱动TM1621D的IO口******************************/
void TM1621D_init()
{

	TM1621D_CS(1);
	TM1621D_WR(1);
	TM1621D_DATA(1);
	//HAL_Delay(1);

	//__delay_ms(10);			//加延时灯不闪了？系统死了
	//bsp_DelayMS(10);


	// SendCmd(TIMER_DIS);
	// SendCmd(WDT_DIS);
	SendCmd(BIAS);	 // 1/3偏压 4公共口
	SendCmd(RC);	 // 内部RC振荡
	SendCmd(SYSDIS); // 关系统振荡器和LCD偏压发生器
	SendCmd(SYSEN);	 // 打开系统振荡器
	SendCmd(LCDON);	 // 开LCD偏压
	// SendCmd(TIMER_EN);
}
