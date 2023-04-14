/*********************************************************************
 *
 *                           TM1621D.h
 *
 *********************************************************************
 * ��    ��: TM1621D��������ͷ�ļ�
 * ����ƽ̨: MDK5
 * ��    ˾:
 * ��    ַ:
 * ����			    ����			ע��
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * ���ƴ������		22/02/11		ԭʼ�ļ�
 ********************************************************************/
 
#ifndef TM1621D_H
#define TM1621D_H

//#include "stm32f0xx_hal.h"

/******************TM1621Dģ�������*********************/
#define SYSDIS   0x00            //��ϵͳ������LCDƫѹ������
#define SYSEN    0x02    		//��ϵͳ����

#define LCDOFF   0x04         //��LCDƫѹ
#define LCDON    0x06        //��LCDƫѹ

#define TIMER_DIS  0x08
#define TIMER_EN   0x0c

#define WDT_DIS    0x0a
#define WDT_EN	   0x0e
#define CLR_WDT    0x1c
                                                         
#define RC       0x30       //�ڲ�RC��

#define BIAS     0x52      //1/3ƫѹ 4������   

#define Pin_CS    GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)       //Ƭѡ����
#define Pin_WR    GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)      //д��������
#define Pin_DATA  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3)      //������������/���

/****************************��������************************************/
void TM1621D_init(void);
void TM1621D_CS(unsigned char sta);
void TM1621D_WR(unsigned char sta);
void TM1621D_DATA(unsigned char sta);
void lcd_display(unsigned char digit,unsigned char number);

void SendCmd(unsigned char command);

extern unsigned char bit_col;
extern unsigned char Tab_Num[];
  
#endif 

