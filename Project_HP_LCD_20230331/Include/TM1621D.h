/*********************************************************************
 *
 *                           TM1621D.h
 *
 *********************************************************************
 * 描    述: TM1621D驱动函数头文件
 * 开发平台: MDK5
 * 公    司:
 * 网    址:
 * 作者			    日期			注释
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 飞云创意电子		22/02/11		原始文件
 ********************************************************************/
 
#ifndef TM1621D_H
#define TM1621D_H

//#include "stm32f0xx_hal.h"

/******************TM1621D模块命令定义*********************/
#define SYSDIS   0x00            //关系统振荡器和LCD偏压发生器
#define SYSEN    0x02    		//打开系统振荡器

#define LCDOFF   0x04         //关LCD偏压
#define LCDON    0x06        //开LCD偏压

#define TIMER_DIS  0x08
#define TIMER_EN   0x0c

#define WDT_DIS    0x0a
#define WDT_EN	   0x0e
#define CLR_WDT    0x1c
                                                         
#define RC       0x30       //内部RC振荡

#define BIAS     0x52      //1/3偏压 4公共口   

#define Pin_CS    GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)       //片选输入
#define Pin_WR    GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)      //写脉冲输入
#define Pin_DATA  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3)      //串行数据输入/输出

/****************************函数声明************************************/
void TM1621D_init(void);
void TM1621D_CS(unsigned char sta);
void TM1621D_WR(unsigned char sta);
void TM1621D_DATA(unsigned char sta);
void lcd_display(unsigned char digit,unsigned char number);

void SendCmd(unsigned char command);

extern unsigned char bit_col;
extern unsigned char Tab_Num[];
  
#endif 

