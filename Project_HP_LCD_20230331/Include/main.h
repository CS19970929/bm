#ifndef MAIN_H
#define MAIN_H

//#define USE_STDPERIPH_DRIVER	//没有这个报一大堆assert_param()函数的错误，要在工程文件中配置

#include "stm32f0xx.h"
#include "stm32f0xx_it.h"			//里面有一些硬件错误之类的中断，还是需要的

#include "stdio.h"

//#include "stm32f0xx_conf.h"
#include "Sci_Master.h"
#include "Sci_Slave.h"
#include "I2C.h"
#include "System.h"
#include "RTC.h"
#include "PubFunc.h"
#include "Spi.h"
#include "EEPROM.h"
#include "LCD.h"

#include "SleepDeal.h"
#include "Flash.h"

#include "TM1621D.h"



#define  TRUE    1
#define  FALSE   0

#define APPLICATION_ADDRESS     (uint32_t)0x08001C00
#define UPDNLMT16(Var,Max,Min)	{(Var)=((Var)>=(Max))?(Max):(Var);(Var)=((Var)<=(Min))?(Min):(Var);}

#define S2U(x)   (*((volatile UINT16*)(&(x))))

#define DEBUG_LINE() 																												\
  printf("Log: [%s:%s] line = %d\n", __FILE__, __func__, __LINE__)
#define DEBUG_INFO(fmt, ...)                                                \
  printf("Log: [%s:%s] line = %d\n" fmt "\n", __FILE__, __func__, __LINE__, \
         ##__VA_ARGS__)

//10ms时基计数器
#define DELAYB10MS_0MS       ((UINT16)0)            //0ms
#define DELAYB10MS_30MS      ((UINT16)3)            //30ms
#define DELAYB10MS_50MS      ((UINT16)5)            //50ms
#define DELAYB10MS_100MS     ((UINT16)10)           //100ms
#define DELAYB10MS_200MS     ((UINT16)20)           //200ms
#define DELAYB10MS_250MS     ((UINT16)25)           //250ms
#define DELAYB10MS_500MS     ((UINT16)50)           //500ms
#define DELAYB10MS_1S        ((UINT16)100)          //1s
#define DELAYB10MS_1S5       ((UINT16)150)          //1.5s
#define DELAYB10MS_2S        ((UINT16)200)          //2s
#define DELAYB10MS_2S5       ((UINT16)250)          //2.5s
#define DELAYB10MS_3S        ((UINT16)300)          //3s
#define DELAYB10MS_4S        ((UINT16)400)          // 4s
#define DELAYB10MS_5S        ((UINT16)500)          // 5s
#define DELAYB10MS_10S       ((UINT16)1000)         //10s
#define DELAYB10MS_30S       ((UINT16)3000)         //30s



#define RTC_Clock 			32000		//32KHz LSE

#define MCU_RESET()	NVIC_SystemReset()

#define _RS232_1          //如名字，还没改好，用串口1
//#define _RS232_2
//#define _IAP
//#define _SLEEP


typedef struct
{
	uint8_t Rxlen;
	char RxBuf[20];
	uint8_t Txlen;
	char TxBuf[20];
}PRINT_MODS_T;

extern PRINT_MODS_T g_tPrint;


void InitSystemWakeUp(void);
void TM1621D_stop(void);

void InitDevice(void);

#endif	/* MAIN_H */

