#include "main.h"
#include "bsp.h"
#include "modbus_host.h"


void InitVar(void);
void InitDevice(void);


void LCDclear(void);

void TM1621D_stop(void);


static void DispMenu(void);

PRINT_MODS_T g_tPrint;

#if 0
int main(void)
{
	uint8_t ucKeyCode;
	int16_t count = 0;
	uint8_t fRefresh = 0;
	uint8_t read;

	InitDevice(); // 初始化外设，这两个函数的位置需要斟酌一下，现在换回去先
	InitVar();	  // 初始化变量

	//InitWakeUp_Base();

	LCDclear();

	bsp_StartAutoTimer(0, 200);
	bsp_StartAutoTimer(1, 1000*10); 

	while (1)
	{
		bsp_Idle();

		App_SysTime();

		if (bsp_CheckTimer(0))
		{
			MCUO_DEBUG_LED1 = ~MCUO_DEBUG_LED1;
			
			printf("hello\n");	
		}

		if (bsp_CheckTimer(1))
		{

			printf("系统运行时钟          %d Hz\r", SystemCoreClock);

			_0x06_sleep_stopBMS();
			bsp_DelayMS(1);
			//stoptest();
			stopTEST();

			//Sys_StopMode();

		}

		ucKeyCode = bsp_GetKey();
		if (ucKeyCode > 0)
		{
			/* 有键按下 */
			switch (ucKeyCode)
			{
			case KEY_DOWN_K1:
				printf("k1 down");
				//_0x06_sleep_stopBMS();
				break;
			case KEY_LONG_K1:
				//_0x06_sleep_L3_sleep();
				_0x06_sleep_stopBMS();
				bsp_DelayMS(1);

				stopTEST();
				printf("long long long\n");
				
				break;
			case KEY_UP_K1:
				printf("UP UP UP\n");
				break;

			default:
				break;
			}
		}

	
		
#if 1

		// App_SciMasterStation(); 

#ifdef _SLEEP
		App_SleepDeal();
#endif

#endif

		Feed_IWatchDog;
	}
}
#else
int main(void)
{
	uint8_t ucKeyCode;				/* 按键代码 */
	MSG_T ucMsg;					/* 消息代码 */

	InitDevice();

	bsp_Init();						/* 硬件初始化 */

	LCDclear();

	DispMenu();						/* 打印寄存器的值 */

	bsp_StartAutoTimer(0, 200);
	bsp_StartAutoTimer(1, 1000*10); 

	/* 进入主程序循环体 */
	while (1)
	{
		bsp_Idle();					/* 这个函数在bsp.c文件。用户可以修改这个函数实现CPU休眠和喂狗 */
		
		if (bsp_CheckTimer(0))
		{
			MCUO_DEBUG_LED1 = ~MCUO_DEBUG_LED1;

//			if (MODH_WriteParam_06H(REG_MOS_Relay_ON, 3) == 1) ;
//			else
//			{
//				// printf("read error")
//				DEBUG_INFO("write 06 error");
//			}
		}

		if (bsp_CheckTimer(1))
		{

			//printf("系统运行时钟          %d Hz\r", SystemCoreClock);
			// if (MODH_WriteParam_06H(REG_MOS_Relay_ON, 3) == 1) ;
			// else
			// {
			// 	// printf("read error")
			// 	DEBUG_INFO("write 06 error");
			// }
		}

		if (bsp_GetMsg(&ucMsg))		/* 读取消息代码 */
		{
			switch (ucMsg.MsgCode)
			{
				case MSG_MODS:		
					DispMenu();		/* 打印实验结果 */
					break;
				
				default:
					break;
			}
		}
	
		/* 按键滤波和检测由后台systick中断服务程序实现，我们只需要调用bsp_GetKey读取键值即可。 */
		ucKeyCode = bsp_GetKey();	/* 读取键值, 无键按下时返回 KEY_NONE = 0 */
		if (ucKeyCode != KEY_NONE)
		{
			bsp_PutMsg(MSG_MODS, 0);
			
			switch (ucKeyCode)
			{			
				case KEY_DOWN_K1:				
					// if (MODH_ReadParam_01H(REG_D01, 4) == 1) ;else ;
					// break;

					if (MODH_WriteParam_06H(RS485_CMD_ADDR_SYSTEM_FUNCTION_ON, 0x000b) == 1) ;else ;
					break;
				
				case KEY_UP_K1:				/* K2键按下 */
					if (MODH_ReadParam_03H(REG_READ_DATA, 15) == 1) ;else ;
					break;
				
				case KEY_LONG_K1:				/* K3键按下 */
					{
						uint8_t buf[10];
						uint8_t index = 0;
						// buf[index++] = 0x01;
						// buf[1] = 0x02;
						// buf[2] = 0x03;
						// buf[3] = 0x04;
						buf[index++] = 0x10;
						buf[index++] = 0x04;
						buf[index++] = 0x10;
						buf[index++] = 0x66;
						buf[index++] = 0x10;
						buf[index++] = 0x9a;
						buf[index++] = 0x10;
						buf[index++] = 0x36;
						buf[index++] = 0x00;
						buf[index++] = 0x20;
						if (MODH_WriteParam_10H(REG_CELL_OVER_VOL, 5, buf) == 1) ;else ;
					}
					break;
				// case JOY_DOWN_U:				/* 摇杆UP键弹起 */
				// 	if (MODH_WriteParam_06H(REG_P01, 1) == 1) ;else ;
				// 	break;
				
				// case JOY_DOWN_D:				/* 摇杆DOWN键按下 */
				// 	if (MODH_WriteParam_06H(REG_P01, 0) == 1) ;else ;
				// 	break;
				
				// case JOY_DOWN_L:				/* 摇杆LEFT键弹起 */
				// 	if (MODH_WriteParam_05H(REG_D01, 1) == 1) ;else ;
				// 	break;
				
				// case JOY_DOWN_R:				/* 摇杆RIGHT键弹起 */
				// 	if (MODH_WriteParam_05H(REG_D01, 0) == 1) ;else ;
				// 	break;
				
				// case JOY_DOWN_OK:				/* 摇杆OK键按下 */
				// 	if (MODH_ReadParam_02H(REG_T01, 3) == 1) ;else ;
				// 	break;

				// case JOY_UP_OK:					/* 摇杆OK键弹起 */
				// 	if (MODH_ReadParam_04H(REG_A01, 1) == 1) ;else ;	
				// 	break;

				default:
					/* 其它的键值不处理 */
					break;
			}
		}
	}
}

#endif
void TM1621D_stop()
{

	// TM1621D_CS(1);
	// TM1621D_WR(1);
	// TM1621D_DATA(1);
	// //HAL_Delay(1);

	// __delay_ms(10);

	// SendCmd(TIMER_DIS);
	// SendCmd(WDT_DIS);
	SendCmd(BIAS);	 // 1/3偏压 4公共口
	SendCmd(RC);	 // 内部RC振荡
	SendCmd(SYSDIS); // 关系统振荡器和LCD偏压发生器
	// SendCmd(SYSEN);	 // 打开系统振荡器
	SendCmd(LCDOFF); // 开LCD偏压
					 // SendCmd(TIMER_EN);
}


void LCD_Init()
{
	TM1621D_init();
}
// 这个初始化函数很容易出问题
void InitDevice(void)
{
	SystemInit(); // 直接调用就可以了。
				  // A，先reset所有配置，使用HSI(8MHz)运行。reset默认是使用HSI运行。
				  // B，调用SetSysClock()，默认使用8MHz外部晶振，然后六倍频输出，倍频输出不能超过48MHz(我使用12MHz，所以改为4倍频)
				  // C，如果倍频失败，会有个else语句让我改，输出一些标志位，我目前没改
				  // D，当从待机和停止模式返回或用作系统时钟的HSE 振荡器发生故障时，该位由硬件置来启动HSI 振荡器。
				  // E，言下之意，进入待机模式要关外部晶振，回来，先用HSI运行，然后开启外部晶振和倍频。
				  // F，还有一个切换时钟的函数，SystemCoreClockUpdate()，使用条件后面了解。
				  // E，外部晶振修改的话，改主头文件HSE_VALUE的值，会影响串口波特率。
				  // SystemCoreClockUpdate();
				  // SystemCoreClockUpdate();

	InitIO();
	// InitTimer();
	// // InitDelay();

	// // InitRTC();
	// //InitUSART1_Master();

	// //InitUSART2_Master();

	LCD_Init();

	bsp_Init();


#ifndef _DEBUG_
	Init_IWDG();
#endif 
}

void InitVar(void)
{

	// g_stLcdDispData.g_u16RunState = 0;
	// g_stLcdDispData.g_u16VoutDisp = 0;
	// g_stLcdDispData.g_u16IoutDisp = 500;
	// g_stLcdDispData.g_u16Temp1 = 40;
	// g_stLcdDispData.g_u16Soc = 0;
}


void LCDclear(void)
{
	uint8_t _delay = 2;

	lcd_display(0, 0xFF);
	__delay_ms(_delay);
	lcd_display(2, 0xFF);
	__delay_ms(_delay);
	lcd_display(4, 0xFF);
	__delay_ms(_delay);
	lcd_display(6, 0xFF);
	__delay_ms(_delay);
	lcd_display(8, 0xFF);
	__delay_ms(_delay);
	lcd_display(10, 0xFF);
}

static void DispMenu(void)
{	
	uint8_t i;
	
	printf("\n\r");
	printf("\33[K");						/* 清除从光标到行尾的内容 */ 
	
	printf(" 发送的命令 : 0x");				/* 打印发送命令 */
	for (i = 0; i < g_tPrint.Txlen; i++)
	{
		printf(" %02X", g_tPrint.TxBuf[i]);
	}

	printf("\n\r");
	printf("\33[K");						/* 清除从光标到行尾的内容 */ 
	
	printf(" 接收的命令 : 0x");				/* 打印接收命令 */
	for (i = 0; i < g_tPrint.Rxlen; i++)
	{
		printf(" %02X", g_tPrint.RxBuf[i]);
	}
	
	printf("\n\r");
	printf("\33[3A");						/* 光标上移3行 */
}
