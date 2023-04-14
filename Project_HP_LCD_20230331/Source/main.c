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

	InitDevice(); // ��ʼ�����裬������������λ����Ҫ����һ�£����ڻ���ȥ��
	InitVar();	  // ��ʼ������

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

			printf("ϵͳ����ʱ��          %d Hz\r", SystemCoreClock);

			_0x06_sleep_stopBMS();
			bsp_DelayMS(1);
			//stoptest();
			stopTEST();

			//Sys_StopMode();

		}

		ucKeyCode = bsp_GetKey();
		if (ucKeyCode > 0)
		{
			/* �м����� */
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
	uint8_t ucKeyCode;				/* �������� */
	MSG_T ucMsg;					/* ��Ϣ���� */

	InitDevice();

	bsp_Init();						/* Ӳ����ʼ�� */

	LCDclear();

	DispMenu();						/* ��ӡ�Ĵ�����ֵ */

	bsp_StartAutoTimer(0, 200);
	bsp_StartAutoTimer(1, 1000*10); 

	/* ����������ѭ���� */
	while (1)
	{
		bsp_Idle();					/* ���������bsp.c�ļ����û������޸��������ʵ��CPU���ߺ�ι�� */
		
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

			//printf("ϵͳ����ʱ��          %d Hz\r", SystemCoreClock);
			// if (MODH_WriteParam_06H(REG_MOS_Relay_ON, 3) == 1) ;
			// else
			// {
			// 	// printf("read error")
			// 	DEBUG_INFO("write 06 error");
			// }
		}

		if (bsp_GetMsg(&ucMsg))		/* ��ȡ��Ϣ���� */
		{
			switch (ucMsg.MsgCode)
			{
				case MSG_MODS:		
					DispMenu();		/* ��ӡʵ���� */
					break;
				
				default:
					break;
			}
		}
	
		/* �����˲��ͼ���ɺ�̨systick�жϷ������ʵ�֣�����ֻ��Ҫ����bsp_GetKey��ȡ��ֵ���ɡ� */
		ucKeyCode = bsp_GetKey();	/* ��ȡ��ֵ, �޼�����ʱ���� KEY_NONE = 0 */
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
				
				case KEY_UP_K1:				/* K2������ */
					if (MODH_ReadParam_03H(REG_READ_DATA, 15) == 1) ;else ;
					break;
				
				case KEY_LONG_K1:				/* K3������ */
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
				// case JOY_DOWN_U:				/* ҡ��UP������ */
				// 	if (MODH_WriteParam_06H(REG_P01, 1) == 1) ;else ;
				// 	break;
				
				// case JOY_DOWN_D:				/* ҡ��DOWN������ */
				// 	if (MODH_WriteParam_06H(REG_P01, 0) == 1) ;else ;
				// 	break;
				
				// case JOY_DOWN_L:				/* ҡ��LEFT������ */
				// 	if (MODH_WriteParam_05H(REG_D01, 1) == 1) ;else ;
				// 	break;
				
				// case JOY_DOWN_R:				/* ҡ��RIGHT������ */
				// 	if (MODH_WriteParam_05H(REG_D01, 0) == 1) ;else ;
				// 	break;
				
				// case JOY_DOWN_OK:				/* ҡ��OK������ */
				// 	if (MODH_ReadParam_02H(REG_T01, 3) == 1) ;else ;
				// 	break;

				// case JOY_UP_OK:					/* ҡ��OK������ */
				// 	if (MODH_ReadParam_04H(REG_A01, 1) == 1) ;else ;	
				// 	break;

				default:
					/* �����ļ�ֵ������ */
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
	SendCmd(BIAS);	 // 1/3ƫѹ 4������
	SendCmd(RC);	 // �ڲ�RC��
	SendCmd(SYSDIS); // ��ϵͳ������LCDƫѹ������
	// SendCmd(SYSEN);	 // ��ϵͳ����
	SendCmd(LCDOFF); // ��LCDƫѹ
					 // SendCmd(TIMER_EN);
}


void LCD_Init()
{
	TM1621D_init();
}
// �����ʼ�����������׳�����
void InitDevice(void)
{
	SystemInit(); // ֱ�ӵ��þͿ����ˡ�
				  // A����reset�������ã�ʹ��HSI(8MHz)���С�resetĬ����ʹ��HSI���С�
				  // B������SetSysClock()��Ĭ��ʹ��8MHz�ⲿ����Ȼ������Ƶ�������Ƶ������ܳ���48MHz(��ʹ��12MHz�����Ը�Ϊ4��Ƶ)
				  // C�������Ƶʧ�ܣ����и�else������Ҹģ����һЩ��־λ����Ŀǰû��
				  // D�����Ӵ�����ֹͣģʽ���ػ�����ϵͳʱ�ӵ�HSE ������������ʱ����λ��Ӳ����������HSI ������
				  // E������֮�⣬�������ģʽҪ���ⲿ���񣬻���������HSI���У�Ȼ�����ⲿ����ͱ�Ƶ��
				  // F������һ���л�ʱ�ӵĺ�����SystemCoreClockUpdate()��ʹ�����������˽⡣
				  // E���ⲿ�����޸ĵĻ�������ͷ�ļ�HSE_VALUE��ֵ����Ӱ�촮�ڲ����ʡ�
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
	printf("\33[K");						/* ����ӹ�굽��β������ */ 
	
	printf(" ���͵����� : 0x");				/* ��ӡ�������� */
	for (i = 0; i < g_tPrint.Txlen; i++)
	{
		printf(" %02X", g_tPrint.TxBuf[i]);
	}

	printf("\n\r");
	printf("\33[K");						/* ����ӹ�굽��β������ */ 
	
	printf(" ���յ����� : 0x");				/* ��ӡ�������� */
	for (i = 0; i < g_tPrint.Rxlen; i++)
	{
		printf(" %02X", g_tPrint.RxBuf[i]);
	}
	
	printf("\n\r");
	printf("\33[3A");						/* �������3�� */
}
