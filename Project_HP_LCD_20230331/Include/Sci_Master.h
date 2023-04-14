#ifndef SCI_MASTER_H
#define SCI_MASTER_H

#define	RS485_BROADCAST_ADDR		(( UINT8 ) 0x00 )
#define	RS485_SLAVE_ADDR			(( UINT8 ) 0x01 )

extern uint8_t Tab_Num1[12] ;
extern uint8_t Tab_Num2[12] ;


#define RS485_STATE_IDLE	0
#define RS485_STATE_TX		1
#define RS485_STATE_WAIT	2
#define RS485_STATE_RXOK	3

enum RS485_CMD_E {
	RS485_CMD_READ_COILS = 1,
	RS485_CMD_READ_REGS = 3,
	RS485_CMD_WRITE_COIL = 5,
	RS485_CMD_WRITE_REG = 6,
	RS485_CMD_WRITE_REGS = 16
};


#define RS485_ADDR_RO_LCD   0xC000
#define RS485_ADDR_YS_LCD3				    	0xC003	//���Ƕ�������
#define RS485_ADDR_YS_LCD4				    0xC004	//���Ƕ�������
#define RS485_ADDR_YS_LCD5				    0xC005	//���Ƕ�������
#define RS485_ADDR_YS_LCD6				    0xC006	//���Ƕ�������
#define RS485_ADDR_YS_LCD7			    	0xC007	//���Ƕ�������
#define RS485_ADDR_YS_LCD9				    0xC009	//���Ƕ�������
#define RS485_ADDR_YS_LCDA				    0xC00A	//���Ƕ�������
#define RS485_ADDR_YS_LCDB				    0xC00B	//���Ƕ�������
#define RS485_ADDR_YS_LCDC				    0xC00C	//���Ƕ�������
#define RS485_ADDR_YS_LCDD				    0xC00D	//���Ƕ�������
#define RS485_ADDR_YS_LCDE				    0xC00E	//���Ƕ�������

#define RS485_CMD_ADDR_SYSTEM_FUNCTION_ON	0x1102

//#define RS485_CMD_ADDR_SYSTEM_FUNCTION_ON	0x1102

#define RXBUFLEN  				50
#define TXBUFNUM    			16
#define SciReceiveTime 			150


struct structDelayFlag
{
    uint8_t b1PowerStDelay      :1;     //�ϵ���ʱ50ms�������ݱ�־
    uint8_t bRcved     			:7;     //
};

typedef struct
{
	uint8_t bSendFlag     		:1;
	uint8_t bReceiveFinishFlag  :1;
	uint8_t bDataRight    		:1;
	uint8_t bSlaveAddrRdFlag    :1;

	uint8_t bSetParamRdFlag    	:1;
	uint8_t bPdtInfoRdFlag    	:1;
	uint8_t bPpctRdedFlag    	:1;
	uint8_t bReserve      		:1;
 }STATUS_Comm_BITS;

 typedef volatile union
 {
	 STATUS_Comm_BITS	 bits;

	 uint8_t		 	all;
 }STATUS_Comm_TypeDef;


 struct LCD_DISPLAY {
	 uint16_t g_u16IoutDisp; //�������A��*10
	 uint16_t g_u16VoutDisp; //�����ѹV��*1, Q0
	 uint16_t g_u16Temp1;		 // Q0 +50��
	 uint16_t g_u16RunState; //����״̬
	 uint16_t g_u16Soc; 	 //SOC 1 %
 };


extern struct LCD_DISPLAY g_stLcdDispData;
 
extern uint16_t SuspendFlag1;
extern uint16_t SuspendFlag2;
extern uint16_t voll;
 
void InitUSART1_Master(void);
void InitUSART2_Master(void);
void TimCnt(void);
void App_SciMasterStation(void);

void _0x06_sleep_L3_sleep(void);
void _0x06_sleep_stopBMS(void);
 
 void LCDshow5(int num);
 void ShowErr0(void);
 void LcdShow1_9(void);

#endif	/* SCI_MASTER_H */

