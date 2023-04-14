#ifndef SYSTEM_H
#define SYSTEM_H

typedef struct _16_Bits_Struct {
    UINT16 bit0 : 1;
    UINT16 bit1 : 1;
    UINT16 bit2 : 1;
    UINT16 bit3 : 1;
    UINT16 bit4 : 1;
    UINT16 bit5 : 1;
    UINT16 bit6 : 1;
    UINT16 bit7 : 1;
    UINT16 bit8 : 1;
    UINT16 bit9 : 1;
    UINT16 bit10 : 1;
    UINT16 bit11 : 1;
    UINT16 bit12 : 1;
    UINT16 bit13 : 1;
    UINT16 bit14 : 1;
    UINT16 bit15 : 1;
} Bits_16_TypeDef;

#define PORT_OUT_GPIOA    ((Bits_16_TypeDef *)(&(GPIOA->ODR)))
#define PORT_OUT_GPIOB    ((Bits_16_TypeDef *)(&(GPIOB->ODR)))
#define PORT_OUT_GPIOC    ((Bits_16_TypeDef *)(&(GPIOC->ODR)))
#define PORT_OUT_GPIOF    ((Bits_16_TypeDef *)(&(GPIOF->ODR)))

#define PORT_IN_GPIOA    ((Bits_16_TypeDef *)(&(GPIOA->IDR)))
#define PORT_IN_GPIOB    ((Bits_16_TypeDef *)(&(GPIOB->IDR)))
#define PORT_IN_GPIOC    ((Bits_16_TypeDef *)(&(GPIOC->IDR)))
#define PORT_IN_GPIOF    ((Bits_16_TypeDef *)(&(GPIOF->IDR)))

//IO -------- OUTPUT
//MCUO_DEBUG_LED1
#define MCUO_DEBUG_LED1     	(PORT_OUT_GPIOB->bit2)       	//LED

#define MCUO_LED_MOD1     	(PORT_OUT_GPIOB->bit3)       	//LED
#define MCUO_LED_STA1     	(PORT_OUT_GPIOB->bit4)       	//LED
#define MCUO_LED_MOD2     	(PORT_OUT_GPIOB->bit5)       	//LED

#define MCUO_LCD_CS     	(PORT_OUT_GPIOA->bit4)       	//LCD
#define MCUO_LCD_WR     	(PORT_OUT_GPIOA->bit8)       	//LCD
#define MCUO_LCD_RS     	(PORT_OUT_GPIOB->bit11)       	//LCD
#define MCUO_LCD_RES     	(PORT_OUT_GPIOB->bit10)       	//LCD
#define MCUO_LCD_RD     	(PORT_OUT_GPIOF->bit7)       	//LCD

#define MCUO_E2PR_WP		(PORT_OUT_GPIOB->bit8)       	//E2PR-WP



//IO -------- INPUT_DET
#define READ_CHG_POS   (uint16_t)(GPIOB->IDR&GPIO_Pin_6)  	//读取充电管
#define READ_DSG_POS   (uint16_t)(GPIOB->IDR&GPIO_Pin_8)  	//读取放电管
#define READ_EXT_CB    (uint16_t)(GPIOB->IDR&GPIO_Pin_0)  	//读取第十六串均衡状况
#define READ_DI_ENI    (uint16_t)(GPIOB->IDR&GPIO_Pin_10)  	//预留输入口，L有效
#define READ_CHG_ON    (uint16_t)(GPIOB->IDR&GPIO_Pin_1)  	//电枪在线状态信号输入，L表示在线
#define READ_MCU_WKUP1 (uint16_t)(GPIOA->IDR&GPIO_Pin_0)  	//WKUP1电平读取，不会用到
#define READ_MCU_CBC   (uint16_t)(GPIOA->IDR&GPIO_Pin_1)  	//CBC电平读取，不会用到


#define MCUI_SW2		(PORT_IN_GPIOF->bit6)


union SYS_TIME{
    UINT8   all;
    struct StatusSysTimeFlagBit
    {
        UINT8 b1Sys10msFlag         :1;
        UINT8 b1Sys20msFlag         :1;
		UINT8 b1Sys200ms1Flag       :1;
		UINT8 b1Sys200msFlag        :1;
		
        UINT8 b1Sys10ms1Flag        :1;
        UINT8 b1Sys10ms2Flag        :1;
        UINT8 b1Sys10ms3Flag        :1;
        UINT8 b1Sys10ms4Flag        :1;
     }bits;
};


union On_OFF_func {
    UINT8   all;
    struct On_OFF_funcFlag {
		UINT8 b1Balanced			:1;
		UINT8 b1PowerON 			:1;
		UINT8 b1Heat                :1;
		UINT8 b1AFE_wakeup          :1;
		UINT8 b1StartUpBMS			:1;
		UINT8 b1SleepFlag			:1;
		UINT8 b2Rcved				:2;
     }bits;
};

#define _DEBUG_

#ifdef _DEBUG_
   #define Feed_IWatchDog 
#else
    #define Feed_IWatchDog IWDG_ReloadCounter()
#endif // _DEBUG_

//#define Feed_IWatchDog IWDG_ReloadCounter()

extern volatile union SYS_TIME g_st_SysTimeFlag;
extern volatile union On_OFF_func On_OFF_funcflag;

void InitDelay(void);
void __delay_us(UINT32 nus);
void __delay_ms(UINT16 ms);
void InitIO(void);
void InitTimer(void);
void App_SysTime(void);
void Init_IWDG(void);
void Sys_Standby(void);
void InitWKUP_Key(void);
void IWDG_Feed(void);

extern UINT8 MasterSlave_Select;

#endif	/* SYSTEM_H */

