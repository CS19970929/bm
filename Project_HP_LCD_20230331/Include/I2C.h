#ifndef I2C_H
#define I2C_H

#define I2C_RW_W	0
#define I2C_RW_R	1

//IO方向设置
#define SDA_IN()  {GPIOA->MODER&=0xFFFFFFFF;GPIOA->MODER|=(UINT32)0<<24;}		//默认值为00，既不上拉也不下拉输入。
#define SDA_OUT() {GPIOA->MODER&=0xFCFFFFFF;GPIOA->MODER|=(UINT32)1<<24;}		//关于输出类型和输出速度，在默认值的Init函数配好，回来立刻生效

//IO操作函数	 
#define IIC_SCL    PORT_OUT_GPIOC->bit13 	//SCL
//#define IIC_SCL    PORT_OUT_GPIOA->bit8 	//SCL
#define IIC_SDA    PORT_OUT_GPIOA->bit12	//SDA
#define READ_SDA   (uint16_t)(GPIOA->IDR&GPIO_Pin_12)  //输入SDA 


void Init_I2C(void);
void IIC_Send_Byte(UINT8 txd);
UINT8 IIC_Read_Byte(unsigned char ack);

#endif	/* I2C_H */

