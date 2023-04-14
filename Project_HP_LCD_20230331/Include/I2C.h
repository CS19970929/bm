#ifndef I2C_H
#define I2C_H

#define I2C_RW_W	0
#define I2C_RW_R	1

//IO��������
#define SDA_IN()  {GPIOA->MODER&=0xFFFFFFFF;GPIOA->MODER|=(UINT32)0<<24;}		//Ĭ��ֵΪ00���Ȳ�����Ҳ���������롣
#define SDA_OUT() {GPIOA->MODER&=0xFCFFFFFF;GPIOA->MODER|=(UINT32)1<<24;}		//����������ͺ�����ٶȣ���Ĭ��ֵ��Init������ã�����������Ч

//IO��������	 
#define IIC_SCL    PORT_OUT_GPIOC->bit13 	//SCL
//#define IIC_SCL    PORT_OUT_GPIOA->bit8 	//SCL
#define IIC_SDA    PORT_OUT_GPIOA->bit12	//SDA
#define READ_SDA   (uint16_t)(GPIOA->IDR&GPIO_Pin_12)  //����SDA 


void Init_I2C(void);
void IIC_Send_Byte(UINT8 txd);
UINT8 IIC_Read_Byte(unsigned char ack);

#endif	/* I2C_H */

