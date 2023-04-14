#include "main.h"
 
__IO uint32_t  sEETimeout = sEE_LONG_TIMEOUT;

UINT8 	sEE_I2CFaultcnt = 0;

void InitE2PROM(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	//PB15_E2PR_WP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			//开漏输出		只需要保留这句话便出问题
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
uint32_t sEE_TIMEOUT_UserCallback(void) {
	sEE_I2CFaultcnt++;
	return -1;
}
*/
//以上便为出现HardFault_Handler()的原因(这货在it.c里面，说明这个.c文件还是需要的)，return -1(实际代码执行是运行了这段代码，才有，不运行不会有)
//但是返回类型为uint32_t，导致出现硬件错误，debug不了，具体怎么找出来，看
INT8 sEE_TIMEOUT_UserCallback(void) {
	sEE_I2CFaultcnt++;
	return -1;
}


INT8 sEE_WaitEepromStandbyState(void)      
{
  __IO uint32_t sEETrials = 0;
  
  MCUO_E2PR_WP = 0;
  /* Keep looping till the slave acknowledge his address or maximum number 
  of trials is reached (this number is defined by sEE_MAX_TRIALS_NUMBER define
  in stm32373c_eval_i2c_ee.h file) */
  
  /* Configure CR2 register : set Slave Address and end mode */
  I2C_TransferHandling(sEE_I2C, sEEAddress, 0, I2C_AutoEnd_Mode, I2C_No_StartStop);  
  
  do
  { 
    /* Initialize sEETimeout */
    sEETimeout = sEE_FLAG_TIMEOUT;
    
    /* Clear NACKF */
    I2C_ClearFlag(sEE_I2C, I2C_ICR_NACKCF | I2C_ICR_STOPCF);
    
    /* Generate start */
    I2C_GenerateSTART(sEE_I2C, ENABLE);
    
    /* Wait until timeout elapsed */
    while (sEETimeout-- != 0); 
    
    /* Check if the maximum allowed numbe of trials has bee reached */
    if (sEETrials++ == sEE_MAX_TRIALS_NUMBER)
    {
      /* If the maximum number of trials has been reached, exit the function */
      return sEE_TIMEOUT_UserCallback();
    }
  }
  while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_NACKF) != RESET);
  
  /* Clear STOPF */
  I2C_ClearFlag(sEE_I2C, I2C_ICR_STOPCF);
  MCUO_E2PR_WP = 1;
  /* Return sEE_OK if device is ready */
  return sEE_OK;
}


//后续维护人员禁止使用这个函数
INT8 EEPROM_Write(UINT16 addr, UINT8 val) {
    Feed_IWatchDog;                    //看门狗定时器清零
	/*
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_FLAG_BUSY) != RESET) {	//等待总线不忙
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/
	//__delay_ms(5);
	//sEE_WaitEepromStandbyState();			//写必须要这个，读可以不用，但为了保险起见，加上，后续观察这个函数的耗时，对别的通讯动作，时序影响
	I2C_TransferHandling(sEE_I2C, sEEAddress, 2, I2C_Reload_Mode, I2C_Generate_Start_Write);	//I2C_Reload_Mode，传完两个地址能继续传

	/* Send MSB of memory address */
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//I2Cx_TXDR寄存器为空置1，跳出循环，则写
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 1;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	I2C_SendData(sEE_I2C, (uint8_t)((addr & 0xFF00) >> 8));  


	/* Send LSB of memory address  */
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 2;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	I2C_SendData(sEE_I2C, (uint8_t)(addr & 0x00FF));

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TCR) == RESET) {		//等待发送完成标志，由硬件清0，I2C_Reload_Mode才会产生TC，但是不会带stop信号，能继续传
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 3;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	/* Update CR2 : set Slave Address , set write request, generate Start and set end mode */
	//不产生起始或者开始信号，在Reload_Mode为1时，Start也没用，也即不会产生起始信号和发送地址，只是更新某些功能
	I2C_TransferHandling(sEE_I2C, sEEAddress, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);	//自动结束，产生stop信号

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//出现在发送中断，则发
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 4;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	I2C_SendData(sEE_I2C, val);
		
	/* Wait until STOPF flag is set */
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_STOPF) == RESET) {
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 5;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	I2C_ClearFlag(sEE_I2C, I2C_ICR_STOPCF);
	
	__delay_ms(5);								//到处想，直接这里一加完美了，巧妙得一匹
	return sEE_OK;
	
}


//后续维护人员禁止使用这个函数
//这个地方，不能return sEE_TIMEOUT_UserCallback()的值！会出错！
UINT8 EEPROM_Read(UINT16 addr) {

    UINT8 u8tmp = 0xff;

	/*
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_FLAG_BUSY) != RESET) {	//等待总线不忙
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/

	//sEE_WaitEepromStandbyState();
	//I2C_SoftEnd_Mode，是在地址数据发送以后产生一个restart信号，以表示用来读取E2的数据
	I2C_TransferHandling(sEE_I2C, sEEAddress, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//I2Cx_TXDR寄存器为空置1，跳出循环，则写
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 6;
			sEE_TIMEOUT_UserCallback();
			return 0;
		}
	}
	I2C_SendData(sEE_I2C, (UINT8)((addr & 0xFF00) >> 8));


	sEETimeout = sEE_LONG_TIMEOUT;  
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 7;
			sEE_TIMEOUT_UserCallback();
			return 0;
		}
	}
	I2C_SendData(sEE_I2C, (UINT8)(addr & 0x00FF));


	/*	//去掉了反而变好了.....哪里return点哪里，无语了
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TC) == RESET) {		//等待发送完成标志，由硬件清0
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/
	/* Update CR2 : set Slave Address , set read request, generate Start and set end mode */
	//I2C_AutoEnd_Mode，NBYTES 个数据传输完后，会自动发送一个停止条件。
	//I2C_Generate_Start_Read应该会再次发送读地址 0xA0 + 0x01 = 0xA1
	I2C_TransferHandling(sEE_I2C, sEEAddress, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_RXNE) == RESET) {		//收到非空，则读
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 8;
			sEE_TIMEOUT_UserCallback();
			return 0;
		}
	}
	u8tmp= I2C_ReceiveData(sEE_I2C);
	
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_STOPF) == RESET) {
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 9;
			sEE_TIMEOUT_UserCallback();
			return 0;
		}
	}
	I2C_ClearFlag(sEE_I2C, I2C_ICR_STOPCF);

    return u8tmp;         //返回EEDATA存储的数据
}


//这个和EEPROM_Read()的返回值类型不能搞错！
//这个函数在出现静电搞错的时候，运行下面一大堆会不会又出现超时错误？可以复现，先留着
UINT16 ReadEEPROM_Word_NoZone(UINT16 addr) {

	UINT16 tmp16a;
	UINT8  tmp8a,tmp8b;	
	
	tmp8a = EEPROM_Read(addr);		                        //读取低位地址A对应的数据
	tmp8b = EEPROM_Read(addr+1);	                        //读取高位地址A+1对应的数据
	tmp16a = tmp8b;
	tmp16a = (tmp16a<<8) | tmp8a;	                        //数据存储

	return tmp16a;

}



//主要调这个，加了几句话
INT8 WriteEEPROM_Word_NoZone(UINT16            addr, UINT16 data)
{
    UINT8  tmp8a;
    UINT8  WriteCounter = 0;		            /* EEPROM写计数----*/
    UINT16 tmp_addr;
    UINT16 tmp16;
	INT8 result = 0;
	tmp_addr = addr;						//移植忘了这句话

	MCUO_E2PR_WP = 0;

	WriteCounter = 0;
	do {
		tmp8a = data & 0xff;				//获取数据的低8位
		EEPROM_Write(tmp_addr, tmp8a);		//数据的低8位写入EEPROM
		tmp8a = data >> 8;					//获取数据的高8位
		//__delay_ms(5);
		EEPROM_Write(tmp_addr+1, tmp8a);	//数据的高8位写入EEPROM
	
		//读取写入的数据
		sEE_WaitEepromStandbyState();
		tmp8a = EEPROM_Read(tmp_addr);		//获取刚存入EEPROM的低8位数据
		tmp16 = tmp8a;
		tmp8a = EEPROM_Read(tmp_addr+1);	//获取刚存入EEPROM的高8位数据
		tmp16 = (tmp8a<<8) |tmp16 ; 		//存储读到的数据于变量tmp16
	
		WriteCounter++;
		if(WriteCounter > 2) {				/*判断tmp16 != data的计数*/
			result = -1;
			++sEE_I2CFaultcnt;
			break;
		}
	}while(tmp16 != data);
	//__delay_ms(5);						//防止没写完就失效，另外那个就不用，因为前面已经read了
	MCUO_E2PR_WP = 1;	

	return result;
}



//这个和EEPROM_Read()的返回值类型不能搞错！
//这个函数在出现静电搞错的时候，运行下面一大堆会不会又出现超时错误？可以复现，先留着
UINT16 ReadEEPROM_Word(UINT16 addr) {

	UINT16 tmp16a,tmp16b,tmp16c;
	UINT8  tmp8a,tmp8b;	
	UINT16 addrB,addrC;
	
	addrB = addr + BZONE;
	addrC = addr + CZONE;
	//sEE_WaitEepromStandbyState();
	tmp8a = EEPROM_Read(addr);		                        //读取低位地址A对应的数据
	tmp8b = EEPROM_Read(addr+1);	                        //读取高位地址A+1对应的数据
	tmp16a = tmp8b;
	tmp16a = (tmp16a<<8) | tmp8a;	                        //数据存储
	
	tmp8a = EEPROM_Read(addrB);                            //读取低位地址B对应的数据
	tmp8b = EEPROM_Read(addrB+1);	                        //读取高位地址B+1对应的数据
	tmp16b = tmp8b;
	tmp16b = (tmp16b<<8) | tmp8a;	                        //数据存储
	
	tmp8a = EEPROM_Read(addrC);                            //读取低位地址C对应的数据
	tmp8b = EEPROM_Read(addrC+1);	                        //读取高位地址C+1对应的数据
	tmp16c = tmp8b;
	tmp16c = (tmp16c<<8) | tmp8a;	                        //数据存储
	
	if (tmp16a == tmp16b) {                                 //a == b
        if (tmp16a != tmp16c) {                              //a != c      
            WriteEEPROM_Word_NoZone(addrC, tmp16a);
        }
        return tmp16a;
    }
    else {
        if (tmp16b == tmp16c) {                              //b==c  a != b
            WriteEEPROM_Word_NoZone(addr, tmp16b);
            return tmp16b;
        }	
        else {
            if (tmp16a == tmp16c) {                          //a == c, a != b
                WriteEEPROM_Word_NoZone(addrB, tmp16a);
                return tmp16a;
            }
            else {                                           //a != b, b != c, c != a
                WriteEEPROM_Word_NoZone(addr, tmp16a);
				WriteEEPROM_Word_NoZone(addrB, tmp16a);
				WriteEEPROM_Word_NoZone(addrC, tmp16a);
                return tmp16a;                        //tmp16a,tmp16b,tmp16c返回默认值，返回第一个值？
            }		
        }
    }
}



//这个函数有问题，BC区写不进去，不知道为什么
INT8 WriteEEPROM_Word2(UINT16            addr, UINT16 data)
{
    UINT8  i,tmp8a;
    UINT8  WriteCounter = 0;		            /* EEPROM写计数----*/
    UINT16 tmp_addr;
    UINT16 tmp16;
	tmp_addr = addr;
	
	MCUO_E2PR_WP = 0;
	for(i = 0; i < 3; i++)
	{
        WriteCounter = 0;
        do
        {
            tmp8a = (UINT8)(data & 0xff);                //获取数据的低8位
            EEPROM_Write(tmp_addr, tmp8a);      //数据的低8位写入EEPROM
            tmp8a = (UINT8)(data >> 8);                  //获取数据的高8位
            //__delay_ms(5);
            EEPROM_Write(tmp_addr+1, tmp8a);    //数据的高8位写入EEPROM

            //读取写入的数据
            sEE_WaitEepromStandbyState();
            tmp8a = EEPROM_Read(tmp_addr);      //获取刚存入EEPROM的低8位数据
            tmp16 = tmp8a;
            tmp8a = EEPROM_Read(tmp_addr+1);    //获取刚存入EEPROM的高8位数据
            tmp16 = (tmp8a<<8) |tmp16 ;         //存储读到的数据于变量tmp16
			//__delay_ms(5);
            WriteCounter++;
            if(WriteCounter > 2)                /*判断tmp16 != data的计数*/
            {
            	++sEE_I2CFaultcnt;
                break;
            }
        }while(tmp16 != data);
		
        //地址加0x100写入
        tmp_addr = tmp_addr + BZONE;
    }
	MCUO_E2PR_WP = 1;
		return 0;//--
}

//主要调这个，加了几句话
INT8 WriteEEPROM_Word(UINT16 addr, UINT16 data)
{
	INT8 result = 0;
	result += WriteEEPROM_Word_NoZone(addr, data);
	result += WriteEEPROM_Word_NoZone(addr + BZONE, data);
	result += WriteEEPROM_Word_NoZone(addr + CZONE, data);
	return result;
}





//问题找出来，就是BC区写不进去，返回0xFF
void EEPROM_test(void) {

}


