#include "main.h"
 
__IO uint32_t  sEETimeout = sEE_LONG_TIMEOUT;

UINT8 	sEE_I2CFaultcnt = 0;

void InitE2PROM(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	//PB15_E2PR_WP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			//��©���		ֻ��Ҫ������仰�������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
uint32_t sEE_TIMEOUT_UserCallback(void) {
	sEE_I2CFaultcnt++;
	return -1;
}
*/
//���ϱ�Ϊ����HardFault_Handler()��ԭ��(�����it.c���棬˵�����.c�ļ�������Ҫ��)��return -1(ʵ�ʴ���ִ������������δ��룬���У������в�����)
//���Ƿ�������Ϊuint32_t�����³���Ӳ������debug���ˣ�������ô�ҳ�������
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


//����ά����Ա��ֹʹ���������
INT8 EEPROM_Write(UINT16 addr, UINT8 val) {
    Feed_IWatchDog;                    //���Ź���ʱ������
	/*
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_FLAG_BUSY) != RESET) {	//�ȴ����߲�æ
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/
	//__delay_ms(5);
	//sEE_WaitEepromStandbyState();			//д����Ҫ����������Բ��ã���Ϊ�˱�����������ϣ������۲���������ĺ�ʱ���Ա��ͨѶ������ʱ��Ӱ��
	I2C_TransferHandling(sEE_I2C, sEEAddress, 2, I2C_Reload_Mode, I2C_Generate_Start_Write);	//I2C_Reload_Mode������������ַ�ܼ�����

	/* Send MSB of memory address */
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//I2Cx_TXDR�Ĵ���Ϊ����1������ѭ������д
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
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TCR) == RESET) {		//�ȴ�������ɱ�־����Ӳ����0��I2C_Reload_Mode�Ż����TC�����ǲ����stop�źţ��ܼ�����
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 3;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	/* Update CR2 : set Slave Address , set write request, generate Start and set end mode */
	//��������ʼ���߿�ʼ�źţ���Reload_ModeΪ1ʱ��StartҲû�ã�Ҳ�����������ʼ�źźͷ��͵�ַ��ֻ�Ǹ���ĳЩ����
	I2C_TransferHandling(sEE_I2C, sEEAddress, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);	//�Զ�����������stop�ź�

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//�����ڷ����жϣ���
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
	
	__delay_ms(5);								//�����룬ֱ������һ�������ˣ������һƥ
	return sEE_OK;
	
}


//����ά����Ա��ֹʹ���������
//����ط�������return sEE_TIMEOUT_UserCallback()��ֵ�������
UINT8 EEPROM_Read(UINT16 addr) {

    UINT8 u8tmp = 0xff;

	/*
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_FLAG_BUSY) != RESET) {	//�ȴ����߲�æ
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/

	//sEE_WaitEepromStandbyState();
	//I2C_SoftEnd_Mode�����ڵ�ַ���ݷ����Ժ����һ��restart�źţ��Ա�ʾ������ȡE2������
	I2C_TransferHandling(sEE_I2C, sEEAddress, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//I2Cx_TXDR�Ĵ���Ϊ����1������ѭ������д
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


	/*	//ȥ���˷��������.....����return�����������
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TC) == RESET) {		//�ȴ�������ɱ�־����Ӳ����0
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/
	/* Update CR2 : set Slave Address , set read request, generate Start and set end mode */
	//I2C_AutoEnd_Mode��NBYTES �����ݴ�����󣬻��Զ�����һ��ֹͣ������
	//I2C_Generate_Start_ReadӦ�û��ٴη��Ͷ���ַ 0xA0 + 0x01 = 0xA1
	I2C_TransferHandling(sEE_I2C, sEEAddress, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_RXNE) == RESET) {		//�յ��ǿգ����
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

    return u8tmp;         //����EEDATA�洢������
}


//�����EEPROM_Read()�ķ���ֵ���Ͳ��ܸ��
//��������ڳ��־������ʱ����������һ��ѻ᲻���ֳ��ֳ�ʱ���󣿿��Ը��֣�������
UINT16 ReadEEPROM_Word_NoZone(UINT16 addr) {

	UINT16 tmp16a;
	UINT8  tmp8a,tmp8b;	
	
	tmp8a = EEPROM_Read(addr);		                        //��ȡ��λ��ַA��Ӧ������
	tmp8b = EEPROM_Read(addr+1);	                        //��ȡ��λ��ַA+1��Ӧ������
	tmp16a = tmp8b;
	tmp16a = (tmp16a<<8) | tmp8a;	                        //���ݴ洢

	return tmp16a;

}



//��Ҫ����������˼��仰
INT8 WriteEEPROM_Word_NoZone(UINT16            addr, UINT16 data)
{
    UINT8  tmp8a;
    UINT8  WriteCounter = 0;		            /* EEPROMд����----*/
    UINT16 tmp_addr;
    UINT16 tmp16;
	INT8 result = 0;
	tmp_addr = addr;						//��ֲ������仰

	MCUO_E2PR_WP = 0;

	WriteCounter = 0;
	do {
		tmp8a = data & 0xff;				//��ȡ���ݵĵ�8λ
		EEPROM_Write(tmp_addr, tmp8a);		//���ݵĵ�8λд��EEPROM
		tmp8a = data >> 8;					//��ȡ���ݵĸ�8λ
		//__delay_ms(5);
		EEPROM_Write(tmp_addr+1, tmp8a);	//���ݵĸ�8λд��EEPROM
	
		//��ȡд�������
		sEE_WaitEepromStandbyState();
		tmp8a = EEPROM_Read(tmp_addr);		//��ȡ�մ���EEPROM�ĵ�8λ����
		tmp16 = tmp8a;
		tmp8a = EEPROM_Read(tmp_addr+1);	//��ȡ�մ���EEPROM�ĸ�8λ����
		tmp16 = (tmp8a<<8) |tmp16 ; 		//�洢�����������ڱ���tmp16
	
		WriteCounter++;
		if(WriteCounter > 2) {				/*�ж�tmp16 != data�ļ���*/
			result = -1;
			++sEE_I2CFaultcnt;
			break;
		}
	}while(tmp16 != data);
	//__delay_ms(5);						//��ֹûд���ʧЧ�������Ǹ��Ͳ��ã���Ϊǰ���Ѿ�read��
	MCUO_E2PR_WP = 1;	

	return result;
}



//�����EEPROM_Read()�ķ���ֵ���Ͳ��ܸ��
//��������ڳ��־������ʱ����������һ��ѻ᲻���ֳ��ֳ�ʱ���󣿿��Ը��֣�������
UINT16 ReadEEPROM_Word(UINT16 addr) {

	UINT16 tmp16a,tmp16b,tmp16c;
	UINT8  tmp8a,tmp8b;	
	UINT16 addrB,addrC;
	
	addrB = addr + BZONE;
	addrC = addr + CZONE;
	//sEE_WaitEepromStandbyState();
	tmp8a = EEPROM_Read(addr);		                        //��ȡ��λ��ַA��Ӧ������
	tmp8b = EEPROM_Read(addr+1);	                        //��ȡ��λ��ַA+1��Ӧ������
	tmp16a = tmp8b;
	tmp16a = (tmp16a<<8) | tmp8a;	                        //���ݴ洢
	
	tmp8a = EEPROM_Read(addrB);                            //��ȡ��λ��ַB��Ӧ������
	tmp8b = EEPROM_Read(addrB+1);	                        //��ȡ��λ��ַB+1��Ӧ������
	tmp16b = tmp8b;
	tmp16b = (tmp16b<<8) | tmp8a;	                        //���ݴ洢
	
	tmp8a = EEPROM_Read(addrC);                            //��ȡ��λ��ַC��Ӧ������
	tmp8b = EEPROM_Read(addrC+1);	                        //��ȡ��λ��ַC+1��Ӧ������
	tmp16c = tmp8b;
	tmp16c = (tmp16c<<8) | tmp8a;	                        //���ݴ洢
	
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
                return tmp16a;                        //tmp16a,tmp16b,tmp16c����Ĭ��ֵ�����ص�һ��ֵ��
            }		
        }
    }
}



//������������⣬BC��д����ȥ����֪��Ϊʲô
INT8 WriteEEPROM_Word2(UINT16            addr, UINT16 data)
{
    UINT8  i,tmp8a;
    UINT8  WriteCounter = 0;		            /* EEPROMд����----*/
    UINT16 tmp_addr;
    UINT16 tmp16;
	tmp_addr = addr;
	
	MCUO_E2PR_WP = 0;
	for(i = 0; i < 3; i++)
	{
        WriteCounter = 0;
        do
        {
            tmp8a = (UINT8)(data & 0xff);                //��ȡ���ݵĵ�8λ
            EEPROM_Write(tmp_addr, tmp8a);      //���ݵĵ�8λд��EEPROM
            tmp8a = (UINT8)(data >> 8);                  //��ȡ���ݵĸ�8λ
            //__delay_ms(5);
            EEPROM_Write(tmp_addr+1, tmp8a);    //���ݵĸ�8λд��EEPROM

            //��ȡд�������
            sEE_WaitEepromStandbyState();
            tmp8a = EEPROM_Read(tmp_addr);      //��ȡ�մ���EEPROM�ĵ�8λ����
            tmp16 = tmp8a;
            tmp8a = EEPROM_Read(tmp_addr+1);    //��ȡ�մ���EEPROM�ĸ�8λ����
            tmp16 = (tmp8a<<8) |tmp16 ;         //�洢�����������ڱ���tmp16
			//__delay_ms(5);
            WriteCounter++;
            if(WriteCounter > 2)                /*�ж�tmp16 != data�ļ���*/
            {
            	++sEE_I2CFaultcnt;
                break;
            }
        }while(tmp16 != data);
		
        //��ַ��0x100д��
        tmp_addr = tmp_addr + BZONE;
    }
	MCUO_E2PR_WP = 1;
		return 0;//--
}

//��Ҫ����������˼��仰
INT8 WriteEEPROM_Word(UINT16 addr, UINT16 data)
{
	INT8 result = 0;
	result += WriteEEPROM_Word_NoZone(addr, data);
	result += WriteEEPROM_Word_NoZone(addr + BZONE, data);
	result += WriteEEPROM_Word_NoZone(addr + CZONE, data);
	return result;
}





//�����ҳ���������BC��д����ȥ������0xFF
void EEPROM_test(void) {

}


