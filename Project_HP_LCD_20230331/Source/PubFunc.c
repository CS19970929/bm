#include "main.h"

uint16_t CRCChk(uint8_t *p, uint8_t len) {   
	// Data指向要计算的CRC数组，Lenth为数据的有效长度
	uint16_t CRCC = 0xFFFF;	//CRC的初始值
	uint16_t i;
	uint16_t j;
	for( i = 0; i < len; i++ )
	{
		CRCC ^= *p++;				//和当前字节异或一次		//CRC ^= Data[i];
		for( j = 0; j < 8; j++ )	//每个字节循环8次		
		{
			if( CRCC & 0x01 )
			{
				CRCC >>= 1;			    //右移1位	
				CRCC ^= 0xA001;		//和多项式异或
			}
			else
			{
				CRCC >>= 1;			    //右移1位	
			}
		}
	}
    return CRCC;
}


UINT16 Sci_CRC16RTU( UINT8 * pszBuf, UINT8 unLength)
{
	UINT16 CRCC=0XFFFF;
	UINT32 CRC_count;

	for(CRC_count=0;CRC_count<unLength;CRC_count++)
	{
		int i;

		CRCC=CRCC^*(pszBuf+CRC_count);

		for(i=0;i<8;i++)
		{
			if(CRCC&1)
			{
				CRCC>>=1;
				CRCC^=0xA001;
			}
			else
			{ 
                CRCC>>=1;
			}
				
		}
	}

	return CRCC;
}

