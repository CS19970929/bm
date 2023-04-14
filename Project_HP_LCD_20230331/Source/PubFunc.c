#include "main.h"

uint16_t CRCChk(uint8_t *p, uint8_t len) {   
	// Dataָ��Ҫ�����CRC���飬LenthΪ���ݵ���Ч����
	uint16_t CRCC = 0xFFFF;	//CRC�ĳ�ʼֵ
	uint16_t i;
	uint16_t j;
	for( i = 0; i < len; i++ )
	{
		CRCC ^= *p++;				//�͵�ǰ�ֽ����һ��		//CRC ^= Data[i];
		for( j = 0; j < 8; j++ )	//ÿ���ֽ�ѭ��8��		
		{
			if( CRCC & 0x01 )
			{
				CRCC >>= 1;			    //����1λ	
				CRCC ^= 0xA001;		//�Ͷ���ʽ���
			}
			else
			{
				CRCC >>= 1;			    //����1λ	
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

