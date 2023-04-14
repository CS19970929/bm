#ifndef EEPROM_H
#define EEPROM_H

#define sEE_OK         			  ((INT8)0)
#define sEE_FAIL           		  ((INT8)-1) 		//负数可以吗，可以的！
#define sEE_I2C        			  I2C2
#define sEE_I2C_RW_W				0
#define sEE_I2C_RW_R				1
#define sEEAddress 				  0xA0		//E2 = E1 = E0 = 0
#define sEE_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define sEE_LONG_TIMEOUT         ((uint32_t)(10 * sEE_FLAG_TIMEOUT))
#define sEE_MAX_TRIALS_NUMBER     300

#define AZONE				0x0000              // A区: 0x0000~0x0799		//2K一个区间
#define BZONE				0x0800				// B区: 0x0800~0x0999
#define CZONE				0x1000				// C区: 0x1000~0x1800


#define E2P_BEGIN_FLAG						0x1133
#define EEPROM_SLEEP_ADDR           		((UINT16)0x3FFA)
#define EEPROM_PASS_ADDRL           		((UINT16)0x3FFC)
//#define EEPROM_PASS_ADDRH           		((UINT16)0x3FFD)
#define EEPROM_FLASHUPDATE_ADDR     		((UINT16)0x3FFE)


#define EEPROM_FLASHUPDATE_VALUE    		((UINT16)0xABCD)
#define EEPROM_FLASHUPDATE_RESET_VALUE    	((UINT16)0xFFFF)
#define EEPROM_SLEEP_VALUE    				((UINT16)0xABCD)
#define EEPROM_SLEEP_RESET_VALUE    		((UINT16)0xFFFF)


extern UINT8  sEE_I2CFaultcnt;

void InitE2PROM(void);
UINT16 ReadEEPROM_Word(UINT16 addr);
INT8 WriteEEPROM_Word(UINT16 addr, UINT16 data);
UINT16 ReadEEPROM_Word_NoZone(UINT16 addr);
INT8 WriteEEPROM_Word_NoZone(UINT16 addr, UINT16 data);
void EEPROM_test(void);


#endif	/* EEPROM_H */


