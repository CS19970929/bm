#ifndef LCD_H
#define LCD_H

#define LCD_DISPLAY_DELAY 2
#define LCD_DISPLAY_DELAY1 2

extern const unsigned char ascii_table_8x16[95][16];
extern const unsigned char ascii_table_5x8[95][5];

void InitLCD(void);
void LCD_Display(void);
void LCD_test(void);

#endif	/* LCD_H */
