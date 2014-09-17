/*
 * LCD.h
 *
 * Created: 02/26/2012 06:50:30 ب.ظ
 *  Author: Milad
 */ 


#ifndef LCD_H_
#define LCD_H_

#include <asf.h>
#include "initialize.h"

//#define LCD_BUSY_CHECK

//struct LCD_DATA_PORT 





#define E_High()	LCD_E_PORT|=(1<<LCD_E_BIT)
#define E_Low()		LCD_E_PORT&=~(1<<LCD_E_BIT)

#define RW_High()	LCD_RW_PORT|=(1<<LCD_RW_BIT)
#define RW_Low()	LCD_RW_PORT&=~(1<<LCD_RW_BIT)

#define RS_High()	LCD_RS_PORT|=(1<<LCD_RS_BIT)
#define RS_Low()	LCD_RS_PORT&=~(1<<LCD_RS_BIT)

#define lcdpgm_read_byte(x)    (*((uint8_t *)(x)))

#define LCD_CLR             1	// clear display
#define LCD_HOME            2	// return to home position
#define LCD_DDRAM           7	//DB7: set DD RAM address

// cursor position to DDRAM mapping
#define LCD_LINE0_DDRAMADDR		0x00
#define LCD_LINE1_DDRAMADDR		0x40
#define LCD_LINE2_DDRAMADDR		0x14
#define LCD_LINE3_DDRAMADDR		0x54

void LCDDataLines(unsigned char);
void LCDDirOut(void);
void LCDDirIn(void);
void LCDSendData(unsigned char);
void LCDSendCommand(unsigned char);
void LCDClr(void);
void LCDHome(void);
void LCDCursorOn(void);
void LCDCursorOnBlink(void);
void LCDCursorOFF(void);
void LCDVisible(void);
void LCDCursorLeft(uint8_t n);
void LCDCursorRight(uint8_t n);
void LCDInit(void);
void LCDGotoXY(unsigned char, unsigned char);
void LCDBusyWait(void);
void LCDStringRam(char *);
void LCDStringFlash(char *);
void LCDShiftRight(uint8_t);
void LCDShiftLeft(uint8_t);
void LCDdefinechar(uint8_t *,uint8_t);
void LCDBlank(void);




#endif /* LCD_H_ */
