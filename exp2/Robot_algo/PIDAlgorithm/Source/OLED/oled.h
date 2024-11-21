#ifndef OLED_H
#define OLED_H
#include "stm32f4xx.h"

void OLED_Init(void);
void OLED_Write_command(unsigned char IIC_Command);
void OLED_IIC_write(unsigned char IIC_Data);
void OLED_fillpicture(unsigned char fill_Data);
void OLED_Clear(void) ;
void OLED_Refresh_Gram(unsigned char startPage, unsigned char endPage);
void OLED_Display_Off(void);
void OLED_Display_On(void);
void OLED_DrawPoint(unsigned char x,unsigned char y,unsigned char t);
void OLED_DisFill(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char dot);
void OLED_ShowChar(unsigned char x,unsigned char y,unsigned char chr,unsigned char Char_Size);
void OLED_ShowCHinese(unsigned char x,unsigned char y,unsigned char num);
void OLED_ShowString(unsigned char x,unsigned char y,unsigned char *chr,unsigned char Char_Size);
void OLED_DisClear(int hstart,int hend,int lstart,int lend);
void OLED_Fill(void);
void oled_turn_on(void);
void oled_turn_off(void);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_drawBmp(unsigned char x, unsigned char y, unsigned char num);
void OLED_drawCircle(unsigned char CircleX, unsigned char CircleY, unsigned char CircleR);
void OLED_drawLine(unsigned char startX, unsigned char startY, unsigned char endX, unsigned char endY, unsigned char status);
void OLED_drawRect(unsigned char startX, unsigned char startY, unsigned char endX, unsigned char endY, unsigned char status);
void oled_showTimeNum(unsigned char x, unsigned char y, unsigned char num);
void OLED_ShowChineseLib(unsigned char x,unsigned char y,char* text,unsigned char textSize,unsigned int showLen);
void oled_showNum(unsigned char x, unsigned char y, unsigned char num);
void OLED_SlideCHinese(unsigned char x,unsigned char y,unsigned char startNum, unsigned char endNum,
                       unsigned char showNum, unsigned char slideSize, char firstFlag);
void OLED_drawBmp(unsigned char x, unsigned char y, unsigned char num);

#endif
