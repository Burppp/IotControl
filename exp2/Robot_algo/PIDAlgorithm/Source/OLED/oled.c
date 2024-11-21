/*********************************************************************************************
* �ļ���oled.c
* ���ߣ�zonesion
* ˵����
* �޸ģ�Chenkm 2017.01.05 ������ע��
* ע�ͣ�
*********************************************************************************************/
/*********************************************************************************************
* ͷ�ļ�
*********************************************************************************************/
#include "soft_iic/soft_iic.h"
#include "OLED/oled.h"
#include "OLED/oledfont.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "delay/delay.h"
#include "wireless uart/wireless_usart.h"
#define ADDR_W          0X78                                    // ����д��ַ
#define ADDR_R          0X79                                    // ��������ַ

#define Max_Column	96
#define Max_Row         4

unsigned char OLED_GRAM[Max_Column][Max_Row];

/*********************************************************************************************
* ����: OLED_Init()
* ����: OLED��ʼ��
* ����: ��
* ����: ��
* ע�ͣ�
* �޸�:
*********************************************************************************************/
void  OLED_Init(void){
  delay_ms(500);
  I2C2_IO_Init();
  OLED_Write_command(0xAE); /*display off*/
  OLED_Write_command(0x00); /*set lower column address*/
  OLED_Write_command(0x10); /*set higher column address*/
  OLED_Write_command(0x40); /*set display start line*/
  OLED_Write_command(0xb0); /*set page address*/
  OLED_Write_command(0x81); /*contract control*/
  OLED_Write_command(0x45); /*128*/
  OLED_Write_command(0xA1); /*set segment remap*/
  OLED_Write_command(0xC0);/*Com scan direction 0XC0 */
  OLED_Write_command(0xA6); /*normal / reverse*/
  OLED_Write_command(0xA8); /*multiplex ratio*/
  OLED_Write_command(0x1F); /*duty = 1/32*/
  OLED_Write_command(0xD3); /*set display offset*/
  OLED_Write_command(0x00);
  OLED_Write_command(0xD5); /*set osc division*/
  OLED_Write_command(0x80);
  OLED_Write_command(0xD9); /*set pre-charge period*/
  OLED_Write_command(0x22);
  OLED_Write_command(0xDA); /*set COM pins*/
  OLED_Write_command(0x12);
  OLED_Write_command(0xdb); /*set vcomh*/
  OLED_Write_command(0x20);
  OLED_Write_command(0x8d); /*set charge pump enable*/
  OLED_Write_command(0x14);
  OLED_Write_command(0xAF); /*display ON*/
  OLED_Clear();             //����
  char mac[20];
  int32_t overtime = 100000, try_times = 3;
  while(try_times-- > 0) {
    Wireless_Usart_len = 0;
    wireless_usart_send("AT+MAC?\r\n",strlen("AT+MAC?\r\n"));
    overtime = 100000;
    while(Wireless_Usart_len == 0 || strstr(WIRELESS_USART_RX_BUF, "MAC")==NULL ||\
          strstr(WIRELESS_USART_RX_BUF, "OK")==NULL) {
      if(overtime-- < 0) break;
    }
    if(overtime > 0) {
      Wireless_Usart_len = 0;
      memset(mac, 0, sizeof(mac));
      strncpy(mac, WIRELESS_USART_RX_BUF+5, 17);
      OLED_ShowString(0, 1, (uint8_t*)mac, 8);
    }
  }
}

/*********************************************************************************************
* ����: OLED_Write_command()
* ����: IIC Write Command
* ����: ����
* ����: ��
* �޸�:
* ע��: 
*********************************************************************************************/
void OLED_Write_command(unsigned char IIC_Command)
{
  I2C2_Start();
  I2C2_WriteByte(ADDR_W);
  if(I2C2_WaitAck()) return ;
  I2C2_WriteByte(0x00);
  if(I2C2_WaitAck()) return ;
  I2C2_WriteByte(IIC_Command);
  if(I2C2_WaitAck()) return ;
  I2C2_Stop();
}

/*********************************************************************************************
* ����: OLED_IIC_write()
* ����: IIC Write Data
* ����: ����
* ����: ��
* �޸�:
* ע��:
*********************************************************************************************/
void OLED_IIC_write(unsigned char IIC_Data)
{
  I2C2_Start();
  I2C2_WriteByte(ADDR_W);
  if(I2C2_WaitAck()) return ;
  I2C2_WriteByte(0x40);
  if(I2C2_WaitAck()) return ;
  I2C2_WriteByte(IIC_Data);
  if(I2C2_WaitAck()) return ;
  I2C2_Stop();
}

/*********************************************************************************************
* ����: OLED_fillpicture()
* ����: OLED_fillpicture
* ����: ����
* ����: ��
* �޸�:
* ע��:
*********************************************************************************************/
void OLED_fillpicture(unsigned char fill_Data){
  unsigned char m,n;
  for(m=0;m<Max_Row;m++){
    OLED_Write_command(0xb0+m);		                        //page0-page1
    OLED_Write_command(0x00);		                        //low column start address
    OLED_Write_command(0x10);		                        //high column start address
    for(n=0;n<Max_Column;n++){
      OLED_IIC_write(fill_Data);
    }
  }
}

/*********************************************************************************************
* ����: OLED_Set_Pos()
* ����: ��������
* ����: ����
* ����: ��
* �޸�:
* ע��:
*********************************************************************************************/
void OLED_Set_Pos(unsigned char x, unsigned char y) { 	
  OLED_Write_command(0xb0+y);
  OLED_Write_command(((x&0xf0)>>4)|0x10);
  OLED_Write_command((x&0x0f)); 
} 

/*********************************************************************************************
* ����: OLED_Display_On()
* ����: ����OLED��ʾ
* ����: 
* ����: 
* �޸�:
* ע��:
*********************************************************************************************/
void OLED_Display_On(void){
  OLED_Write_command(0X8D);                                     //SET DCDC����
  OLED_Write_command(0X14);                                     //DCDC ON
  OLED_Write_command(0XAF);                                     //DISPLAY ON
}

/*********************************************************************************************
* ����: OLED_Display_Off()
* ����: �ر�OLED��ʾ
* ����: 
* ����: 
* �޸�:
* ע��:
*********************************************************************************************/  
void OLED_Display_Off(void){
  OLED_Write_command(0X8D);                                     //SET DCDC����
  OLED_Write_command(0X10);                                     //DCDC OFF
  OLED_Write_command(0XAE);                                     //DISPLAY OFF
}

/*********************************************************************************************
* ����: OLED_Clear()
* ����: ��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!
* ����: 
* ����: 
* �޸�:
* ע��:
*********************************************************************************************/  
void OLED_Clear(void)  {  
  unsigned char i,n;		    
  for(i=0;i<Max_Row;i++)  {  
    OLED_Write_command (0xb0+i);                                //����ҳ��ַ��0~7��
    OLED_Write_command (0x00);                                  //������ʾλ�á��е͵�ַ
    OLED_Write_command (0x10);                                  //������ʾλ�á��иߵ�ַ   
    for(n=0;n<Max_Column;n++)
      OLED_IIC_write(0); 
  } 
}

//�����Դ浽OLED		 
void OLED_Refresh_Gram(unsigned char startPage, unsigned char endPage)
{
  unsigned char i=0,n=0;		    
  for(i=startPage; i<endPage; i++)  
  {  
    OLED_Write_command (0xb0+i);                                //����ҳ��ַ��0~7��
    OLED_Write_command (0x00);                                  //������ʾλ�á��е͵�ַ
    OLED_Write_command (0x10);                                  //������ʾλ�á��иߵ�ַ   
    for(n=0;n<Max_Column;n++)
      OLED_IIC_write(OLED_GRAM[n][i]); 
  }   
}
//���� 
//x:0~127
//y:0~63
//t:1 ��� 0,���				   
void OLED_DrawPoint(unsigned char x,unsigned char y,unsigned char t)
{
  unsigned char pos,bx,temp=0;
  if(x>(Max_Column-1)||y>(Max_Row*8-1))return;//������Χ��.
  pos=y/8;
  bx=y%8;
  temp=1<<bx;
  if(t)OLED_GRAM[x][pos]|=temp;
  else OLED_GRAM[x][pos]&=~temp;
}

void OLED_DisFill(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char dot)  
{  
  unsigned char x,y;  
  for(x=x1;x<=x2;x++)
  {
    for(y=y1;y<=y2;y++)OLED_DrawPoint(x,y,dot);
  }													    
  //OLED_Refresh_Gram();//������ʾ
}
/*********************************************************************************************
* ����: OLED_DisClear()
* ����: �������
* ����: ����
* ����: 
* �޸�:
* ע��:
*********************************************************************************************/  
void OLED_DisClear(int hstart,int hend,int lstart,int lend){
  unsigned char i,n;
  for(i=hstart;i<=hend;i++)  {  
    OLED_Write_command (0xb0+i);                                //����ҳ��ַ��0~7��
    OLED_Write_command (0x00);                                  //������ʾλ�á��е͵�ַ
    OLED_Write_command (0x10);                                  //������ʾλ�á��иߵ�ַ   
    for(n=lstart;n<=lend;n++) {
      OLED_IIC_write(0); 
    }
      
  } 
}

/*********************************************************************************************
* ����: OLED_ShowChar()
* ����: ��ָ��λ����ʾһ���ַ�,���������ַ�
* ����: ���꣨x:0~127��y:0~63����chr�ַ���Char_Size�ַ�����
* ����: 
* �޸�:
* ע��:
*********************************************************************************************/  
void OLED_ShowChar(unsigned char x,unsigned char y,unsigned char chr,unsigned char Char_Size){      	
  unsigned char c=0,i=0;	
  c=chr-' ';//�õ�ƫ�ƺ��ֵ			
  if(x>Max_Column-1){x=0;y=y+2;}
  if(Char_Size ==16){
    OLED_Set_Pos(x,y);	
    for(i=0;i<8;i++)
      OLED_IIC_write(F8X16[c*16+i]);
    OLED_Set_Pos(x,y+1);
    for(i=0;i<8;i++)
      OLED_IIC_write(F8X16[c*16+i+8]);
  }
  else if(Char_Size ==12){
    OLED_Set_Pos(x,y);	
    for(i=0;i<6;i++)
      OLED_IIC_write(F6X12[c*12+i]);
    OLED_Set_Pos(x,y+1);
    for(i=0;i<6;i++)
      OLED_IIC_write(F6X12[c*12+i+6]);
  }
  else {	
    OLED_Set_Pos(x,y);
    for(i=0;i<6;i++)
      OLED_IIC_write(F6x8[c][i]);
  }
}

/*********************************************************************************************
* ����: OLED_ShowString()
* ����: ��ʾһ���ַ���
* ����: ��ʼ���꣨x:0~127��y:0~63����chr�ַ���ָ�룻Char_Size�ַ�������
* ����: 
* �޸�:
* ע��:
*********************************************************************************************/  
void OLED_ShowString(unsigned char x,unsigned char y,unsigned char *chr,unsigned char Char_Size){
  unsigned char j=0;
  while (chr[j]!='\0'){	
    OLED_ShowChar(x,y,chr[j],Char_Size);
    if(Char_Size == 8)
      x += 6;
    else
      x += 8;
    if(x>120){
      x=0;
      if(Char_Size == 8)
        y++;
      else
        y += 2;
    }
    j++;
  }
}

/*********************************************************************************************
* ����: OLED_ShowCHinese()
* ����: ��ʾһ������
* ����: ��ʼ���꣨x:0~127��y:0~63����num�������Զ����ֿ��еı�ţ�oledfont.h�����
* ����: 
* �޸�:
* ע��:
*********************************************************************************************/ 
void OLED_ShowCHinese(unsigned char x,unsigned char y,unsigned char num){      			    
  unsigned char t;
  OLED_Set_Pos(x,y);	
  for(t=0;t<16;t++){
    OLED_IIC_write(Hzk[2*num][t]);
  }	
  OLED_Set_Pos(x,y+1);	
  for(t=0;t<16;t++){
    OLED_IIC_write(Hzk[2*num+1][t]);
  }					
}

void OLED_Fill(void)  {  
  unsigned char i,n;		    
  for(i=0;i<Max_Row;i++)  {  
    OLED_Write_command (0xb0+i);                                //����ҳ��ַ��0~7��
    OLED_Write_command (0x00);                                  //������ʾλ�á��е͵�ַ
    OLED_Write_command (0x10);                                  //������ʾλ�á��иߵ�ַ   
    for(n=0;n<Max_Column;n++)
      OLED_IIC_write(0xff); 
  } 
}

// ����
void OLED_drawLine(unsigned char startX, unsigned char startY, unsigned char endX, unsigned char endY, unsigned char status)
{
  unsigned short x, y;
  if((startX == endX) && (startY == endY))
    OLED_DrawPoint(startX, startY, status);
  else if(abs(endY - startY) > abs(endX - startX))                   
  {
    if(startY > endY) 
    {
      startY ^= endY;
      endY ^= startY;
      startY ^= endY;
      startX ^= endX;
      endX ^= startX;
      startX ^= endX;
    }
    for(y=startY; y<endY; y++)                                  
    {
      x = (unsigned short)(y-startY)*(endX-startX)/(endY-startY)+startX;
      OLED_DrawPoint(x, y, status);  
    }
  }
  else                                                      
  {
    if(startX>endX)
    {
      startY ^= endY;
      endY ^= startY;
      startY ^= endY;
      startX ^= endX;
      endX ^= startX;
      startX ^= endX;
    }   
    for(x=startX;x<=endX;x++)                                  
    {
      y = (unsigned short)(x-startX)*(endY-startY)/(endX-startX)+startY;
      OLED_DrawPoint(x, y, status); 
    }
  }
}

// ��Բ
void OLED_drawCircle(unsigned char CircleX, unsigned char CircleY, unsigned char CircleR)
{
  float PI = 3.14;
  unsigned short x = 0, y = 0;
  for(unsigned short i=0; i<360; i++)
  {
    x = (unsigned char)(CircleR * sin((2*i)*(PI/360)) + CircleX);  //����Ӧ�ô���λ��
    y = (unsigned char)(CircleR * cos((2*i)*(PI/360)) + CircleY);
    OLED_DrawPoint(x ,y ,1);   //���
  }
}

// ������
void OLED_drawRect(unsigned char startX, unsigned char startY, unsigned char endX, unsigned char endY, unsigned char status)
{
  OLED_drawLine(startX,startY,endX,startY,status);
  OLED_drawLine(startX,startY,startX,endY,status);
  OLED_drawLine(startX,endY,endX,endY,status);
  OLED_drawLine(endX,startY,endX,endY,status);
}

// ��ʾ����ʱ����
void oled_showTimeNum(unsigned char x, unsigned char y, unsigned char num)
{
  OLED_Set_Pos(x,y);	
  for(unsigned char i=0; i<8; i++)
    OLED_IIC_write(TimeNum[2*num][i]);
  OLED_Set_Pos(x,y+1);
  for(unsigned char i=0; i<8; i++)	
    OLED_IIC_write(TimeNum[2*num+1][i]);
}

// ��ʾ������
void oled_showNum(unsigned char x, unsigned char y, unsigned char num)
{
  OLED_Set_Pos(x,y);	
  for(unsigned char i=0; i<20; i++)
    OLED_IIC_write(Num[4*num][i]);
  OLED_Set_Pos(x,y+1);
  for(unsigned char i=0; i<20; i++)	
    OLED_IIC_write(Num[4*num+1][i]);
  OLED_Set_Pos(x,y+2);	
  for(unsigned char i=0; i<20; i++)
    OLED_IIC_write(Num[4*num+2][i]);
  OLED_Set_Pos(x,y+3);
  for(unsigned char i=0; i<20; i++)	
    OLED_IIC_write(Num[4*num+3][i]);
}

/*********************************************************************************************
* ����: OLED_SlideCHinese()
* ����: ������ʾ�������
* ����: ��ʼ���꣨x:0~127��y:0~7����startNum:��ʼ��ţ�endNum:��β��� �������Զ����ֿ��еı�ţ�oledfont.h�����
        showNum: ��ʾ���ָ��� slideSize:ÿ�λ������ش�С   firstFlag:�Ƿ�����ϴ���ʾ��¼
* ����: 
* �޸�:
* ע��:
*********************************************************************************************/
void OLED_SlideCHinese(unsigned char x,unsigned char y,unsigned char startNum, unsigned char endNum, 
                       unsigned char showNum, unsigned char slideSize, char firstFlag)
{
  static unsigned int slideCount = 0;
  if(firstFlag == 1)
    slideCount = 0;
  slideCount += slideSize;
  if(slideCount > (endNum-startNum-showNum+1)*16)
    slideCount = 0;
  unsigned char a = slideCount/16;
  unsigned char b = startNum+a;
  unsigned char c = slideCount%16;
  for(unsigned char i=0; i<showNum+1; i++)
  {
    if(i == 0)
    {
      OLED_Set_Pos(x+(i*16), y);	
      for(unsigned char t=0; t<16-c; t++){
        OLED_IIC_write(Hzk[2*b][t+c]);
      }	
      OLED_Set_Pos(x+(i*16), y+1);	
      for(unsigned char t=0; t<16-c; t++){	
        OLED_IIC_write(Hzk[2*b+1][t+c]);
      }
    }
    else
    {
      OLED_Set_Pos(x+(i*16)-c, y);	
      for(unsigned char t=0;t<16;t++){
        OLED_IIC_write(Hzk[2*(i+startNum+a)][t]);
      }	
      OLED_Set_Pos(x+(i*16)-c, y+1);	
      for(unsigned char t=0;t<16;t++){	
        OLED_IIC_write(Hzk[2*(i+startNum+a)+1][t]);
      }
    }
  }
}

// ��ʾͼƬ
void OLED_drawBmp(unsigned char x, unsigned char y, unsigned char num)
{
  for(unsigned char i=0; i<4; i++)
  {
    OLED_Set_Pos(x,y+i);
    for(unsigned char j=0; j<32; j++)
      OLED_IIC_write(Bmp[4*num+i][j]);
  }
}

