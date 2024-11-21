#include "soft_iic/soft_iic.h"
#include "delay/delay.h"

void I2C1_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  RCC_AHB1PeriphClockCmd(I2C1_SCL_LCK | I2C1_SDA_LCK, ENABLE);
  
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_InitStruct.GPIO_Pin = I2C1_SCL_PIN;
  GPIO_Init(I2C1_SCL_GPIO, &GPIO_InitStruct);
  
  GPIO_InitStruct.GPIO_Pin = I2C1_SDA_PIN;
  GPIO_Init(I2C1_SDA_GPIO, &GPIO_InitStruct);
}

#define I2C1_PERIOD 128
void I2C1_Delay(unsigned char times)
{
  unsigned short delay = 0;
  while(delay++ < I2C1_PERIOD * times);
}

void I2C1_Start(void)
{
  I2C1_SDA_OUT;
  I2C1_SCL_H;
  I2C1_SDA_H;
  I2C1_Delay(1);
  I2C1_SDA_L;
  I2C1_Delay(1);
  I2C1_SCL_L;
}

void I2C1_Stop(void)
{
  I2C1_SDA_OUT;
  I2C1_SCL_L;
  I2C1_SDA_L;
  I2C1_Delay(1);
  I2C1_SCL_H;
  I2C1_Delay(1);
  I2C1_SDA_H;
}

unsigned char I2C1_WaitAck(void)
{
  unsigned char timeCount = 0;
  I2C1_SDA_IN;
  I2C1_SCL_H;
  I2C1_Delay(1);
  while(I2C1_SDA_R)
  {
    timeCount++;
    I2C1_Delay(1);
    if(timeCount > 250)
    {
      I2C1_Stop();
      I2C1_SCL_L;
      return 1;
    }
  }
  I2C1_SCL_L;
  return 0;
}

void I2C1_Ack(void)
{
  I2C1_SDA_OUT;
  I2C1_SDA_L;
  I2C1_Delay(1);
  I2C1_SCL_H;
  I2C1_Delay(1);
  I2C1_SCL_L;
}

void I2C1_NAck(void)
{
  I2C1_SDA_OUT;
  I2C1_SDA_H;
  I2C1_Delay(1);
  I2C1_SCL_H;
  I2C1_Delay(1);
  I2C1_SCL_L;
}

void I2C1_WriteByte(unsigned char data)
{
  I2C1_SDA_OUT;
  I2C1_SCL_L;
  for(unsigned char i=0; i<8; i++)
  {
    if(data & 0x80)
      I2C1_SDA_H;
    else
      I2C1_SDA_L;
    I2C1_Delay(1);
    I2C1_SCL_H;
    I2C1_Delay(2);
    I2C1_SCL_L;
    I2C1_Delay(1);
    data <<= 1;
  }
}

unsigned char I2C1_ReadByte(unsigned char ack)
{
  unsigned char data = 0;
  I2C1_SDA_IN;
  for(unsigned char i=0; i<8; i++)
  {
    I2C1_SCL_L;
    I2C1_Delay(1);
    I2C1_SCL_H;
    data <<= 1;
    if(I2C1_SDA_R)
      data ++;
    I2C1_Delay(2);
  }
  I2C1_SCL_L;
  I2C1_Delay(1);
  if(ack)
    I2C1_Ack();
  else
    I2C1_NAck();
  return data;
}




