#include "laser/laser.h"
#include "soft_iic/soft_iic.h"
#include "delay/delay.h"

LASER_I2C_t laserI2C = {
  .LASER_I2C_IO_Init = I2C2_IO_Init,
  .LASER_I2C_Start = I2C2_Start,
  .LASER_I2C_WriteByte = I2C2_WriteByte,
  .LASER_I2C_WaitAck = I2C2_WaitAck,
  .LASER_I2C_ReadByte = I2C2_ReadByte,
  .LASER_I2C_Stop = I2C2_Stop,
};

/*********************************************************************************************
*���ƣ�laser_I2C_Init()
*���ܣ�������I2C��ʼ��
*������dir-���� FRONT-ǰ�ü��� REAR-���ü���
*���أ���
*�޸ģ�
*********************************************************************************************/
void laser_I2C_Init(unsigned char i2cNum)
{
  switch(i2cNum)
  {
  case 1:
    laserI2C.LASER_I2C_IO_Init = I2C1_IO_Init;
    laserI2C.LASER_I2C_Start = I2C1_Start;
    laserI2C.LASER_I2C_WriteByte = I2C1_WriteByte;
    laserI2C.LASER_I2C_WaitAck = I2C1_WaitAck;
    laserI2C.LASER_I2C_ReadByte = I2C1_ReadByte;
    laserI2C.LASER_I2C_Stop = I2C1_Stop;
    break;
  case 2:
    laserI2C.LASER_I2C_IO_Init = I2C2_IO_Init;
    laserI2C.LASER_I2C_Start = I2C2_Start;
    laserI2C.LASER_I2C_WriteByte = I2C2_WriteByte;
    laserI2C.LASER_I2C_WaitAck = I2C2_WaitAck;
    laserI2C.LASER_I2C_ReadByte = I2C2_ReadByte;
    laserI2C.LASER_I2C_Stop = I2C2_Stop;
    break;
  case 3:
    /*
    laserI2C.LASER_I2C_IO_Init = I2C3_IO_Init;
    laserI2C.LASER_I2C_Start = I2C3_Start;
    laserI2C.LASER_I2C_WriteByte = I2C3_WriteByte;
    laserI2C.LASER_I2C_WaitAck = I2C3_WaitAck;
    laserI2C.LASER_I2C_ReadByte = I2C3_ReadByte;
    laserI2C.LASER_I2C_Stop = I2C3_Stop;
    */
    break;
  case 4:
    /*
    laserI2C.LASER_I2C_IO_Init = I2C4_IO_Init;
    laserI2C.LASER_I2C_Start = I2C4_Start;
    laserI2C.LASER_I2C_WriteByte = I2C4_WriteByte;
    laserI2C.LASER_I2C_WaitAck = I2C4_WaitAck;
    laserI2C.LASER_I2C_ReadByte = I2C4_ReadByte;
    laserI2C.LASER_I2C_Stop = I2C4_Stop;
    */
    break;
  }
}

/*********************************************************************************************
*���ƣ�laser_Init()
*���ܣ������ഫ������ʼ��
*��������
*���أ���
*�޸ģ�
*********************************************************************************************/
void laser_Init(void)
{
  laserI2C.LASER_I2C_IO_Init();
}

/*********************************************************************************************
*���ƣ�laser_WriteReg()
*���ܣ��򼤹⴫�����Ĵ���д����
*������data-Ҫд�������
*���أ�0-ʧ�� 1-�ɹ�
*�޸ģ�
*********************************************************************************************/
unsigned char laser_WriteReg(unsigned char data)
{
  laserI2C.LASER_I2C_Start();
  laserI2C.LASER_I2C_WriteByte(LASER_ADDR);
  if(laserI2C.LASER_I2C_WaitAck()) return 0;
  laserI2C.LASER_I2C_WriteByte(data);
  if(laserI2C.LASER_I2C_WaitAck()) return 0;
  laserI2C.LASER_I2C_Stop();
  return 1;
}

/*********************************************************************************************
*���ƣ�laser_readReg()
*���ܣ���ȡ���⴫�����Ĵ�������
*������data-���� 
*      num-��ȡ�ֽ�����
*���أ�0-ʧ�� 1-�ɹ�
*�޸ģ�
*********************************************************************************************/
unsigned char laser_readReg(char *data, unsigned char num)
{
  laserI2C.LASER_I2C_Start();
  laserI2C.LASER_I2C_WriteByte(LASER_ADDR + 1);
  if(laserI2C.LASER_I2C_WaitAck()) return 0;
  unsigned char i = 0;
  for(i=0; i<num-1; i++)
  {
    data[i] = laserI2C.LASER_I2C_ReadByte(1);
  }
  data[i] = laserI2C.LASER_I2C_ReadByte(0);
  laserI2C.LASER_I2C_Stop();
  return 1;
}

/*********************************************************************************************
*���ƣ�get_laserData()
*���ܣ���ȡ�����ഫ��������
*��������
*���أ�����������
*�޸ģ�
*********************************************************************************************/
float get_laserData(void)
{
  char dataBuf[2] = {0};
  laser_WriteReg(LASER_READ);
  delay_ms(250);
  laser_readReg(dataBuf, 2);
  return dataBuf[0] * 256 + dataBuf[1];
}

