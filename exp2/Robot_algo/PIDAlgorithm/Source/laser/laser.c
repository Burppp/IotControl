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
*名称：laser_I2C_Init()
*功能：激光测距I2C初始化
*参数：dir-方向 FRONT-前置激光 REAR-后置激光
*返回：无
*修改：
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
*名称：laser_Init()
*功能：激光测距传感器初始化
*参数：无
*返回：无
*修改：
*********************************************************************************************/
void laser_Init(void)
{
  laserI2C.LASER_I2C_IO_Init();
}

/*********************************************************************************************
*名称：laser_WriteReg()
*功能：向激光传感器寄存器写数据
*参数：data-要写入的数据
*返回：0-失败 1-成功
*修改：
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
*名称：laser_readReg()
*功能：读取激光传感器寄存器数据
*参数：data-数据 
*      num-读取字节数量
*返回：0-失败 1-成功
*修改：
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
*名称：get_laserData()
*功能：获取激光测距传感器数据
*参数：无
*返回：激光测距数据
*修改：
*********************************************************************************************/
float get_laserData(void)
{
  char dataBuf[2] = {0};
  laser_WriteReg(LASER_READ);
  delay_ms(250);
  laser_readReg(dataBuf, 2);
  return dataBuf[0] * 256 + dataBuf[1];
}

