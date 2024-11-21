#include "pca9685/pca9685.h"
#include "soft_iic/soft_iic.h"
#include "delay/delay.h"
#include <math.h>

uint8_t PCA9685_WriteReg(uint8_t addr, uint8_t regAddr, char *data, uint8_t num)
{
  I2C1_Start();
  I2C1_WriteByte(addr);
  if(I2C1_WaitAck()) return 0;
  I2C1_WriteByte(regAddr);
  if(I2C1_WaitAck()) return 0;
  for(uint8_t i=0; i<num; i++)
  {
    I2C1_WriteByte(*data++);
    if(I2C1_WaitAck())
      return i;
  }
  I2C1_Stop();
  return num;
}

uint8_t PCA9685_ReadReg(uint8_t addr, uint8_t regAddr, char *data, uint8_t num)
{
  uint8_t i = 0;
  I2C1_Start();
  I2C1_WriteByte(addr);
  if(I2C1_WaitAck()) return 0;
  I2C1_WriteByte(regAddr);
  if(I2C1_WaitAck()) return 0;
  I2C1_Start();
  I2C1_WriteByte(addr + 1);
  if(I2C1_WaitAck()) return 0;
  for(i=0; i<num-1; i++)
  {
    data[i] = I2C1_ReadByte(1);
  }
  data[i] = I2C1_ReadByte(0);
  I2C1_Stop();
  return num;
}

void PCA9685_Reset(uint8_t addr)
{
  char val = 0;
  val = 0x11;
  PCA9685_WriteReg(addr, PCA9685_MODE1, &val, 1);
  delay_us(500);
  val = 0xA1;
  PCA9685_WriteReg(addr, PCA9685_MODE1, &val, 1);
}

void PCA9685_Init(void)
{
  char val = 0x21;
  I2C1_IO_Init();
  PCA9685_Reset(PCA9685_ADDR1);
  PCA9685_Reset(PCA9685_ADDR2);
  PCA9685_WriteReg(PCA9685_ADDR1, PCA9685_MODE1, &val, 1);
  PCA9685_WriteReg(PCA9685_ADDR2, PCA9685_MODE1, &val, 1);
}

void PCA9685_SetPWMFreq(uint8_t addr, float val)
{
  char lastMode = 0, newMode = 0, preVal = 0;
  float prescaleVal = 0;
  PCA9685_ReadReg(addr, PCA9685_MODE1, &lastMode, 1);
  val *= 0.915f;
  prescaleVal = 25000000 / (4096 * val);
  prescaleVal = round(prescaleVal) - 1;
  newMode = (lastMode & 0x7F) | 0x10;
  PCA9685_WriteReg(addr, PCA9685_MODE1, &newMode, 1);
  preVal = (char)prescaleVal;
  PCA9685_WriteReg(addr, PCA9685_PRE_SCALE, &preVal, 1);
  PCA9685_WriteReg(addr, PCA9685_MODE1, &lastMode, 1);
  delay_ms(2);
  lastMode |= 0xA1;
  PCA9685_WriteReg(addr, PCA9685_MODE1, &lastMode, 1);
  PCA9685_ReadReg(addr, PCA9685_PRE_SCALE, &lastMode, 1);
}

void PCA9685_SetPWM(uint8_t addr, uint8_t num, uint16_t onCount, uint16_t offCount)
{
  char dataBuf[4] = {onCount & 0xFF, onCount >> 8, offCount & 0xFF, offCount >> 8};
  num *= 4;
  PCA9685_WriteReg(addr, PCA9685_LED_ON_L + num, dataBuf, 4);
}
