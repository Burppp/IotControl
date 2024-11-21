#ifndef __PCA9685_H_
#define __PCA9685_H_
#include "stm32f4xx.h"

#define PCA9685_ADDR            0xE0                            // Õ®”√µÿ÷∑
#define PCA9685_ADDR1           0x82
#define PCA9685_ADDR2           0x84

#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01

#define PCA9685_LED_ON_L        0x06
#define PCA9685_LED_ON_H        0x07
#define PCA9685_LED_OFF_L       0x08
#define PCA9685_LED_OFF_H       0x09

#define PCA9685_PRE_SCALE       0xFE


uint8_t PCA9685_WriteReg(uint8_t addr, uint8_t regAddr, char *data, uint8_t num);
uint8_t PCA9685_ReadReg(uint8_t addr, uint8_t regAddr, char *data, uint8_t num);

void PCA9685_Init(void);
void PCA9685_SetPWMFreq(uint8_t addr, float val);
void PCA9685_SetPWM(uint8_t addr, uint8_t num, uint16_t onCount, uint16_t offCount);


#endif
