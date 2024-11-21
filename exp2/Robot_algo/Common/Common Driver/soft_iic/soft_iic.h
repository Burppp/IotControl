#ifndef __SOFT_I2C_H_
#define __SOFT_I2C_H_
#include "stm32f4xx.h"

/*****  I2C1  *****/

#define I2C1_SCL_LCK         RCC_AHB1Periph_GPIOB
#define I2C1_SCL_GPIO        GPIOB
#define I2C1_SCL_PIN         GPIO_Pin_8

#define I2C1_SDA_LCK         RCC_AHB1Periph_GPIOB
#define I2C1_SDA_GPIO        GPIOB
#define I2C1_SDA_PIN         GPIO_Pin_9

#define I2C1_SDA_NUM         9

#define I2C1_SDA_IN          do{I2C1_SDA_GPIO->MODER &= ~(3<<I2C1_SDA_NUM*2); I2C1_SDA_GPIO->MODER |= (0<<I2C1_SDA_NUM*2);}while(0)
#define I2C1_SDA_OUT         do{I2C1_SDA_GPIO->MODER &= ~(3<<I2C1_SDA_NUM*2); I2C1_SDA_GPIO->MODER |= (1<<I2C1_SDA_NUM*2);}while(0)

#define I2C1_SCL_L           (I2C1_SCL_GPIO->BSRRH = I2C1_SCL_PIN)
#define I2C1_SCL_H           (I2C1_SCL_GPIO->BSRRL = I2C1_SCL_PIN)

#define I2C1_SDA_L           (I2C1_SDA_GPIO->BSRRH = I2C1_SDA_PIN)
#define I2C1_SDA_H           (I2C1_SDA_GPIO->BSRRL = I2C1_SDA_PIN)

#define I2C1_SDA_R           (I2C1_SDA_GPIO->IDR & I2C1_SDA_PIN)

void I2C1_IO_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
unsigned char I2C1_WaitAck(void);
void I2C1_Ack(void);
void I2C1_NAck(void);
void I2C1_WriteByte(unsigned char data);
unsigned char I2C1_ReadByte(unsigned char ack);

/*****  I2C2  *****/

#define I2C2_SCL_LCK         RCC_AHB1Periph_GPIOD
#define I2C2_SCL_GPIO        GPIOD
#define I2C2_SCL_PIN         GPIO_Pin_13

#define I2C2_SDA_LCK         RCC_AHB1Periph_GPIOD
#define I2C2_SDA_GPIO        GPIOD
#define I2C2_SDA_PIN         GPIO_Pin_12

#define I2C2_SDA_NUM         12

#define I2C2_SDA_IN          do{I2C2_SDA_GPIO->MODER &= ~(3<<I2C2_SDA_NUM*2); I2C2_SDA_GPIO->MODER |= (0<<I2C2_SDA_NUM*2);}while(0)
#define I2C2_SDA_OUT         do{I2C2_SDA_GPIO->MODER &= ~(3<<I2C2_SDA_NUM*2); I2C2_SDA_GPIO->MODER |= (1<<I2C2_SDA_NUM*2);}while(0)

#define I2C2_SCL_L           (I2C2_SCL_GPIO->BSRRH = I2C2_SCL_PIN)
#define I2C2_SCL_H           (I2C2_SCL_GPIO->BSRRL = I2C2_SCL_PIN)

#define I2C2_SDA_L           (I2C2_SDA_GPIO->BSRRH = I2C2_SDA_PIN)
#define I2C2_SDA_H           (I2C2_SDA_GPIO->BSRRL = I2C2_SDA_PIN)

#define I2C2_SDA_R           (I2C2_SDA_GPIO->IDR & I2C2_SDA_PIN)

void I2C2_IO_Init(void);
void I2C2_Start(void);
void I2C2_Stop(void);
unsigned char I2C2_WaitAck(void);
void I2C2_Ack(void);
void I2C2_NAck(void);
void I2C2_WriteByte(unsigned char data);
unsigned char I2C2_ReadByte(unsigned char ack);

/*****  I2C3  *****/

#define I2C3_SCL_LCK         RCC_AHB1Periph_GPIOE
#define I2C3_SCL_GPIO        GPIOE
#define I2C3_SCL_PIN         GPIO_Pin_3

#define I2C3_SDA_LCK         RCC_AHB1Periph_GPIOE
#define I2C3_SDA_GPIO        GPIOE
#define I2C3_SDA_PIN         GPIO_Pin_4

#define I2C3_SDA_NUM         4

#define I2C3_SDA_IN          do{I2C3_SDA_GPIO->MODER &= ~(3<<I2C3_SDA_NUM*2); I2C3_SDA_GPIO->MODER |= (0<<I2C3_SDA_NUM*2);}while(0)
#define I2C3_SDA_OUT         do{I2C3_SDA_GPIO->MODER &= ~(3<<I2C3_SDA_NUM*2); I2C3_SDA_GPIO->MODER |= (1<<I2C3_SDA_NUM*2);}while(0)

#define I2C3_SCL_L           (I2C3_SCL_GPIO->BSRRH = I2C3_SCL_PIN)
#define I2C3_SCL_H           (I2C3_SCL_GPIO->BSRRL = I2C3_SCL_PIN)

#define I2C3_SDA_L           (I2C3_SDA_GPIO->BSRRH = I2C3_SDA_PIN)
#define I2C3_SDA_H           (I2C3_SDA_GPIO->BSRRL = I2C3_SDA_PIN)

#define I2C3_SDA_R           (I2C3_SDA_GPIO->IDR & I2C3_SDA_PIN)

void I2C3_IO_Init(void);
void I2C3_Start(void);
void I2C3_Stop(void);
unsigned char I2C3_WaitAck(void);
void I2C3_Ack(void);
void I2C3_NAck(void);
void I2C3_WriteByte(unsigned char data);
unsigned char I2C3_ReadByte(unsigned char ack);


#endif
