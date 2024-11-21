#include "Motor/motor.h"
#include "PCA9685/pca9685.h"

void motor_Init(void)
{
  PCA9685_Init();
  PCA9685_SetPWMFreq(PCA9685_ADDR1, 1000);
  set_motorSpeed(MOTOR_ALL, 0);//所有电机速度设为0
}

/*********************************************************************************************
* 名称：set_motorSpeed
* 功能：设置各车轮速度
* 参数：id：车轮编号，车轮1-6对应bit0-bit5，可一次控制多个车轮
*       speed：-100~100，>0时正转，<0时反转；数值对应电机占空比0-100%
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void set_motorSpeed(unsigned char id, signed char speed) {
  unsigned short paramOn1, paramOff1;
  unsigned short paramOn2, paramOff2;
  paramOn1 = paramOn2 = 0;
  if(speed > 0) {
    paramOff1 = 0;
    paramOff2 = 4095 * (speed * 0.01);
  } else if(speed < 0) {
    paramOff1 = 4095 * (-speed * 0.01);
    paramOff2  = 0;
  } else {
    paramOff1 = paramOff2 = 0;
  }
  if(id & MOTOR_NUM1) {
    PCA9685_SetPWM(PCA9685_ADDR1, 0, paramOn1, paramOff1);
    PCA9685_SetPWM(PCA9685_ADDR1, 1, paramOn2, paramOff2);
  }
  if(id & MOTOR_NUM2) {
    PCA9685_SetPWM(PCA9685_ADDR1, 2, paramOn1, paramOff1);
    PCA9685_SetPWM(PCA9685_ADDR1, 3, paramOn2, paramOff2);
  }
  if(id & MOTOR_NUM3) {
    PCA9685_SetPWM(PCA9685_ADDR1, 4, paramOn1, paramOff1);
    PCA9685_SetPWM(PCA9685_ADDR1, 5, paramOn2, paramOff2);
  }
  if(id & MOTOR_NUM4) {
    PCA9685_SetPWM(PCA9685_ADDR1, 6, paramOn2, paramOff2);
    PCA9685_SetPWM(PCA9685_ADDR1, 7, paramOn1, paramOff1);
  }
  if(id & MOTOR_NUM5) {
    PCA9685_SetPWM(PCA9685_ADDR1, 8, paramOn2, paramOff2);
    PCA9685_SetPWM(PCA9685_ADDR1, 9, paramOn1, paramOff1);
  }
  if(id & MOTOR_NUM6) {
    PCA9685_SetPWM(PCA9685_ADDR1, 10, paramOn2, paramOff2);
    PCA9685_SetPWM(PCA9685_ADDR1, 11, paramOn1, paramOff1);
  }
}
