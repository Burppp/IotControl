#ifndef __MOTOR_H_
#define __MOTOR_H_
#include "stm32f4xx.h"


#define MOTOR_NUM1              0x01
#define MOTOR_NUM2              0x02
#define MOTOR_NUM3              0x04
#define MOTOR_NUM4              0x08
#define MOTOR_NUM5              0x10
#define MOTOR_NUM6              0x20
#define MOTOR_LEFT              0x07
#define MOTOR_RIGHT             0x38
#define MOTOR_ALL               0x3F
//�����ķ����Ҫ���������Խ��ߵĳ��֣�MOTOR_DIAG_16ָ��1����6��ɵĶԽ��ߣ�MOTOR_DIAG_34ָ��3����4��ɵĶԽ���
#define MOTOR_DIAG_16           0x21
#define MOTOR_DIAG_34           0x0C

void motor_Init(void);
void set_motorSpeed(unsigned char id, signed char speed);

#endif
