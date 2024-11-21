#ifndef __CarMove_H_
#define __CarMove_H_
#include "stm32f4xx.h"

extern int8_t Car_Direction, Car_Speed;
void car_Init(void);
void car_Move(void);

#endif
