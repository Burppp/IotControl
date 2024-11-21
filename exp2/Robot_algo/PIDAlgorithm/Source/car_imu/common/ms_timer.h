#ifndef _MS_TIMER_H_
#define _MS_TIMER_H_

#include "stm32f4xx.h"

#define MS_TIMER           TIM7
#define MS_TIMER_CLK       RCC_APB1Periph_TIM7
#define MS_TIMER_CLK_INIT  RCC_APB1PeriphClockCmd
#define MS_TIMER_INT_ID    TIM7_IRQn

void ms_Timer_Init(void);
uint32_t ms_Timer_GetCount(void);

#endif //_MS_TIMER_H_
