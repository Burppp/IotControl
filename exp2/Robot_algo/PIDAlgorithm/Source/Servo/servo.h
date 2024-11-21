#ifndef __SERVO_H_
#define __SERVO_H_
#include "stm32f4xx.h"

//Uart Device
#define SERVO_UART            UART4
#define SERVO_UART_CLK        RCC_APB1Periph_UART4
#define SERVO_UART_CLK_INIT   RCC_APB1PeriphClockCmd
#define SERVO_UART_IRQn       UART4_IRQn
#define SERVO_UART_IRQHandler UART4_IRQHandler
//Signal Pin
#define SERVO_PORT            GPIOC
#define SERVO_PIN             GPIO_Pin_10
#define SERVO_CLK             RCC_AHB1Periph_GPIOC
#define SERVO_PINSOURCE       GPIO_PinSource10
#define SERVO_AF_UART         GPIO_AF_UART4
//Timer
/*
#define SERVO_TIMER           TIM4
#define SERVO_TIMER_CLK       RCC_APB1Periph_TIM4
#define SERVO_TIMER_CLK_INIT  RCC_APB1PeriphClockCmd
#define SERVO_TIMER_IRQn      TIM4_IRQn
#define SERVO_TIMER_IRQHandler  TIM4_IRQHandler
*/

extern volatile uint16_t servo_Timer_Count;

void SERVO_TIMER_IRQHandler(void);
void SERVO_UART_IRQHandler(void);
void servo_Init(void);
void get_servoAngle(int16_t angle[]);
void set_servoAngle(int16_t angle[], uint16_t time, uint8_t *pset_flag);

#endif
