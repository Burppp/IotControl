/*********************************************************************************************
* 文件：motor_speed.c
* 作者：Cage 2019.4.8
* 说明：电机测速
* 修改：
* 注释：
*********************************************************************************************/
#ifndef __MT_SPEED_H__
#define	__MT_SPEED_H__

#include "stm32f4xx.h"

#ifndef PI
#define PI 3.14159265358979f
#endif
//小车物理参数设置
#define ROS_CAR   0
#define ZI_ROBOT  1
/*********************************在此处修改小车类型*************************************/
#define CAR_TYPE  ZI_ROBOT
/***************************************************************************************/
#if CAR_TYPE==ROS_CAR
  #define HOARE_NUM   11.f          /*码盘上的霍尔元件数量，每转一圈出现的脉冲数量即为此值*/
  #define GEAR_RATIO  90.f        /*电机齿轮减速比*/
  #define D_WHEEL     77.f         /*车轮直径(mm)*/
  #define CAR_WIDTH   200.0f        /*小车两轮横距(mm)*/
#elif CAR_TYPE==ZI_ROBOT
  #define HOARE_NUM   11.f          /*码盘上的霍尔元件数量，每转一圈出现的脉冲数量即为此值*/
  #define GEAR_RATIO  34.02f        /*电机齿轮减速比*/
  #define D_WHEEL     50.0f         /*车轮直径(mm)*/
  #define CAR_WIDTH   196.0f        /*小车两轮横距(mm)*/
#else
  #define HOARE_NUM   11.f          /*码盘上的霍尔元件数量，每转一圈出现的脉冲数量即为此值*/
  #define GEAR_RATIO  34.02f        /*电机齿轮减速比*/
  #define D_WHEEL     74.0f         /*车轮直径(mm)*/
  #define CAR_WIDTH   175.0f        /*小车两轮横距(mm)*/
#endif
#define C_WHEEL     (PI*D_WHEEL)  /*车轮周长(mm)*/

/***************************各车轮定时器定义***************************/
//1号轮：T4C1,T4C2
#define SPEED_H1_TIMER            TIM4
#define SPEED_H1_TIMER_CLK        RCC_APB1Periph_TIM4
#define SPEED_H1_TIMER_CLK_INIT   RCC_APB1PeriphClockCmd
#define SPEED_H1_TIMER_NVIC       TIM4_IRQn
#define SPEED_H1A_ICChannel       TIM_Channel_1
#define SPEED_H1A_IT_CC           TIM_IT_CC1
#define SPEED_H1B_ICChannel       TIM_Channel_2
#define SPEED_H1B_IT_CC           TIM_IT_CC2
//2号轮：T3C1,T3C2
#define SPEED_H2_TIMER            TIM3
#define SPEED_H2_TIMER_CLK        RCC_APB1Periph_TIM3
#define SPEED_H2_TIMER_CLK_INIT   RCC_APB1PeriphClockCmd
#define SPEED_H2_TIMER_NVIC       TIM3_IRQn
#define SPEED_H2A_ICChannel       TIM_Channel_1
#define SPEED_H2A_IT_CC           TIM_IT_CC1
#define SPEED_H2B_ICChannel       TIM_Channel_2
#define SPEED_H2B_IT_CC           TIM_IT_CC2
//3号轮：T4C3,T4C4
#define SPEED_H3_TIMER            TIM4
#define SPEED_H3_TIMER_CLK        RCC_APB1Periph_TIM4
#define SPEED_H3_TIMER_CLK_INIT   RCC_APB1PeriphClockCmd
#define SPEED_H3_TIMER_NVIC       TIM4_IRQn
#define SPEED_H3A_ICChannel       TIM_Channel_3
#define SPEED_H3A_IT_CC           TIM_IT_CC3
#define SPEED_H3B_ICChannel       TIM_Channel_4
#define SPEED_H3B_IT_CC           TIM_IT_CC4
//4号轮：T9C1,T9C2
#define SPEED_H4_TIMER            TIM9
#define SPEED_H4_TIMER_CLK        RCC_APB2Periph_TIM9
#define SPEED_H4_TIMER_CLK_INIT   RCC_APB2PeriphClockCmd
#define SPEED_H4_TIMER_NVIC       TIM1_BRK_TIM9_IRQn
#define SPEED_H4A_ICChannel       TIM_Channel_1
#define SPEED_H4A_IT_CC           TIM_IT_CC1
#define SPEED_H4B_ICChannel       TIM_Channel_2
#define SPEED_H4B_IT_CC           TIM_IT_CC2
//5号轮：T2C2,T2C1
#define SPEED_H5_TIMER            TIM2
#define SPEED_H5_TIMER_CLK        RCC_APB1Periph_TIM2
#define SPEED_H5_TIMER_CLK_INIT   RCC_APB1PeriphClockCmd
#define SPEED_H5_TIMER_NVIC       TIM2_IRQn
#define SPEED_H5A_ICChannel       TIM_Channel_2
#define SPEED_H5A_IT_CC           TIM_IT_CC2
#define SPEED_H5B_ICChannel       TIM_Channel_1
#define SPEED_H5B_IT_CC           TIM_IT_CC1
//6号轮：T1C1,T1C2
#define SPEED_H6_TIMER            TIM1
#define SPEED_H6_TIMER_CLK        RCC_APB2Periph_TIM1
#define SPEED_H6_TIMER_CLK_INIT   RCC_APB2PeriphClockCmd
#define SPEED_H6_TIMER_NVIC       TIM1_CC_IRQn
#define SPEED_H6A_ICChannel       TIM_Channel_1
#define SPEED_H6A_IT_CC           TIM_IT_CC1
#define SPEED_H6B_ICChannel       TIM_Channel_2
#define SPEED_H6B_IT_CC           TIM_IT_CC2
//用于溢出中断的定时器：必须是通用定时器之一（TIM2-5/9-14）
#define SPEED_UPDATE_TIMER        SPEED_H2_TIMER

/*************************各车轮输入捕获引脚定义**************************/
//1号轮：PB6,PB7,T4
#define SPEED_H1A_PORT    GPIOB
#define SPEED_H1A_PIN     GPIO_Pin_6
#define SPEED_H1A_CLK     RCC_AHB1Periph_GPIOB
#define SPEED_H1A_SOURCE  GPIO_PinSource6
#define SPEED_H1A_AF      GPIO_AF_TIM4
#define SPEED_H1B_PORT    GPIOB
#define SPEED_H1B_PIN     GPIO_Pin_7
#define SPEED_H1B_CLK     RCC_AHB1Periph_GPIOB
#define SPEED_H1B_SOURCE  GPIO_PinSource7
#define SPEED_H1B_AF      GPIO_AF_TIM4
//2号轮：PB4,PB5,T3
#define SPEED_H2A_PORT    GPIOB
#define SPEED_H2A_PIN     GPIO_Pin_4
#define SPEED_H2A_CLK     RCC_AHB1Periph_GPIOB
#define SPEED_H2A_SOURCE  GPIO_PinSource4
#define SPEED_H2A_AF      GPIO_AF_TIM3
#define SPEED_H2B_PORT    GPIOB
#define SPEED_H2B_PIN     GPIO_Pin_5
#define SPEED_H2B_CLK     RCC_AHB1Periph_GPIOB
#define SPEED_H2B_SOURCE  GPIO_PinSource5
#define SPEED_H2B_AF      GPIO_AF_TIM3
//3号轮：PD14,PD15,T4
#define SPEED_H3A_PORT    GPIOD
#define SPEED_H3A_PIN     GPIO_Pin_14
#define SPEED_H3A_CLK     RCC_AHB1Periph_GPIOD
#define SPEED_H3A_SOURCE  GPIO_PinSource14
#define SPEED_H3A_AF      GPIO_AF_TIM4
#define SPEED_H3B_PORT    GPIOD
#define SPEED_H3B_PIN     GPIO_Pin_15
#define SPEED_H3B_CLK     RCC_AHB1Periph_GPIOD
#define SPEED_H3B_SOURCE  GPIO_PinSource15
#define SPEED_H3B_AF      GPIO_AF_TIM4
//4号轮：PE5,PE6,T9
#define SPEED_H4A_PORT    GPIOE
#define SPEED_H4A_PIN     GPIO_Pin_5
#define SPEED_H4A_CLK     RCC_AHB1Periph_GPIOE
#define SPEED_H4A_SOURCE  GPIO_PinSource5
#define SPEED_H4A_AF      GPIO_AF_TIM9
#define SPEED_H4B_PORT    GPIOE
#define SPEED_H4B_PIN     GPIO_Pin_6
#define SPEED_H4B_CLK     RCC_AHB1Periph_GPIOE
#define SPEED_H4B_SOURCE  GPIO_PinSource6
#define SPEED_H4B_AF      GPIO_AF_TIM9
//5号轮：PB3,PA15,T2
#define SPEED_H5A_PORT    GPIOB
#define SPEED_H5A_PIN     GPIO_Pin_3
#define SPEED_H5A_CLK     RCC_AHB1Periph_GPIOB
#define SPEED_H5A_SOURCE  GPIO_PinSource3
#define SPEED_H5A_AF      GPIO_AF_TIM2
#define SPEED_H5B_PORT    GPIOA
#define SPEED_H5B_PIN     GPIO_Pin_15
#define SPEED_H5B_CLK     RCC_AHB1Periph_GPIOA
#define SPEED_H5B_SOURCE  GPIO_PinSource15
#define SPEED_H5B_AF      GPIO_AF_TIM2
//6号轮：PE9,PE11,T1
#define SPEED_H6A_PORT    GPIOE
#define SPEED_H6A_PIN     GPIO_Pin_9
#define SPEED_H6A_CLK     RCC_AHB1Periph_GPIOE
#define SPEED_H6A_SOURCE  GPIO_PinSource9
#define SPEED_H6A_AF      GPIO_AF_TIM1
#define SPEED_H6B_PORT    GPIOE
#define SPEED_H6B_PIN     GPIO_Pin_11
#define SPEED_H6B_CLK     RCC_AHB1Periph_GPIOE
#define SPEED_H6B_SOURCE  GPIO_PinSource11
#define SPEED_H6B_AF      GPIO_AF_TIM1
//6个车轮12相索引编号；内部计算速度时，0-5指6轮A相速度；外部获取速度时，0-5指6轮转速
#define SPEED_H1A_INDEX   0
#define SPEED_H2A_INDEX   1
#define SPEED_H3A_INDEX   2
#define SPEED_H4A_INDEX   3
#define SPEED_H5A_INDEX   4
#define SPEED_H6A_INDEX   5
#define SPEED_H1B_INDEX   6
#define SPEED_H2B_INDEX   7
#define SPEED_H3B_INDEX   8
#define SPEED_H4B_INDEX   9
#define SPEED_H5B_INDEX   10
#define SPEED_H6B_INDEX   11
#define SPEED_ALL_INDEX   12  /*所有车轮--用于清零数组*/
#define SPEED_ERR_INDEX   13  /*错误参数*/

extern volatile int Time_Update_Count_100MS;

void ms_IRQ(uint8_t ch, uint32_t ccrx);
void ms_Clear(uint8_t ch);
float ms_Get_Speed(uint8_t ch);
void ms_Module_Init(void);

#endif  //__MT_SPEED_H__
