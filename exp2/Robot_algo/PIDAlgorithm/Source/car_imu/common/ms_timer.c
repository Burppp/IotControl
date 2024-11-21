#include "common/ms_timer.h"

/*********************************************************************************************
* 名称：TIM7_IRQHandler
* 功能：基本定时器TIM7中断处理函数
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void TIM7_IRQHandler(void) {
  extern void SysTick_Handler(void);
	extern void timerHandler(void);
	
  if(TIM_GetITStatus(MS_TIMER, TIM_IT_Update) == SET) {
    TIM_ClearITPendingBit(MS_TIMER, TIM_IT_Update);
		timerHandler();
  }
}

/*********************************************************************************************
* 名称：ms_Timer_Init
* 功能：ms定时器初始化
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void ms_Timer_Init(void) {
  TIM_TimeBaseInitTypeDef	TIM_TimeBaseInitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  MS_TIMER_CLK_INIT(MS_TIMER_CLK, ENABLE);
  //定时器7APB时钟频率：84MHz
  TIM_DeInit(MS_TIMER);
  TIM_TimeBaseInitStruct.TIM_Prescaler   = 84 - 1; //分频频率：
  TIM_TimeBaseInitStruct.TIM_Period      = 1000-1;    //1ms一次中断
  TIM_TimeBaseInit(MS_TIMER, &TIM_TimeBaseInitStruct);

  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //抢占式优先级
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 7;         //响应式优先级
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannel = MS_TIMER_INT_ID;
  NVIC_Init(&NVIC_InitStruct);
  
  TIM_ClearFlag(MS_TIMER, TIM_FLAG_Update);
  TIM_ITConfig(MS_TIMER, TIM_IT_Update, ENABLE);
  TIM_Cmd(MS_TIMER, ENABLE);
}

uint32_t ms_Timer_GetCount() {
	return TIM_GetCounter(MS_TIMER);
}
