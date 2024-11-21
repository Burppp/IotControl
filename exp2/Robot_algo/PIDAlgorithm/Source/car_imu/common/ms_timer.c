#include "common/ms_timer.h"

/*********************************************************************************************
* ���ƣ�TIM7_IRQHandler
* ���ܣ�������ʱ��TIM7�жϴ�����
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
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
* ���ƣ�ms_Timer_Init
* ���ܣ�ms��ʱ����ʼ��
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void ms_Timer_Init(void) {
  TIM_TimeBaseInitTypeDef	TIM_TimeBaseInitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  MS_TIMER_CLK_INIT(MS_TIMER_CLK, ENABLE);
  //��ʱ��7APBʱ��Ƶ�ʣ�84MHz
  TIM_DeInit(MS_TIMER);
  TIM_TimeBaseInitStruct.TIM_Prescaler   = 84 - 1; //��ƵƵ�ʣ�
  TIM_TimeBaseInitStruct.TIM_Period      = 1000-1;    //1msһ���ж�
  TIM_TimeBaseInit(MS_TIMER, &TIM_TimeBaseInitStruct);

  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //��ռʽ���ȼ�
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 7;         //��Ӧʽ���ȼ�
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
