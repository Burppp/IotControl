/*********************************************************************************************
* �ļ���motor_speed.c
* ���ߣ�Cage 2019.4.8
* ˵�����������
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
#include "stm32f4xx.h"
#include "string.h"
#include "MotorSpeed/motor_speed.h"
#include "Servo/servo.h"

#define APB1_CLOCK  84000000    //APB1ʱ��Ƶ�ʣ�84MHz
#define APB2_CLOCK  168000000   //APB2ʱ��Ƶ�ʣ�168MHz
#define TIMER_CLOCK 100000      //��Ƶ��Ķ�ʱ��Ƶ�ʣ�100kHz
#define TIMER_COUNT 10000       //��ʱ���������ֵ��10000

#define CAR_FILTER_N    3
static volatile float _s_Motor_Speed[SPEED_ALL_INDEX][CAR_FILTER_N];//�洢���ת��ֵ�����飨ת/s��
volatile int Time_Update_Count_100MS = 0;                            //ʱ����¼�����ÿ100ms��1

/*********************************************************************************************
* ���ƣ�ms_Get_Speed
* ���ܣ���ȡ�����ٶ�
* ���������ֱ��
* ���أ������ٶȣ�mm/s��
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
float ms_Get_Speed(uint8_t ch) {
  if(ch > SPEED_H6A_INDEX) return 0.f;
  
  float buf[CAR_FILTER_N];
  uint8_t i,j;
  float temp, sum = 0.f;
  //��λֵƽ���˲��㷨
  for(i = 0; i < CAR_FILTER_N; i++) {
    buf[i] = _s_Motor_Speed[ch][i];
  }
  for(i = 0; i < CAR_FILTER_N-1; i++) {
    for(j = 0; j < CAR_FILTER_N-i-1; j++) {
      if(buf[j] < buf[j+1]) {
        temp = buf[j];
        buf[j] = buf[j+1];
        buf[j+1] = temp;
      }
    }
  }
  for(i = 1; i < CAR_FILTER_N-1; i++) {
    sum += buf[i];
  }
  return (sum / (CAR_FILTER_N-2.f)) * C_WHEEL;
}

/*********************************************************************************************
* ���ƣ�ms_IRQ
* ���ܣ���������ж�--��������µĵ��ת��
* ���������ֱ��
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void ms_IRQ(uint8_t ch, uint32_t ccrx) {
  static uint8_t s_Filter_Index[SPEED_ALL_INDEX];//�˲���������ÿ�ν�����ֵ�����ڲ�ͬ������λ��
  static uint32_t s_CCRX_Last[SPEED_ALL_INDEX];//�ϴε�CCRXֵ
  float period;//��������֮���ʱ���������������ڣ�ms/T��
  
  if(ccrx > s_CCRX_Last[ch]) {
    period = (ccrx - s_CCRX_Last[ch]) / (TIMER_CLOCK / 1000.f);
  } else {//������һ���������
    period = (TIMER_COUNT - s_CCRX_Last[ch] + ccrx) / (TIMER_CLOCK / 1000.f);
  }
  _s_Motor_Speed[ch][s_Filter_Index[ch]] = (1000.f / period) / HOARE_NUM / GEAR_RATIO;
  s_Filter_Index[ch] = (s_Filter_Index[ch]+1) % CAR_FILTER_N;
  s_CCRX_Last[ch] = ccrx;
}

/*********************************************************************************************
* ���ƣ�ms_Clear
* ���ܣ�������ת������
* ���������ֱ��
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void ms_Clear(uint8_t ch) {
  uint8_t i;
  if(ch < SPEED_ALL_INDEX) {
    for(i = 0; i < CAR_FILTER_N; i++) {
      _s_Motor_Speed[ch][i] = 0.f;
    }
  } else if(ch == SPEED_ALL_INDEX) {
    for(i = 0; i < SPEED_ALL_INDEX; i++) {
      ms_Clear(i);
    }
  }
}

/*********************************************************************************************
* ���ƣ�_ms_Get_Channel
* ���ܣ�ͨ����ʱ����š��ж�ͨ����ţ���ȡ��Ӧ�ĳ��ֱ��
* ��������ʱ����ţ��ж�ͨ�����
* ���أ����ֱ��
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
static uint8_t _ms_Get_Channel(TIM_TypeDef* TIMx, uint16_t TIM_IT) {
  if(TIMx == SPEED_H1_TIMER) {
    if(TIM_IT == SPEED_H1A_IT_CC) {
      return SPEED_H1A_INDEX;
    } else if(TIM_IT == SPEED_H1B_IT_CC) {
      return SPEED_H1B_INDEX;
    }
  }
  if(TIMx == SPEED_H2_TIMER) {//ע��������else if�����������ֶ�ʱ����ͬʱ�����ں���ĳ��ֵ�if���޷������
    if(TIM_IT == SPEED_H2A_IT_CC) {
      return SPEED_H2A_INDEX;
    } else if(TIM_IT == SPEED_H2B_IT_CC) {
      return SPEED_H2B_INDEX;
    }
  }
  if(TIMx == SPEED_H3_TIMER) {
    if(TIM_IT == SPEED_H3A_IT_CC) {
      return SPEED_H3A_INDEX;
    } else if(TIM_IT == SPEED_H3B_IT_CC) {
      return SPEED_H3B_INDEX;
    }
  }
  if(TIMx == SPEED_H4_TIMER) {
    if(TIM_IT == SPEED_H4A_IT_CC) {
      return SPEED_H4A_INDEX;
    } else if(TIM_IT == SPEED_H4B_IT_CC) {
      return SPEED_H4B_INDEX;
    }
  }
  if(TIMx == SPEED_H5_TIMER) {
    if(TIM_IT == SPEED_H5A_IT_CC) {
      return SPEED_H5A_INDEX;
    } else if(TIM_IT == SPEED_H5B_IT_CC) {
      return SPEED_H5B_INDEX;
    }
  }
  if(TIMx == SPEED_H6_TIMER) {
    if(TIM_IT == SPEED_H6A_IT_CC) {
      return SPEED_H6A_INDEX;
    } else if(TIM_IT == SPEED_H6B_IT_CC) {
      return SPEED_H6B_INDEX;
    }
  }
  return SPEED_ERR_INDEX;
}

/*********************************************************************************************
* ���ƣ�TIMx_IRQHandler
* ���ܣ��ǻ�����ʱ���жϴ�����
* ������TIMx: ��ʱ�����
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
static void TIMx_IRQHandler(TIM_TypeDef* TIMx) {
  static uint16_t s_Speed_Count[SPEED_ALL_INDEX], s_Count_Last[SPEED_ALL_INDEX], s_Clear_Count[SPEED_ALL_INDEX];
  uint8_t channel = SPEED_ERR_INDEX;
  if(TIM_GetITStatus(TIMx, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIMx, TIM_IT_Update);
    Time_Update_Count_100MS++;
    servo_Timer_Count += 10;
    for(uint8_t i = 0; i < SPEED_ALL_INDEX; i++) {
      if(s_Count_Last[i] == s_Speed_Count[i]) {//300ms��û�в���������ʱ����Ϊ�����ͣת--100msһ�������ض�Ӧ��ת��Ϊ0.027ת/��
        if(s_Clear_Count[i] < 3) {
          s_Clear_Count[i]++;
        } else if(s_Clear_Count[i] == 3) {
          ms_Clear(i);
          s_Clear_Count[i] = 0xFF;
        }
      } else {
        s_Clear_Count[i] = 0;
      }
      s_Count_Last[i] = s_Speed_Count[i];
    }
  }
  if(TIM_GetITStatus(TIMx, TIM_IT_CC1) != RESET) {
    TIM_ClearITPendingBit(TIMx, TIM_IT_CC1);
    channel = _ms_Get_Channel(TIMx, TIM_IT_CC1);
    if(channel == SPEED_ERR_INDEX) return;
    ms_IRQ(channel, TIM_GetCapture1(TIMx));
    s_Speed_Count[channel]++;
  }
  if(TIM_GetITStatus(TIMx, TIM_IT_CC2) != RESET) {
    TIM_ClearITPendingBit(TIMx, TIM_IT_CC2);
    channel = _ms_Get_Channel(TIMx, TIM_IT_CC2);
    if(channel == SPEED_ERR_INDEX) return;
    ms_IRQ(channel, TIM_GetCapture2(TIMx));
    s_Speed_Count[channel]++;
  }
  if(TIM_GetITStatus(TIMx, TIM_IT_CC3) != RESET) {
    TIM_ClearITPendingBit(TIMx, TIM_IT_CC3);
    channel = _ms_Get_Channel(TIMx, TIM_IT_CC3);
    if(channel == SPEED_ERR_INDEX) return;
    ms_IRQ(channel, TIM_GetCapture3(TIMx));
    s_Speed_Count[channel]++;
  }
  if(TIM_GetITStatus(TIMx, TIM_IT_CC4) != RESET) {
    TIM_ClearITPendingBit(TIMx, TIM_IT_CC4);
    channel = _ms_Get_Channel(TIMx, TIM_IT_CC4);
    if(channel == SPEED_ERR_INDEX) return;
    ms_IRQ(channel, TIM_GetCapture4(TIMx));
    s_Speed_Count[channel]++;
  }
}

/*********************************************************************************************
* ���ƣ�TIM1_IRQHandler-TIM5_IRQHandler��TIM8_IRQHandler-TIM14_IRQHandler
* ���ܣ��ǻ�����ʱ���жϴ�����
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void TIM1_CC_IRQHandler(void)             {TIMx_IRQHandler(TIM1);}
void TIM2_IRQHandler(void)                {TIMx_IRQHandler(TIM2);}
void TIM3_IRQHandler(void)                {TIMx_IRQHandler(TIM3);}
void TIM4_IRQHandler(void)                {TIMx_IRQHandler(TIM4);}
void TIM5_IRQHandler(void)                {TIMx_IRQHandler(TIM5);}
void TIM8_CC_IRQHandler(void)             {TIMx_IRQHandler(TIM8);}
void TIM1_BRK_TIM9_IRQHandler(void)       {TIMx_IRQHandler(TIM9);}
void TIM1_UP_TIM10_IRQHandler(void)       {TIMx_IRQHandler(TIM10);}
void TIM1_TRG_COM_TIM11_IRQHandler(void)  {TIMx_IRQHandler(TIM11);}
void TIM8_BRK_TIM12_IRQHandler(void)      {TIMx_IRQHandler(TIM12);}
void TIM8_UP_TIM13_IRQHandler(void)       {TIMx_IRQHandler(TIM13);}
void TIM8_TRG_COM_TIM14_IRQHandler(void)  {TIMx_IRQHandler(TIM14);}

/*********************************************************************************************
* ���ƣ�_ms_GPIO_Init
* ���ܣ��������GPIO��ʼ��
* ��������
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void _ms_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  
  RCC_AHB1PeriphClockCmd(SPEED_H1A_CLK | SPEED_H1B_CLK | SPEED_H2A_CLK | SPEED_H2B_CLK |\
                         SPEED_H3A_CLK | SPEED_H3B_CLK | SPEED_H4A_CLK | SPEED_H4B_CLK |\
                         SPEED_H5A_CLK | SPEED_H5B_CLK | SPEED_H6A_CLK | SPEED_H6B_CLK, ENABLE);
  
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  
  GPIO_InitStruct.GPIO_Pin = SPEED_H1A_PIN;
  GPIO_Init(SPEED_H1A_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H1B_PIN;
  GPIO_Init(SPEED_H1B_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H2A_PIN;
  GPIO_Init(SPEED_H2A_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H2B_PIN;
  GPIO_Init(SPEED_H2B_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H3A_PIN;
  GPIO_Init(SPEED_H3A_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H3B_PIN;
  GPIO_Init(SPEED_H3B_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H4A_PIN;
  GPIO_Init(SPEED_H4A_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H4B_PIN;
  GPIO_Init(SPEED_H4B_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H5A_PIN;
  GPIO_Init(SPEED_H5A_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H5B_PIN;
  GPIO_Init(SPEED_H5B_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H6A_PIN;
  GPIO_Init(SPEED_H6A_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = SPEED_H6B_PIN;
  GPIO_Init(SPEED_H6B_PORT, &GPIO_InitStruct);
  
  GPIO_PinAFConfig(SPEED_H1A_PORT, SPEED_H1A_SOURCE, SPEED_H1A_AF);
  GPIO_PinAFConfig(SPEED_H1B_PORT, SPEED_H1B_SOURCE, SPEED_H1B_AF);
  GPIO_PinAFConfig(SPEED_H2A_PORT, SPEED_H2A_SOURCE, SPEED_H2A_AF);
  GPIO_PinAFConfig(SPEED_H2B_PORT, SPEED_H2B_SOURCE, SPEED_H2B_AF);
  GPIO_PinAFConfig(SPEED_H3A_PORT, SPEED_H3A_SOURCE, SPEED_H3A_AF);
  GPIO_PinAFConfig(SPEED_H3B_PORT, SPEED_H3B_SOURCE, SPEED_H3B_AF);
  GPIO_PinAFConfig(SPEED_H4A_PORT, SPEED_H4A_SOURCE, SPEED_H4A_AF);
  GPIO_PinAFConfig(SPEED_H4B_PORT, SPEED_H4B_SOURCE, SPEED_H4B_AF);
  GPIO_PinAFConfig(SPEED_H5A_PORT, SPEED_H5A_SOURCE, SPEED_H5A_AF);
  GPIO_PinAFConfig(SPEED_H5B_PORT, SPEED_H5B_SOURCE, SPEED_H5B_AF);
  GPIO_PinAFConfig(SPEED_H6A_PORT, SPEED_H6A_SOURCE, SPEED_H6A_AF);
  GPIO_PinAFConfig(SPEED_H6B_PORT, SPEED_H6B_SOURCE, SPEED_H6B_AF);
}

/*********************************************************************************************
* ���ƣ�_ms_Timer_Init
* ���ܣ�������ٶ�ʱ����ʼ��
* ��������
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void _ms_Timer_Init(void) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  
  SPEED_H1_TIMER_CLK_INIT(SPEED_H1_TIMER_CLK, ENABLE);
  SPEED_H2_TIMER_CLK_INIT(SPEED_H2_TIMER_CLK, ENABLE);
  SPEED_H3_TIMER_CLK_INIT(SPEED_H3_TIMER_CLK, ENABLE);
  SPEED_H4_TIMER_CLK_INIT(SPEED_H4_TIMER_CLK, ENABLE);
  SPEED_H5_TIMER_CLK_INIT(SPEED_H5_TIMER_CLK, ENABLE);
  SPEED_H6_TIMER_CLK_INIT(SPEED_H6_TIMER_CLK, ENABLE);
  
  //APB1:42MHz--TIM2/3/4/5/6/7/12/13/14:84MHz
  TIM_TimeBaseInitStruct.TIM_Prescaler = APB1_CLOCK/TIMER_CLOCK - 1; //Ƶ�ʣ�100kHz
  TIM_TimeBaseInitStruct.TIM_Period = TIMER_COUNT - 1;  //������10000ʱ���--100ms
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(SPEED_H1_TIMER, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(SPEED_H2_TIMER, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(SPEED_H3_TIMER, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(SPEED_H5_TIMER, &TIM_TimeBaseInitStruct);

  //APB2:84MHz--TIM1/8/9/10/11:168MHz
  TIM_TimeBaseInitStruct.TIM_Prescaler = APB2_CLOCK/TIMER_CLOCK - 1; //Ƶ�ʣ�100kHz
  TIM_TimeBaseInitStruct.TIM_Period = TIMER_COUNT - 1;  //������10000ʱ���--100ms
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(SPEED_H4_TIMER, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(SPEED_H6_TIMER, &TIM_TimeBaseInitStruct);
}

/*********************************************************************************************
* ���ƣ�_ms_Capture_Init
* ���ܣ�������ٲ����ܳ�ʼ��
* ��������
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void _ms_Capture_Init(void) {
  TIM_ICInitTypeDef TIM_ICInitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;     //����������
  TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ͨ������Ϊ���룬ICxӳ�䵽TIx��
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;            //ÿ1����Чת��ʱִ�в������
  TIM_ICInitStruct.TIM_ICFilter    = 0;                         //���˲�������fDTSƵ�ʲ�����һ���¼���Ϊһ����Ч����
  
  //1����
  TIM_ICInitStruct.TIM_Channel = SPEED_H1A_ICChannel;
  TIM_ICInit(SPEED_H1_TIMER, &TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_Channel = SPEED_H1B_ICChannel;
  TIM_ICInit(SPEED_H1_TIMER, &TIM_ICInitStruct);
  //2����
  TIM_ICInitStruct.TIM_Channel = SPEED_H2A_ICChannel;
  TIM_ICInit(SPEED_H2_TIMER, &TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_Channel = SPEED_H2B_ICChannel;
  TIM_ICInit(SPEED_H2_TIMER, &TIM_ICInitStruct);
  //3����
  TIM_ICInitStruct.TIM_Channel = SPEED_H3A_ICChannel;
  TIM_ICInit(SPEED_H3_TIMER, &TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_Channel = SPEED_H3B_ICChannel;
  TIM_ICInit(SPEED_H3_TIMER, &TIM_ICInitStruct);
  //4����
  TIM_ICInitStruct.TIM_Channel = SPEED_H4A_ICChannel;
  TIM_ICInit(SPEED_H4_TIMER, &TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_Channel = SPEED_H4B_ICChannel;
  TIM_ICInit(SPEED_H4_TIMER, &TIM_ICInitStruct);
  //5����
  TIM_ICInitStruct.TIM_Channel = SPEED_H5A_ICChannel;
  TIM_ICInit(SPEED_H5_TIMER, &TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_Channel = SPEED_H5B_ICChannel;
  TIM_ICInit(SPEED_H5_TIMER, &TIM_ICInitStruct);
  //6����
  TIM_ICInitStruct.TIM_Channel = SPEED_H6A_ICChannel;
  TIM_ICInit(SPEED_H6_TIMER, &TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_Channel = SPEED_H6B_ICChannel;
  TIM_ICInit(SPEED_H6_TIMER, &TIM_ICInitStruct);
  
  //NVIC�ж�����
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //��ռʽ���ȼ�
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 7;         //��Ӧʽ���ȼ�
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannel = SPEED_H1_TIMER_NVIC;
  NVIC_Init(&NVIC_InitStruct);
  NVIC_InitStruct.NVIC_IRQChannel = SPEED_H2_TIMER_NVIC;
  NVIC_Init(&NVIC_InitStruct);
  NVIC_InitStruct.NVIC_IRQChannel = SPEED_H3_TIMER_NVIC;
  NVIC_Init(&NVIC_InitStruct);
  NVIC_InitStruct.NVIC_IRQChannel = SPEED_H4_TIMER_NVIC;
  NVIC_Init(&NVIC_InitStruct);
  NVIC_InitStruct.NVIC_IRQChannel = SPEED_H5_TIMER_NVIC;
  NVIC_Init(&NVIC_InitStruct);
  NVIC_InitStruct.NVIC_IRQChannel = SPEED_H6_TIMER_NVIC;
  NVIC_Init(&NVIC_InitStruct);
  
  //TIM�ж�����
  TIM_ClearFlag(SPEED_H1_TIMER, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
  TIM_ClearFlag(SPEED_H2_TIMER, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
  TIM_ClearFlag(SPEED_H3_TIMER, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
  TIM_ClearFlag(SPEED_H4_TIMER, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
  TIM_ClearFlag(SPEED_H5_TIMER, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
  TIM_ClearFlag(SPEED_H6_TIMER, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
  TIM_ITConfig(SPEED_H1_TIMER, SPEED_H1A_IT_CC/* | SPEED_H1B_IT_CC*/, ENABLE);//�ݲ�����B����
  TIM_ITConfig(SPEED_H2_TIMER, SPEED_H2A_IT_CC/* | SPEED_H2B_IT_CC*/, ENABLE);
  TIM_ITConfig(SPEED_H3_TIMER, SPEED_H3A_IT_CC/* | SPEED_H3B_IT_CC*/, ENABLE);
  TIM_ITConfig(SPEED_H4_TIMER, SPEED_H4A_IT_CC/* | SPEED_H4B_IT_CC*/, ENABLE);
  TIM_ITConfig(SPEED_H5_TIMER, SPEED_H5A_IT_CC/* | SPEED_H5B_IT_CC*/, ENABLE);
  TIM_ITConfig(SPEED_H6_TIMER, SPEED_H6A_IT_CC/* | SPEED_H6B_IT_CC*/, ENABLE);
  TIM_ITConfig(SPEED_UPDATE_TIMER, TIM_IT_Update, ENABLE);//��ʱ������ж�
  TIM_Cmd(SPEED_H1_TIMER, ENABLE);
  TIM_Cmd(SPEED_H2_TIMER, ENABLE);
  TIM_Cmd(SPEED_H3_TIMER, ENABLE);
  TIM_Cmd(SPEED_H4_TIMER, ENABLE);
  TIM_Cmd(SPEED_H5_TIMER, ENABLE);
  TIM_Cmd(SPEED_H6_TIMER, ENABLE);
}

/*********************************************************************************************
* ���ƣ�ms_Module_Init
* ���ܣ�������ٳ�ʼ��
* ��������
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void ms_Module_Init(void) {
  _ms_GPIO_Init();
  _ms_Timer_Init();
  _ms_Capture_Init();
}
