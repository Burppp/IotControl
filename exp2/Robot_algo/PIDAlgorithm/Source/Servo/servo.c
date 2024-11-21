#include "Servo/servo.h"
#include  <stdio.h>
#include  <string.h>

static uint16_t servo_angle[6];     //����Ƕȣ���ѯ��
static uint8_t  servo_recv_flag;    //����ǶȽ���flag
volatile uint16_t servo_Timer_Count = 0;      //�����ʱ��������־

/*********************************************************************************************
* ���ƣ�_servo_Uart_Send
* ���ܣ����߶�����ڷ���
* ��������������ͷָ�룻���ݳ���
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
static void _servo_Uart_Send(char *pdata, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    USART_SendData(SERVO_UART, *(pdata + i));
    while(USART_GetFlagStatus(SERVO_UART, USART_FLAG_TC) == RESET); 
  } 
}

/*********************************************************************************************
* ���ƣ�set_servoAngle
* ���ܣ�����Ƕ�����
* ������angle��Ҫ���õĽǶ����飬�Ƕȷ�Χ��[500-2500]��time�����߶����ʱ�����
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void get_servoAngle(int16_t angle[]) {
  char cmd[10] = "#000PRAD!";
  for(uint8_t i = 0; i < 4; i++) {
    cmd[3] = i+'0';
    servo_recv_flag = 0;
    servo_Timer_Count = 0;
    _servo_Uart_Send(cmd, strlen(cmd));
    while(servo_recv_flag==0) {
      if(servo_Timer_Count>1) {
        return;//��ʱֱ�ӷ���
      }
    }
    //angle��Χ��-135~135��input��Χ��500~2500��angle = (input-500)/(2500-500)*270-135
    angle[i] = (int16_t)(servo_angle[i]-500) * 270 / (2500-500) - 135;
  }
}

/*********************************************************************************************
* ���ƣ�_servo_angle_trim
* ���ܣ��޼�����Ƕ�����ֵ���Է���������ײ��������������
* ������id--�����ţ�pangle--���������Ƕ�
* ���أ�
* �޸ģ�
*********************************************************************************************/
static void _servo_angle_trim(uint8_t id, int16_t *pangle) {
  switch(id) {
    case 0://����צ���ϣ��Ƕ�����ʱ������צ�ſ����Ƕȷ�Χ��[-54,81]
      if(*pangle > 81) *pangle =81;
      else if(*pangle < -54) *pangle = -54;
    break;
    case 1://����צ��ת���Ƕ�����ʱ������צ˳ʱ����ת��ת���ӻ���צ�ĺ󷽿���
    break;
    case 2://����צת�����Ƕ�����ʱ������צ���ת��
    break;
    case 3://������ת�����Ƕ�����ʱ�����������ת��
    break;
    default:
      *pangle = 0;
    break;
  }
}

/*********************************************************************************************
* ���ƣ�set_servoAngle
* ���ܣ�����Ƕ�����
* ������angle��Ҫ���õĽǶ����飻time�����߶����ʱ����ƣ����ñ�־λ
* ���أ�
* �޸ģ�
* ע�ͣ�������������뷶Χ��[500-2500]
*********************************************************************************************/
void set_servoAngle(int16_t angle[], uint16_t time, uint8_t *pset_flag) {
  char servo_uart_cmd[100];    //�������ָ������
  uint16_t index = 0;
  uint16_t input = 0;
  
  servo_uart_cmd[index++] = '{';
  for(uint8_t i = 0; i < 4; i++) {
    if(((*pset_flag>>i)&0x01) != 0) {//�������ñ�־λ��Чʱ���ŷ��ͽǶ�����ָ��
      *pset_flag &= ~(0x01<<i);//������ñ�־λ
      //����ʵ��������޼�����Ƕȷ�Χ
      _servo_angle_trim(i, &angle[i]);
      //angle��Χ��-135~135��input��Χ��500~2500��input = (angle+135)/270*(2500-500)+500
      input = (angle[i]+135) * (2500-500) / 270 + 500;
      if(input > 2500) { //�ǶȲü�
        input = 2500;
      } else if(input < 500) {
        input = 500;
      }
      /*�������ָ�"#[ID-03]P[Deg-04]T[Time-04]!"*/
      index += sprintf(servo_uart_cmd+index, "#%03dP%04dT%04d!", i, input, time);
    }
  }
  servo_uart_cmd[index++] = '}';
  _servo_Uart_Send(servo_uart_cmd, index);
}

/*********************************************************************************************
* ���ƣ�_servo_Data_Process
* ���ܣ����߶�����ڽ��մ���
* �������жϽ��յ�������
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
static void _servo_Data_Process(char data) {
  static uint8_t servo_id = 0;
  static uint16_t servo_angle_recv;
  static char servo_index;
  
  //#[ID-03]P[Deg-04]!
  switch(servo_index) {
    case 0://#
      if(data == '#') {
        servo_index++;
      } else {
        servo_index = 0;
      }
    break;
    case 1: case 2: case 3://ID��3λ
      if(data>='0' && data<='9') {
        if(servo_index==3) {
          //ID������10��ֻȡ��λ
          servo_id = data-'0';
          if(servo_id > 5) {
            servo_index = 0;
          }
        }
        servo_index++;
      } else {
        servo_index = 0;
      }
    break;
    case 4://P
      if(data == 'P') {
        servo_index++;
        servo_angle_recv = 0;
      } else {
        servo_index = 0;
      }
    break;
    case 5: case 6: case 7: case 8://�Ƕȣ�4λ
      if(data>='0' && data<='9') {
        uint8_t times = 8 - servo_index;
        uint16_t temp = data-'0';
        while(times-->0) {
          temp *= 10;
        }
        servo_angle_recv += temp;
        servo_index++;
      } else {
        servo_index = 0;
      }
    break;
    case 9://!
      if(data == '!') {
        servo_recv_flag = 1;
        servo_angle[servo_id] = servo_angle_recv;
      }
      servo_index = 0;
    default:
      servo_index = 0;
    break;
  }
}

/*********************************************************************************************
* ���ƣ�SERVO_UART_IRQHandler
* ���ܣ����߶�������жϷ������
* ��������
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void SERVO_UART_IRQHandler(void) {
  if (USART_GetFlagStatus(SERVO_UART, USART_FLAG_RXNE) != RESET) {
    _servo_Data_Process(USART_ReceiveData(SERVO_UART));
  }
  if (USART_GetFlagStatus(SERVO_UART, USART_FLAG_ORE) != RESET) {
    USART_ClearFlag(SERVO_UART, USART_FLAG_ORE);
    _servo_Data_Process(USART_ReceiveData(SERVO_UART));
  }
}

/*********************************************************************************************
* ���ƣ�SERVO_TIMER_IRQHandler
* ���ܣ����߶����ʱ���жϷ������
* ��������
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
/*
void SERVO_TIMER_IRQHandler(void) {
  if(TIM_GetITStatus(SERVO_TIMER, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(SERVO_TIMER, TIM_IT_Update);
    servo_Timer_Count++;
  }
}
*/

/*********************************************************************************************
* ���ƣ�servo_Init
* ���ܣ������ʼ��
* ��������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void servo_Init(void) {
  RCC_AHB1PeriphClockCmd(SERVO_CLK, ENABLE);
  SERVO_UART_CLK_INIT(SERVO_UART_CLK, ENABLE);
  //SERVO_TIMER_CLK_INIT(SERVO_TIMER_CLK, ENABLE);
  
  //���ù�������
  GPIO_PinAFConfig(SERVO_PORT, SERVO_PINSOURCE, SERVO_AF_UART);
  //IO��ʼ��
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = SERVO_PIN;
  GPIO_Init(SERVO_PORT, &GPIO_InitStructure);
  
  //���ڳ�ʼ��
  USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.USART_BaudRate = 115200;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(SERVO_UART, &USART_InitStruct);
  //��˫��ģʽʹ��
  USART_HalfDuplexCmd(SERVO_UART, ENABLE);
  //�ж�ʹ��
  USART_ITConfig(SERVO_UART, USART_IT_RXNE, ENABLE);
  //NVIC����
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = SERVO_UART_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 6;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  //����ʹ��
  USART_Cmd(SERVO_UART, ENABLE);
  
  //��ʱ����ʼ��--ÿ10ms����һ������ж�
  /*
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 8400 - 1;//Ƶ�ʣ�10kHz
  TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;    //������100ʱ���--10ms
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(SERVO_TIMER, &TIM_TimeBaseInitStruct);
  //�ж�ʹ��
  TIM_ClearFlag(SERVO_TIMER, TIM_IT_Update);
  TIM_ITConfig(SERVO_TIMER, TIM_IT_Update, ENABLE);//��ʱ������ж�
  //NVIC�ж�����
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //��ռʽ���ȼ�
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 5;         //��Ӧʽ���ȼ�
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannel = SERVO_TIMER_IRQn;
  NVIC_Init(&NVIC_InitStruct);
  //��ʱ��ʹ��
  TIM_Cmd(SERVO_TIMER, ENABLE);
  */
}
