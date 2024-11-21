#include "Servo/servo.h"
#include  <stdio.h>
#include  <string.h>

static uint16_t servo_angle[6];     //舵机角度（查询）
static uint8_t  servo_recv_flag;    //舵机角度接收flag
volatile uint16_t servo_Timer_Count = 0;      //舵机定时器计数标志

/*********************************************************************************************
* 名称：_servo_Uart_Send
* 功能：总线舵机串口发送
* 参数：发送数据头指针；数据长度
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
static void _servo_Uart_Send(char *pdata, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    USART_SendData(SERVO_UART, *(pdata + i));
    while(USART_GetFlagStatus(SERVO_UART, USART_FLAG_TC) == RESET); 
  } 
}

/*********************************************************************************************
* 名称：set_servoAngle
* 功能：舵机角度设置
* 参数：angle：要设置的角度数组，角度范围：[500-2500]；time：总线舵机的时间控制
* 返回：
* 修改：
* 注释：
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
        return;//超时直接返回
      }
    }
    //angle范围：-135~135；input范围：500~2500：angle = (input-500)/(2500-500)*270-135
    angle[i] = (int16_t)(servo_angle[i]-500) * 270 / (2500-500) - 135;
  }
}

/*********************************************************************************************
* 名称：_servo_angle_trim
* 功能：修剪舵机角度输入值，以防机器臂碰撞机器车其他部件
* 参数：id--舵机编号；pangle--舵机的输入角度
* 返回：
* 修改：
*********************************************************************************************/
static void _servo_angle_trim(uint8_t id, int16_t *pangle) {
  switch(id) {
    case 0://机器爪开合：角度增大时，机器爪张开；角度范围：[-54,81]
      if(*pangle > 81) *pangle =81;
      else if(*pangle < -54) *pangle = -54;
    break;
    case 1://机器爪旋转：角度增大时，机器爪顺时针旋转旋转（从机器爪的后方看）
    break;
    case 2://机器爪转动：角度增大时，机器爪向后转动
    break;
    case 3://机器臂转动：角度增大时，机器臂向后转动
    break;
    default:
      *pangle = 0;
    break;
  }
}

/*********************************************************************************************
* 名称：set_servoAngle
* 功能：舵机角度设置
* 参数：angle：要设置的角度数组；time：总线舵机的时间控制；设置标志位
* 返回：
* 修改：
* 注释：舵机控制量输入范围：[500-2500]
*********************************************************************************************/
void set_servoAngle(int16_t angle[], uint16_t time, uint8_t *pset_flag) {
  char servo_uart_cmd[100];    //舵机发送指令数组
  uint16_t index = 0;
  uint16_t input = 0;
  
  servo_uart_cmd[index++] = '{';
  for(uint8_t i = 0; i < 4; i++) {
    if(((*pset_flag>>i)&0x01) != 0) {//仅在设置标志位有效时，才发送角度设置指令
      *pset_flag &= ~(0x01<<i);//清除设置标志位
      //根据实际情况，修剪舵机角度范围
      _servo_angle_trim(i, &angle[i]);
      //angle范围：-135~135；input范围：500~2500：input = (angle+135)/270*(2500-500)+500
      input = (angle[i]+135) * (2500-500) / 270 + 500;
      if(input > 2500) { //角度裁剪
        input = 2500;
      } else if(input < 500) {
        input = 500;
      }
      /*舵机控制指令："#[ID-03]P[Deg-04]T[Time-04]!"*/
      index += sprintf(servo_uart_cmd+index, "#%03dP%04dT%04d!", i, input, time);
    }
  }
  servo_uart_cmd[index++] = '}';
  _servo_Uart_Send(servo_uart_cmd, index);
}

/*********************************************************************************************
* 名称：_servo_Data_Process
* 功能：总线舵机串口接收处理
* 参数：中断接收到的数据
* 返回：无
* 修改：
* 注释：
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
    case 1: case 2: case 3://ID，3位
      if(data>='0' && data<='9') {
        if(servo_index==3) {
          //ID不超过10，只取个位
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
    case 5: case 6: case 7: case 8://角度，4位
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
* 名称：SERVO_UART_IRQHandler
* 功能：总线舵机串口中断服务程序
* 参数：无
* 返回：无
* 修改：
* 注释：
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
* 名称：SERVO_TIMER_IRQHandler
* 功能：总线舵机定时器中断服务程序
* 参数：无
* 返回：无
* 修改：
* 注释：
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
* 名称：servo_Init
* 功能：舵机初始化
* 参数：无
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void servo_Init(void) {
  RCC_AHB1PeriphClockCmd(SERVO_CLK, ENABLE);
  SERVO_UART_CLK_INIT(SERVO_UART_CLK, ENABLE);
  //SERVO_TIMER_CLK_INIT(SERVO_TIMER_CLK, ENABLE);
  
  //复用功能配置
  GPIO_PinAFConfig(SERVO_PORT, SERVO_PINSOURCE, SERVO_AF_UART);
  //IO初始化
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = SERVO_PIN;
  GPIO_Init(SERVO_PORT, &GPIO_InitStructure);
  
  //串口初始化
  USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.USART_BaudRate = 115200;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(SERVO_UART, &USART_InitStruct);
  //半双工模式使能
  USART_HalfDuplexCmd(SERVO_UART, ENABLE);
  //中断使能
  USART_ITConfig(SERVO_UART, USART_IT_RXNE, ENABLE);
  //NVIC配置
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = SERVO_UART_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 6;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  //串口使能
  USART_Cmd(SERVO_UART, ENABLE);
  
  //定时器初始化--每10ms触发一次溢出中断
  /*
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 8400 - 1;//频率：10kHz
  TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;    //计数到100时溢出--10ms
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(SERVO_TIMER, &TIM_TimeBaseInitStruct);
  //中断使能
  TIM_ClearFlag(SERVO_TIMER, TIM_IT_Update);
  TIM_ITConfig(SERVO_TIMER, TIM_IT_Update, ENABLE);//定时器溢出中断
  //NVIC中断配置
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //抢占式优先级
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 5;         //响应式优先级
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannel = SERVO_TIMER_IRQn;
  NVIC_Init(&NVIC_InitStruct);
  //定时器使能
  TIM_Cmd(SERVO_TIMER, ENABLE);
  */
}
