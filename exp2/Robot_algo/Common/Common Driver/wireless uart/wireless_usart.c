/*********************************************************************************************
* 文件：usart.c
* 作者：zonesion 2017.02.17
* 说明：串口驱动程序
* 修改：Chenkm 2017.02.17 添加注释
* 注释：
*
*********************************************************************************************/
/*********************************************************************************************
* 头文件
*********************************************************************************************/
#include <string.h>
#include <stdio.h>
#include "wireless uart/wireless_usart.h"

/*********************************************************************************************
* 全局变量
*********************************************************************************************/
volatile unsigned char Wireless_Usart_len=0;               //接收缓冲区当前数据长度
char WIRELESS_USART_RX_BUF[WIRELESS_USART_REC_MAX];        //接收缓冲,最大USART_REC_LEN个字节.

/*********************************************************************************************
* 名称：_wireless_DMA_Init
* 功能：usart2-DMA初始化
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
static void _wireless_DMA_Init(void) {
  DMA_InitTypeDef DMA_InitStruct;    
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Stream5);
  DMA_InitStruct.DMA_Channel = DMA_Channel_4;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(&WIRELESS_USART_RX_BUF[0]);
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_BufferSize = WIRELESS_USART_REC_MAX;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStruct);
  
  DMA_Cmd(DMA1_Stream5, ENABLE);
}

/*********************************************************************************************
* 名称：wireless_usart_init
* 功能：usart2初始化
* 参数：bound波特率
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void wireless_usart_init(unsigned int bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);     			//使能GPIOD时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);    			//使能USART2时钟 
  //串口2对应引脚复用映射
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);  			//GPIOD5复用为USART2
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); 			//GPIOD6复用为USART2	
  //USART2端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;  			//GPIOD5与GPIOD6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             			//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   					//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           			//推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             			//上拉
  GPIO_Init(GPIOD,&GPIO_InitStructure);                    			//初始化PD5，PD6
  //USART2 初始化设置
  USART_InitStructure.USART_BaudRate = bound;                   //波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;        //一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;           //无奇偶校验位
  //无硬件数据流控制
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  //收发模式
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);                     //根据上述配置初始化串口2	
  //Usart2 NVIC 配置
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);               //优先级组设为1，抢占式优先级可设为0-1，响应式优先级可设为0-7
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;             //串口2中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;       //抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =6;	        		//子优先级6
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	                        			//根据指定的参数初始化VIC寄存器
  USART_ClearFlag(USART2, USART_FLAG_TC | USART_FLAG_RXNE);     //清除已有标志位
  USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);                //开启串口2接收中断
  _wireless_DMA_Init();                                         //使能串口2DMA
  USART_Cmd(USART2, ENABLE);                                    //使能串口2 	
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);                //使能串口2接收到DMA的请求通道
}

/*********************************************************************************************
* 名称：USART2_IRQHandler
* 功能：串口中断处理函数
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void USART2_IRQHandler(void)                	           
{
  if(USART_GetITStatus(USART2, USART_IT_IDLE) == SET) {
    USART_ReceiveData(USART2);
    
    Wireless_Usart_len = WIRELESS_USART_REC_MAX - DMA_GetCurrDataCounter(DMA1_Stream5);
    //重置DMA
    DMA_Cmd(DMA1_Stream5, DISABLE);
    WIRELESS_USART_RX_BUF[Wireless_Usart_len] = 0;
    _wireless_DMA_Init();
  }
}

/*********************************************************************************************
* 名称：wireless_usart_send
* 功能：串口2发送数据
* 参数：s待发送的数据指针，len待发送的数据长度
* 返回：
* 修改：
* 注释：
*********************************************************************************************/ 
void wireless_usart_send(char *s,unsigned char len){
  for(unsigned char i = 0;i < len;i++)
  {
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2, *(s+i));
  }
}
