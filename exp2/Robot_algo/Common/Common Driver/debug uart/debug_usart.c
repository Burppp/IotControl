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
#include "debug uart/debug_usart.h"

/*********************************************************************************************
* 全局变量
*********************************************************************************************/
unsigned char Debug_Usart_len=0;                        //接收缓冲区当前数据长度
char DEBUG_USART_RX_BUF[DEBUG_USART_REC_MAX];           //接收缓冲,最大USART_REC_LEN个字节.

/*********************************************************************************************
* 名称：fputc
* 功能：将usart1映射到printf函数
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
int fputc(int ch, FILE *f)
{ 	
  while((USART1->SR&0X40)==0);																	//循环发送,直到发送完毕   
  USART1->DR = (unsigned char) ch;      
  return ch;
}

/*********************************************************************************************
* 名称：_debug_DMA_Init
* 功能：usart1-DMA初始化
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
static void _debug_DMA_Init(void) {
  DMA_InitTypeDef DMA_InitStruct;    
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  DMA_DeInit(DMA2_Stream5);
  DMA_InitStruct.DMA_Channel = DMA_Channel_4;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(&DEBUG_USART_RX_BUF[0]);
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_BufferSize = DEBUG_USART_REC_MAX;
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
  DMA_Init(DMA2_Stream5, &DMA_InitStruct);
  
  DMA_Cmd(DMA2_Stream5, ENABLE);
}

/*********************************************************************************************
* 名称：usart_init
* 功能：usart1初始化
* 参数：bound波特率
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void debug_usart_init(unsigned int bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);     			//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);    			//使能USART1时钟 
  //串口1对应引脚复用映射
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);  			//GPIOA9复用为USART1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 			//GPIOA10复用为USART1	
  //USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;  			//GPIOA9与GPIOA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             			//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   					//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           			//推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             			//上拉
  GPIO_Init(GPIOA,&GPIO_InitStructure);                    			//初始化PA9，PA10
  //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = bound;                   //波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;        //一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;           //无奇偶校验位
  //无硬件数据流控制
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  //收发模式
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);                     //根据上述配置初始化串口1	
  //Usart1 NVIC 配置
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);               //优先级组设为1，抢占式优先级可设为0-1，响应式优先级可设为0-7
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;             //串口1中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;       //抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =7;	        		//子优先级7
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	                        			//根据指定的参数初始化VIC寄存器、
  USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);                //开启串口1接收中断
  _debug_DMA_Init();                                            //使能串口1DMA
  USART_Cmd(USART1, ENABLE);                                    //使能串口1 	
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);                //使能串口1接收到DMA的请求通道
}

/*********************************************************************************************
* 名称：USART1_IRQHandler
* 功能：串口中断处理函数
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void USART1_IRQHandler(void)                	           
{
  if(USART_GetITStatus(USART1, USART_IT_IDLE) == SET) {
    USART_ReceiveData(USART1);
    
    Debug_Usart_len = DEBUG_USART_REC_MAX - DMA_GetCurrDataCounter(DMA2_Stream5);
    //重置DMA
    DMA_Cmd(DMA2_Stream5, DISABLE);
    DEBUG_USART_RX_BUF[Debug_Usart_len] = 0;
    _debug_DMA_Init();
  }
}

/*********************************************************************************************
* 名称：usart_send
* 功能：串口1发送数据
* 参数：s待发送的数据指针，len待发送的数据长度
* 返回：
* 修改：
* 注释：
*********************************************************************************************/ 
void debug_usart_send(char *s,unsigned char len){
  for(unsigned char i = 0;i < len;i++)
  {
    USART_SendData(USART1, *(s+i));
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  }
}
