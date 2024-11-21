/*********************************************************************************************
* �ļ���usart.c
* ���ߣ�zonesion 2017.02.17
* ˵����������������
* �޸ģ�Chenkm 2017.02.17 ���ע��
* ע�ͣ�
*
*********************************************************************************************/
/*********************************************************************************************
* ͷ�ļ�
*********************************************************************************************/
#include <string.h>
#include <stdio.h>
#include "wireless uart/wireless_usart.h"

/*********************************************************************************************
* ȫ�ֱ���
*********************************************************************************************/
volatile unsigned char Wireless_Usart_len=0;               //���ջ�������ǰ���ݳ���
char WIRELESS_USART_RX_BUF[WIRELESS_USART_REC_MAX];        //���ջ���,���USART_REC_LEN���ֽ�.

/*********************************************************************************************
* ���ƣ�_wireless_DMA_Init
* ���ܣ�usart2-DMA��ʼ��
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
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
* ���ƣ�wireless_usart_init
* ���ܣ�usart2��ʼ��
* ������bound������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void wireless_usart_init(unsigned int bound){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);     			//ʹ��GPIODʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);    			//ʹ��USART2ʱ�� 
  //����2��Ӧ���Ÿ���ӳ��
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);  			//GPIOD5����ΪUSART2
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); 			//GPIOD6����ΪUSART2	
  //USART2�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;  			//GPIOD5��GPIOD6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             			//���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   					//�ٶ�50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           			//���츴�����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             			//����
  GPIO_Init(GPIOD,&GPIO_InitStructure);                    			//��ʼ��PD5��PD6
  //USART2 ��ʼ������
  USART_InitStructure.USART_BaudRate = bound;                   //����������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;        //һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;           //����żУ��λ
  //��Ӳ������������
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  //�շ�ģʽ
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);                     //�����������ó�ʼ������2	
  //Usart2 NVIC ����
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);               //���ȼ�����Ϊ1����ռʽ���ȼ�����Ϊ0-1����Ӧʽ���ȼ�����Ϊ0-7
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;             //����2�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;       //��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =6;	        		//�����ȼ�6
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	                        			//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  USART_ClearFlag(USART2, USART_FLAG_TC | USART_FLAG_RXNE);     //������б�־λ
  USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);                //��������2�����ж�
  _wireless_DMA_Init();                                         //ʹ�ܴ���2DMA
  USART_Cmd(USART2, ENABLE);                                    //ʹ�ܴ���2 	
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);                //ʹ�ܴ���2���յ�DMA������ͨ��
}

/*********************************************************************************************
* ���ƣ�USART2_IRQHandler
* ���ܣ������жϴ�����
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void USART2_IRQHandler(void)                	           
{
  if(USART_GetITStatus(USART2, USART_IT_IDLE) == SET) {
    USART_ReceiveData(USART2);
    
    Wireless_Usart_len = WIRELESS_USART_REC_MAX - DMA_GetCurrDataCounter(DMA1_Stream5);
    //����DMA
    DMA_Cmd(DMA1_Stream5, DISABLE);
    WIRELESS_USART_RX_BUF[Wireless_Usart_len] = 0;
    _wireless_DMA_Init();
  }
}

/*********************************************************************************************
* ���ƣ�wireless_usart_send
* ���ܣ�����2��������
* ������s�����͵�����ָ�룬len�����͵����ݳ���
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/ 
void wireless_usart_send(char *s,unsigned char len){
  for(unsigned char i = 0;i < len;i++)
  {
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2, *(s+i));
  }
}
