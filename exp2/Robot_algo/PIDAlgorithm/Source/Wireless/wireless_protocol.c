#include "wireless uart/wireless_usart.h"
#include "Wireless/wireless_protocol.h"
#include "CarMove/CarMove.h"
#include <stdlib.h>

char wireless_data[100];
/*********************************************************************************************
*名称：wireless_protocol_parse
*功能：解析无线串口消息内容
*参数：pdata：调试串口输入的数据；len：数据长度
*返回：消息内容是设置车辆状态时，返回1，否则返回0
*备注：设置车辆状态的标准格式：{V4=x,V5=y}
       V4：方向，0-4分别对应停止/前进/后退/左转/右转；V5：速度，速度范围0-100
*********************************************************************************************/
uint8_t wireless_protocol_parse(char *pdata, uint16_t len) {
  char *ptag = NULL, *pval = NULL, *pend = NULL;
  int8_t ch, value;
  if(pdata == NULL || len == 0) {
    return 0;
  }
  
  ptag = strchr(pdata, '{');
  while(ptag != NULL) {
    //确认标签V[]
    ptag += 1;
    if(*ptag == 'V') {
      ch = *(ptag+1)-'0';
    } else {
      ptag = strchr(ptag, ',');
      continue;
    }
    //确认数值=[]
    pval = strchr(ptag, '=');
    if(pval == NULL) break;
    value = strtol(pval+1, &pend, 10);//将'='后的内容按十进制转为int型；将首个非数字字符地址存入pend
    if(pend == pval+1) break;         //确保'='后的内容为数值
    //根据指令内容进行操作
    if(ch == 4) {//设置方向
      if(value > 4 || value < 0) {
        value = 0;
      }
      Car_Direction = value;
    } else if(ch==5) {//设置速度
      if(value > 100) {
        value = 100;
      } else if(value < 0) {
        value = 0;
      }
      Car_Speed = value;
    }
    ptag = strchr(ptag, ',');
  }
  return 1;
}
