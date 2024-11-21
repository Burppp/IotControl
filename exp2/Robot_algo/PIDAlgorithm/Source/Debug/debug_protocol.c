#include "debug uart/debug_usart.h"
#include "Debug/debug_protocol.h"
#include <stdlib.h>

char debugdata[100];
/*********************************************************************************************
*名称：debug_protocol_parse
*功能：解析调试串口消息内容
*参数：pdata：调试串口输入的数据；len：数据长度
*返回：消息内容是设置车轮速度时，返回1，否则返回0
*备注：设置车轮速度的标准格式：{H1=30,H2=30,...,H6=30}
       H1-H6对应1号轮到6号轮，一次可同时设置1-6个轮子的速度；速度范围为-100~100。
*********************************************************************************************/
uint8_t debug_protocol_parse(char *pdata, uint16_t len) {
  extern int8_t Motor_Speed[6];
  char *ptag = NULL, *pval = NULL, *pend = NULL;
  int8_t ch, speed, index;
  if(pdata == NULL || len == 0 || *pdata != '{' || *(pdata+len-1) != '}') {
    return 0;
  }
  
  index = sprintf(debugdata, "Set Speed: {");
  ptag = (char*)(pdata);
  while(ptag != NULL) {
    //确认标签H[]
    ptag += 1;
    if(*ptag == 'H') {
      ch = *(ptag+1)-'0';
    } else {
      ptag = strchr(ptag, ',');
      continue;
    }
    //确认数值=[]
    pval = strchr(ptag, '=');
    if(pval == NULL) break;
    speed = strtol(pval+1, &pend, 10);//将'='后的内容按十进制转为int型；将首个非数字字符地址存入pend
    //根据指令内容进行操作
    if(ch>=1 && ch<=6 && pend!=pval+1) {//确保'H'后的内容为1-6；确保'='后的内容为数值
      if(speed > 100) {
        speed = 100;
      } else if(speed < -100) {
        speed = -100;
      }
      Motor_Speed[ch-1] = speed;//将新设置的速度存入Motor_Speed数组
      index += sprintf(debugdata+index, "H%d=%d,", ch, speed);
    }
    ptag = strchr(ptag, ',');
  }
  debugdata[index] = '}';
  debugdata[index+1] = 0;
  DEBUG_INFO(debugdata, strlen(debugdata));
  
  return 1;
}
