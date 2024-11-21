#include "debug uart/debug_usart.h"
#include "Debug/debug_protocol.h"
#include <stdlib.h>

char debugdata[100];
/*********************************************************************************************
*���ƣ�debug_protocol_parse
*���ܣ��������Դ�����Ϣ����
*������pdata�����Դ�����������ݣ�len�����ݳ���
*���أ���Ϣ���������ó����ٶ�ʱ������1�����򷵻�0
*��ע�����ó����ٶȵı�׼��ʽ��{H1=30,H2=30,...,H6=30}
       H1-H6��Ӧ1���ֵ�6���֣�һ�ο�ͬʱ����1-6�����ӵ��ٶȣ��ٶȷ�ΧΪ-100~100��
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
    //ȷ�ϱ�ǩH[]
    ptag += 1;
    if(*ptag == 'H') {
      ch = *(ptag+1)-'0';
    } else {
      ptag = strchr(ptag, ',');
      continue;
    }
    //ȷ����ֵ=[]
    pval = strchr(ptag, '=');
    if(pval == NULL) break;
    speed = strtol(pval+1, &pend, 10);//��'='������ݰ�ʮ����תΪint�ͣ����׸��������ַ���ַ����pend
    //����ָ�����ݽ��в���
    if(ch>=1 && ch<=6 && pend!=pval+1) {//ȷ��'H'�������Ϊ1-6��ȷ��'='�������Ϊ��ֵ
      if(speed > 100) {
        speed = 100;
      } else if(speed < -100) {
        speed = -100;
      }
      Motor_Speed[ch-1] = speed;//�������õ��ٶȴ���Motor_Speed����
      index += sprintf(debugdata+index, "H%d=%d,", ch, speed);
    }
    ptag = strchr(ptag, ',');
  }
  debugdata[index] = '}';
  debugdata[index+1] = 0;
  DEBUG_INFO(debugdata, strlen(debugdata));
  
  return 1;
}
