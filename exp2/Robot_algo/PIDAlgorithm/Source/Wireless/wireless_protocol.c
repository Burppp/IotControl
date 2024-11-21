#include "wireless uart/wireless_usart.h"
#include "Wireless/wireless_protocol.h"
#include "CarMove/CarMove.h"
#include <stdlib.h>

char wireless_data[100];
/*********************************************************************************************
*���ƣ�wireless_protocol_parse
*���ܣ��������ߴ�����Ϣ����
*������pdata�����Դ�����������ݣ�len�����ݳ���
*���أ���Ϣ���������ó���״̬ʱ������1�����򷵻�0
*��ע�����ó���״̬�ı�׼��ʽ��{V4=x,V5=y}
       V4������0-4�ֱ��Ӧֹͣ/ǰ��/����/��ת/��ת��V5���ٶȣ��ٶȷ�Χ0-100
*********************************************************************************************/
uint8_t wireless_protocol_parse(char *pdata, uint16_t len) {
  char *ptag = NULL, *pval = NULL, *pend = NULL;
  int8_t ch, value;
  if(pdata == NULL || len == 0) {
    return 0;
  }
  
  ptag = strchr(pdata, '{');
  while(ptag != NULL) {
    //ȷ�ϱ�ǩV[]
    ptag += 1;
    if(*ptag == 'V') {
      ch = *(ptag+1)-'0';
    } else {
      ptag = strchr(ptag, ',');
      continue;
    }
    //ȷ����ֵ=[]
    pval = strchr(ptag, '=');
    if(pval == NULL) break;
    value = strtol(pval+1, &pend, 10);//��'='������ݰ�ʮ����תΪint�ͣ����׸��������ַ���ַ����pend
    if(pend == pval+1) break;         //ȷ��'='�������Ϊ��ֵ
    //����ָ�����ݽ��в���
    if(ch == 4) {//���÷���
      if(value > 4 || value < 0) {
        value = 0;
      }
      Car_Direction = value;
    } else if(ch==5) {//�����ٶ�
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
