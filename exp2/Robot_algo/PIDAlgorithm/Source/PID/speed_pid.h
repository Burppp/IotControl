/*
***************************************Copyright (c)***************************
**                               	  �人������Ϣ�������޹�˾
**                                     		������ҵ��
**                                        
**
**                                		 
**
**--------------�ļ���Ϣ-------------------------------------------------------
**��   ��   ��: dmp_pid.h
**��   ��   ��: ���ĳ�
**����޸�����: 2018��10��26��
**��        ��: PID����ͷ�ļ�
**              
**--------------��ʷ�汾��Ϣ---------------------------------------------------
** ������: ���ĳ�
** ��  ��: v1.0
** �ա���: 2018��10��26��
** �衡��: ��ʼ����
**
**--------------��ǰ�汾�޶�---------------------------------------------------
** �޸���: 
** �ա���: 
** �衡��: 
**
**-----------------------------------------------------------------------------
******************************************************************************/
#ifndef _SPEED_PID_H_
#define _SPEED_PID_H_

#include "stm32f4xx.h"

//����ʱ�ĵ��������������������
#define CAR_START_MOTOR_LIMIT_ADD   10
#define CAR_START_MOTOR_LIMIT_TIMES 6

extern int8_t motor_ctrl[6];
extern uint8_t xcar_speed_start_flag[6];

void XCAR_PID_Control(signed char target);
void speed_Get_PID(unsigned char param, unsigned *pdata);
void speed_Set_PID(unsigned char param, unsigned data);
void xcar_Get_CST(unsigned *pdata);
void xcar_Set_CST(unsigned char ch, unsigned data);

#endif  //_SPEED_PID_H_
