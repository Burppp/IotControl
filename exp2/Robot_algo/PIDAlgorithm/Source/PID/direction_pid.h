/****************************************Copyright (c)**************************************************
**
**--------------�ļ���Ϣ--------------------------------------------------------------------------------
**��   ��   ��: direction_pid.h
**��   ��   ��: ���ĳ�
**����޸�����: 2018��10��26��
**��        ��: PID �����ļ�
**              
**--------------��ʷ�汾��Ϣ----------------------------------------------------------------------------
** ������: ���ĳ�
** ��  ��: v1.0
** �ա���: 2018��10��26��
** �衡��: �����汾
**
**--------------��ǰ�汾�޶�------------------------------------------------------------------------------
** �޸���: ���
** �ա���: 2018��11��5��
** �衡��: λ����PID-�������
**----------------------------------------------------------------------------------------------------

********************************************************************************************************/
#ifndef _DIRECTION_PID_H_
#define _DIRECTION_PID_H_

//���ε���dir_Cal_Speed_Target�������������Ŀ���ٶȣ��ļ��(s)
#define DIR_TO_SPEED_INTERVAL (0.1f)

#define FABS(x) (x>0.0f ? x : -x)

void dir_Cal_Speed_Target(signed char carspeed_target, signed char wheelspeed_target[]);
void car_Set_Target(int speed_target, int dir_target);
void car_Adjust_Target(unsigned char type, int data);
int car_Get_Target(unsigned char type);
void dir_Get_PID(unsigned char param, unsigned *pdata);
void dir_Set_PID(unsigned char param, unsigned data);

#endif  //_DIRECTION_PID_H_
