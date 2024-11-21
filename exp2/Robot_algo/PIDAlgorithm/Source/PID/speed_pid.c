/****************************************Copyright (c)**************************************************
**
**--------------�ļ���Ϣ--------------------------------------------------------------------------------
**��   ��   ��: speed_pid.c
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
** �衡��: ������PID-�ٶȿ���
**----------------------------------------------------------------------------------------------------

********************************************************************************************************/
#include "stm32f4xx.h"
#include "PID/speed_pid.h"
#include "PID/direction_pid.h"
#include "MotorSpeed/motor_speed.h"
#include "Motor/motor.h"
//#include "CarMove.h"

#define ABS(x) x>=0?x:(-x)
float g_Car_Speed_Status = 0.f;//С���ٶ�״̬

uint8_t xcar_speed_start_flag[6];//��������־�飺����ʱȫ����Ϊ6��ÿ�ν���PID�㷨ʱ-1��>0ʱ�ٶ����������Ȳ��ó����޶�
int8_t motor_ctrl[6];  //���������
/*�ٶ�PID����*/
static float speed_pid_param_P = 2.f;//PID��P����
static float speed_pid_param_I = 1.f; //PID��I����
static float speed_pid_param_D = 0.f;  //PID��D����
static float pid_param_C = 0.35f; //������-״̬��ת����
static int   speed_target_vary_limit = 5;  //Ŀ��仯��������޶�ֵʱ������PID��ر���

/*
ͨ������ʽPID�㷨����Ŀ���ٶȡ���ǰ�ٶȡ���ǰ�����������Ŀ�������
@param obs: ���ֵ�ǰ�ٶ�(in)
@param target: ����Ŀ���ٶ�(in)
@param motor: ���������(in-��ǰ������; out-Ŀ�������)
*/
static void speed_PID_Control(float obs[], int8_t target[], int8_t motor[])
{
  /*PID�㷨��ر���*/
  static int8_t s_last_target[6];//�ϴ�Ŀ��ֵ
  static float s_speed_err_last[6];//�ϴ�ƫ����(err[k-1])
  static float s_speed_err_now[6]; //����ƫ����(err[k])
  float speed_err_befor_last, speed_err_diff;//���ϴ�ƫ����(err[k-2])��ƫ���������
  
  float motor_add[6];     //�������������--result
  uint8_t i = 0;
  
  for(i = 0; i < 6; i++) {
    if(target[i] == 0) {
      motor[i] = 0;
      continue;
    }
    
    //��ȡƫ����
    speed_err_befor_last = s_speed_err_last[i];
    s_speed_err_last[i]  = s_speed_err_now[i];
    s_speed_err_now[i]   = target[i] - obs[i];
    
    //Ŀ��ֵ�仯����ʱ����������΢��������
    if(ABS(s_last_target[i]-target[i]) > speed_target_vary_limit) {
      s_speed_err_last[i]  = s_speed_err_now[i];
      speed_err_befor_last = s_speed_err_last[i];
      s_last_target[i] = target[i];
    }
    
    //״̬��ƫ��*������/״̬��ת���ȣ�Ϊ����������
    //��������P--������
    motor_add[i] = speed_pid_param_P * (pid_param_C * (s_speed_err_now[i] - s_speed_err_last[i]));

    //���ֿ���I--��Ӧ�ٶ�
    motor_add[i] += speed_pid_param_I * (pid_param_C * s_speed_err_now[i]);
    
    //΢�ֿ���D--��̬���
    speed_err_diff = s_speed_err_now[i] - 2*s_speed_err_last[i] + speed_err_befor_last;
    motor_add[i] += speed_pid_param_D * (pid_param_C * speed_err_diff);
    
    //������
    if(xcar_speed_start_flag[i] > 0) {
      xcar_speed_start_flag[i]--;
      if(motor_add[i] > CAR_START_MOTOR_LIMIT_ADD) {
        motor_add[i] = CAR_START_MOTOR_LIMIT_ADD;
      } else if (motor_add[i] < -CAR_START_MOTOR_LIMIT_ADD) {
        motor_add[i] = -CAR_START_MOTOR_LIMIT_ADD;
      }
    }
    
    //������������
    motor[i] += (int)motor_add[i];
    if(target[i] > 0) {
      if(motor[i] < 20) motor[i]=20;
      else if(motor[i]>60) motor[i]=60;
    } else {
      if(motor[i] > -20) motor[i]=-20;
      else if(motor[i]<-60) motor[i]=-60;
    }
  }
}

/*********************************************************************************************
*���ƣ�XCAR_PID_Control
*���ܣ�ͨ��PID�㷨������С�����ٶ����˶�
*������С��Ŀ���ٶ�
*���أ���
*�޸ģ�
*********************************************************************************************/
void XCAR_PID_Control(int8_t target) {
  static int last_counter = 0;
  int8_t wheel_speed_target[6];  // �����ٶ�Ŀ�����顾cm/s��
  float wheel_speed_obs[6];      // ���ֹ۲��ٶȡ�cm/s��
  uint8_t i = 0;
  
  //ÿ100msִ��һ��PID����
  if(last_counter == Time_Update_Count_100MS) return;
  last_counter = Time_Update_Count_100MS;
  //��ȡ���µĸ����ٶ�Ŀ��--��λ��cm/s
  dir_Cal_Speed_Target(target, wheel_speed_target);
  //��ȡ���µĸ��ֵ�ǰ�ٶ�--��λ��cm/s
  for(i = 0; i < 6; i++) {//Ŀ��Ϊ��ֵʱ����Ϊ����ٶ�Ҳ�Ǹ�ֵ
    if(wheel_speed_target[i] >= 0) {
      wheel_speed_obs[i] = ms_Get_Speed(i) / 10.f;
    } else {
      wheel_speed_obs[i] = -ms_Get_Speed(i) / 10.f;
    }
  }
  //PID����ó����µĿ�����
  speed_PID_Control(wheel_speed_obs, wheel_speed_target, motor_ctrl);
  
  //���Ƶ��ת��
  //for(i = 0; i < 6; i++) {
  //  set_motorSpeed(1<<i, motor_ctrl[i]);
  //}
  
  if(target > 0)
  {
    set_motorSpeed(MOTOR_LEFT, 30);//������ٶ���Ϊ-Car_Speed
    set_motorSpeed(MOTOR_RIGHT, 30); //�Ҳ����ٶ���ΪCar_Speed
  }
  if(target == 0)
  {
  set_motorSpeed(MOTOR_LEFT, 0); //������ٶ���Ϊ-Car_Speed
    set_motorSpeed(MOTOR_RIGHT, 0); //�Ҳ����ٶ���ΪCar_Speed
  }
  if(target < 0)
  {
    set_motorSpeed(MOTOR_LEFT, -30); //������ٶ���Ϊ-Car_Speed
      set_motorSpeed(MOTOR_RIGHT,-30); //�Ҳ����ٶ���ΪCar_Speed
  }
}
