/****************************************Copyright (c)**************************************************
**
**--------------�ļ���Ϣ--------------------------------------------------------------------------------
**��   ��   ��: direction_pid.c
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
#include "PID/direction_pid.h"
#include "MotorSpeed/motor_speed.h"
#include "stm32f4xx.h"
#include <string.h>
#include <math.h>

float car_dir_init;//С������ʱ�ĳ�ʼ���ԽǶȡ������ĽǶȾ�Ϊ��˽Ƕȵ���ԽǶȡ�
/*�Ƕ�PID����*/
static float dir_pid_param_P = 1.0f; //PID��P����
static float dir_pid_param_I = 0.0f; //PID��I����
static float dir_pid_param_D = 0.0f; //PID��D����

/*
ͨ��λ��ʽPID�㷨����Ŀ��Ƕȡ���ǰ�Ƕȼ����Ҫ�ı�ĽǶ�
@param dir_obs: С����ǰ�Ƕ�(in)
@param dir_target: С��Ŀ��Ƕ�(in)
@return: Ҫ�ı�ĽǶ�
*/
static float dir_PID_Control(float dir_obs, int dir_target)
{
  /*PID�㷨��ر���*/
  static int s_last_target;//ǰ��Ŀ��ֵ
  static float s_dir_err_now = 0.f;//��ǰƫ����
  static float s_dir_err_int = 0.f;//ƫ����������
  float dir_err_last, dir_err_diff;//ǰ��ƫ������ƫ���������
  
  float dir_change;//Ҫ�ı�ĽǶ�--����ֵ
  
  //��ȡƫ����
  dir_err_last = s_dir_err_now;
  s_dir_err_now = dir_target - dir_obs;
  //ȡ���·��
  if(s_dir_err_now > 180) s_dir_err_now -= 360;
  else if(s_dir_err_now <= -180) s_dir_err_now += 360;
  
  //Ŀ��ֵ�仯ʱ����������΢��������
  if(s_last_target != dir_target) {
    s_dir_err_int = 0;
    dir_err_last = s_dir_err_now;
    s_last_target = dir_target;
  }
  
  //��������P--��Ӧ�ٶ�
  dir_change = dir_pid_param_P * s_dir_err_now;

  //���ֿ���I--��̬���
  s_dir_err_int += s_dir_err_now;
  if(fabs(s_dir_err_now) > 45.f) {//���ַ���PID�㷨
    s_dir_err_int = 0;
  }
  dir_change += dir_pid_param_I * s_dir_err_int;
  
  //΢�ֿ���D--������
  dir_err_diff = s_dir_err_now - dir_err_last;
  dir_change += dir_pid_param_D * dir_err_diff;
  
  return dir_change;
}

/*********************************************************************************************
*���ƣ�dir_Cal_Speed_Target
*���ܣ�ͨ������ģ�ͣ��ɴ��ı�ǶȺ�С��Ŀ���ٶȼ������Ŀ���ٶ�
*���������ı�Ƕȣ�С��Ŀ���ٶȣ�������Ŀ���ٶ�
*���أ���
*�޸ģ�
*********************************************************************************************/
static void dir_Conver_Speed(float dir_change, int8_t car_speed, int8_t wheel_speed[]) {
  int speed_diff;//�������ٶȲ�ࣨ������ٶ�-�Ҳ����ٶȣ�
  int speed_left, speed_right;//���������趨�ٶ�
  
  //��Ŀ���ٶȡ��������Ƕȼ������Ҳ���Ŀ���ٶ�
  speed_diff  = (int)(CAR_WIDTH * dir_change * PI / 180.f / DIR_TO_SPEED_INTERVAL)/10;//����ģ��--cm/s
  speed_left  = car_speed + speed_diff/2;
  speed_right = car_speed - speed_diff/2;
  //���ٶ�Ŀ��������[-100,100]��Χ��
  speed_left = speed_left>100 ? 100 : (speed_left<-100 ? -100 : speed_left);
  speed_right = speed_right>100 ? 100 : (speed_right<-100 ? -100 : speed_right);
  
  //�����Ҳ���Ŀ���ٶȸ�ֵ���������
  wheel_speed[SPEED_H1A_INDEX] = speed_left;
  wheel_speed[SPEED_H2A_INDEX] = speed_left;
  wheel_speed[SPEED_H3A_INDEX] = speed_left;
  wheel_speed[SPEED_H4A_INDEX] = speed_right;
  wheel_speed[SPEED_H5A_INDEX] = speed_right;
  wheel_speed[SPEED_H6A_INDEX] = speed_right;
}

/*********************************************************************************************
*���ƣ�dir_Cal_Speed_Target
*���ܣ�ͨ��С��Ŀ���ٶȡ�С����ǰ���򣬼���С������Ŀ���ٶ�
*������С��Ŀ���ٶȣ���Ÿ���Ŀ���ٶȵ�����
*���أ���
*�޸ģ�
*********************************************************************************************/
void dir_Cal_Speed_Target(int8_t carspeed_target, int8_t wheelspeed_target[]) {
  extern float imu2_yaw;
  if(carspeed_target == 0) {//���ջ���
    memset(wheelspeed_target, 0, 6);
    return;
  }
  
  //��ȡ��ǰ�Ƕ�
  float dir_now = imu2_yaw - car_dir_init;
  if(dir_now > 180.f) {
    dir_now -= 360.f;
  } else if(dir_now <= -180.f) {
    dir_now += 360.f;
  }
  //��ȡҪ�ı�ĽǶ�
  float dir_change = dir_PID_Control(dir_now, 0);
  //��ȡ����Ŀ���ٶ�
  dir_Conver_Speed(dir_change, carspeed_target, wheelspeed_target);
}
