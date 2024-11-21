/****************************************Copyright (c)**************************************************
**
**--------------文件信息--------------------------------------------------------------------------------
**文   件   名: speed_pid.c
**创   建   人: 何文超
**最后修改日期: 2018年10月26日
**描        述: PID 控制文件
**              
**--------------历史版本信息----------------------------------------------------------------------------
** 创建人: 何文超
** 版  本: v1.0
** 日　期: 2018年10月26日
** 描　述: 第六版本
**
**--------------当前版本修订------------------------------------------------------------------------------
** 修改人: 杨开琦
** 日　期: 2018年11月5日
** 描　述: 增量型PID-速度控制
**----------------------------------------------------------------------------------------------------

********************************************************************************************************/
#include "stm32f4xx.h"
#include "PID/speed_pid.h"
#include "PID/direction_pid.h"
#include "MotorSpeed/motor_speed.h"
#include "Motor/motor.h"
//#include "CarMove.h"

#define ABS(x) x>=0?x:(-x)
float g_Car_Speed_Status = 0.f;//小车速度状态

uint8_t xcar_speed_start_flag[6];//缓启动标志组：启动时全部置为6；每次进入PID算法时-1；>0时速度量增量幅度不得超过限度
int8_t motor_ctrl[6];  //电机控制量
/*速度PID参数*/
static float speed_pid_param_P = 2.f;//PID的P参数
static float speed_pid_param_I = 1.f; //PID的I参数
static float speed_pid_param_D = 0.f;  //PID的D参数
static float pid_param_C = 0.35f; //控制量-状态量转化比
static int   speed_target_vary_limit = 5;  //目标变化超过这个限定值时，重置PID相关变量

/*
通过增量式PID算法，由目标速度、当前速度、当前控制量计算出目标控制量
@param obs: 各轮当前速度(in)
@param target: 各轮目标速度(in)
@param motor: 电机控制量(in-当前控制量; out-目标控制量)
*/
static void speed_PID_Control(float obs[], int8_t target[], int8_t motor[])
{
  /*PID算法相关变量*/
  static int8_t s_last_target[6];//上次目标值
  static float s_speed_err_last[6];//上次偏差量(err[k-1])
  static float s_speed_err_now[6]; //本次偏差量(err[k])
  float speed_err_befor_last, speed_err_diff;//上上次偏差量(err[k-2])，偏差量差分量
  
  float motor_add[6];     //电机控制量增量--result
  uint8_t i = 0;
  
  for(i = 0; i < 6; i++) {
    if(target[i] == 0) {
      motor[i] = 0;
      continue;
    }
    
    //获取偏差量
    speed_err_befor_last = s_speed_err_last[i];
    s_speed_err_last[i]  = s_speed_err_now[i];
    s_speed_err_now[i]   = target[i] - obs[i];
    
    //目标值变化过大时，积分量、微分量清零
    if(ABS(s_last_target[i]-target[i]) > speed_target_vary_limit) {
      s_speed_err_last[i]  = s_speed_err_now[i];
      speed_err_befor_last = s_speed_err_last[i];
      s_last_target[i] = target[i];
    }
    
    //状态量偏差*控制量/状态量转化比，为控制量增量
    //比例控制P--超调量
    motor_add[i] = speed_pid_param_P * (pid_param_C * (s_speed_err_now[i] - s_speed_err_last[i]));

    //积分控制I--响应速度
    motor_add[i] += speed_pid_param_I * (pid_param_C * s_speed_err_now[i]);
    
    //微分控制D--稳态误差
    speed_err_diff = s_speed_err_now[i] - 2*s_speed_err_last[i] + speed_err_befor_last;
    motor_add[i] += speed_pid_param_D * (pid_param_C * speed_err_diff);
    
    //缓启动
    if(xcar_speed_start_flag[i] > 0) {
      xcar_speed_start_flag[i]--;
      if(motor_add[i] > CAR_START_MOTOR_LIMIT_ADD) {
        motor_add[i] = CAR_START_MOTOR_LIMIT_ADD;
      } else if (motor_add[i] < -CAR_START_MOTOR_LIMIT_ADD) {
        motor_add[i] = -CAR_START_MOTOR_LIMIT_ADD;
      }
    }
    
    //输出电机控制量
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
*名称：XCAR_PID_Control
*功能：通过PID算法，控制小车定速定向运动
*参数：小车目标速度
*返回：无
*修改：
*********************************************************************************************/
void XCAR_PID_Control(int8_t target) {
  static int last_counter = 0;
  int8_t wheel_speed_target[6];  // 各轮速度目标数组【cm/s】
  float wheel_speed_obs[6];      // 各轮观测速度【cm/s】
  uint8_t i = 0;
  
  //每100ms执行一次PID程序
  if(last_counter == Time_Update_Count_100MS) return;
  last_counter = Time_Update_Count_100MS;
  //获取最新的各轮速度目标--单位：cm/s
  dir_Cal_Speed_Target(target, wheel_speed_target);
  //获取最新的各轮当前速度--单位：cm/s
  for(i = 0; i < 6; i++) {//目标为负值时，认为测得速度也是负值
    if(wheel_speed_target[i] >= 0) {
      wheel_speed_obs[i] = ms_Get_Speed(i) / 10.f;
    } else {
      wheel_speed_obs[i] = -ms_Get_Speed(i) / 10.f;
    }
  }
  //PID运算得出最新的控制量
  speed_PID_Control(wheel_speed_obs, wheel_speed_target, motor_ctrl);
  
  //控制电机转动
  //for(i = 0; i < 6; i++) {
  //  set_motorSpeed(1<<i, motor_ctrl[i]);
  //}
  
  if(target > 0)
  {
    set_motorSpeed(MOTOR_LEFT, 30);//左侧电机速度设为-Car_Speed
    set_motorSpeed(MOTOR_RIGHT, 30); //右侧电机速度设为Car_Speed
  }
  if(target == 0)
  {
  set_motorSpeed(MOTOR_LEFT, 0); //左侧电机速度设为-Car_Speed
    set_motorSpeed(MOTOR_RIGHT, 0); //右侧电机速度设为Car_Speed
  }
  if(target < 0)
  {
    set_motorSpeed(MOTOR_LEFT, -30); //左侧电机速度设为-Car_Speed
      set_motorSpeed(MOTOR_RIGHT,-30); //右侧电机速度设为Car_Speed
  }
}
