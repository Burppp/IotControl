/****************************************Copyright (c)**************************************************
**
**--------------文件信息--------------------------------------------------------------------------------
**文   件   名: direction_pid.c
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
** 描　述: 位置型PID-方向控制
**----------------------------------------------------------------------------------------------------

********************************************************************************************************/
#include "PID/direction_pid.h"
#include "MotorSpeed/motor_speed.h"
#include "stm32f4xx.h"
#include <string.h>
#include <math.h>

float car_dir_init;//小车启动时的初始绝对角度【其他的角度均为与此角度的相对角度】
/*角度PID参数*/
static float dir_pid_param_P = 1.0f; //PID的P参数
static float dir_pid_param_I = 0.0f; //PID的I参数
static float dir_pid_param_D = 0.0f; //PID的D参数

/*
通过位置式PID算法，由目标角度、当前角度计算出要改变的角度
@param dir_obs: 小车当前角度(in)
@param dir_target: 小车目标角度(in)
@return: 要改变的角度
*/
static float dir_PID_Control(float dir_obs, int dir_target)
{
  /*PID算法相关变量*/
  static int s_last_target;//前次目标值
  static float s_dir_err_now = 0.f;//当前偏差量
  static float s_dir_err_int = 0.f;//偏差量积分量
  float dir_err_last, dir_err_diff;//前次偏差量，偏差量差分量
  
  float dir_change;//要改变的角度--返回值
  
  //获取偏差量
  dir_err_last = s_dir_err_now;
  s_dir_err_now = dir_target - dir_obs;
  //取最短路径
  if(s_dir_err_now > 180) s_dir_err_now -= 360;
  else if(s_dir_err_now <= -180) s_dir_err_now += 360;
  
  //目标值变化时，积分量、微分量清零
  if(s_last_target != dir_target) {
    s_dir_err_int = 0;
    dir_err_last = s_dir_err_now;
    s_last_target = dir_target;
  }
  
  //比例控制P--响应速度
  dir_change = dir_pid_param_P * s_dir_err_now;

  //积分控制I--稳态误差
  s_dir_err_int += s_dir_err_now;
  if(fabs(s_dir_err_now) > 45.f) {//积分分离PID算法
    s_dir_err_int = 0;
  }
  dir_change += dir_pid_param_I * s_dir_err_int;
  
  //微分控制D--超调量
  dir_err_diff = s_dir_err_now - dir_err_last;
  dir_change += dir_pid_param_D * dir_err_diff;
  
  return dir_change;
}

/*********************************************************************************************
*名称：dir_Cal_Speed_Target
*功能：通过物理模型，由待改变角度和小车目标速度计算各轮目标速度
*参数：待改变角度；小车目标速度；各车轮目标速度
*返回：无
*修改：
*********************************************************************************************/
static void dir_Conver_Speed(float dir_change, int8_t car_speed, int8_t wheel_speed[]) {
  int speed_diff;//两侧轮速度差距（左侧轮速度-右侧轮速度）
  int speed_left, speed_right;//左右轮轮设定速度
  
  //由目标速度、待矫正角度计算左右侧轮目标速度
  speed_diff  = (int)(CAR_WIDTH * dir_change * PI / 180.f / DIR_TO_SPEED_INTERVAL)/10;//扇形模型--cm/s
  speed_left  = car_speed + speed_diff/2;
  speed_right = car_speed - speed_diff/2;
  //将速度目标限制在[-100,100]范围内
  speed_left = speed_left>100 ? 100 : (speed_left<-100 ? -100 : speed_left);
  speed_right = speed_right>100 ? 100 : (speed_right<-100 ? -100 : speed_right);
  
  //将左右侧轮目标速度赋值给输出数组
  wheel_speed[SPEED_H1A_INDEX] = speed_left;
  wheel_speed[SPEED_H2A_INDEX] = speed_left;
  wheel_speed[SPEED_H3A_INDEX] = speed_left;
  wheel_speed[SPEED_H4A_INDEX] = speed_right;
  wheel_speed[SPEED_H5A_INDEX] = speed_right;
  wheel_speed[SPEED_H6A_INDEX] = speed_right;
}

/*********************************************************************************************
*名称：dir_Cal_Speed_Target
*功能：通过小车目标速度、小车当前方向，计算小车各轮目标速度
*参数：小车目标速度；存放各轮目标速度的数组
*返回：无
*修改：
*********************************************************************************************/
void dir_Cal_Speed_Target(int8_t carspeed_target, int8_t wheelspeed_target[]) {
  extern float imu2_yaw;
  if(carspeed_target == 0) {//保险机制
    memset(wheelspeed_target, 0, 6);
    return;
  }
  
  //获取当前角度
  float dir_now = imu2_yaw - car_dir_init;
  if(dir_now > 180.f) {
    dir_now -= 360.f;
  } else if(dir_now <= -180.f) {
    dir_now += 360.f;
  }
  //获取要改变的角度
  float dir_change = dir_PID_Control(dir_now, 0);
  //获取各轮目标速度
  dir_Conver_Speed(dir_change, carspeed_target, wheelspeed_target);
}
