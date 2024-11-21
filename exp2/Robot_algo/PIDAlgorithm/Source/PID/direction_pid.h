/****************************************Copyright (c)**************************************************
**
**--------------文件信息--------------------------------------------------------------------------------
**文   件   名: direction_pid.h
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
#ifndef _DIRECTION_PID_H_
#define _DIRECTION_PID_H_

//两次调用dir_Cal_Speed_Target函数（计算各轮目标速度）的间隔(s)
#define DIR_TO_SPEED_INTERVAL (0.1f)

#define FABS(x) (x>0.0f ? x : -x)

void dir_Cal_Speed_Target(signed char carspeed_target, signed char wheelspeed_target[]);
void car_Set_Target(int speed_target, int dir_target);
void car_Adjust_Target(unsigned char type, int data);
int car_Get_Target(unsigned char type);
void dir_Get_PID(unsigned char param, unsigned *pdata);
void dir_Set_PID(unsigned char param, unsigned data);

#endif  //_DIRECTION_PID_H_
