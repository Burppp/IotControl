/*
***************************************Copyright (c)***************************
**                               	  武汉钛联信息技术有限公司
**                                     		工程事业部
**                                        
**
**                                		 
**
**--------------文件信息-------------------------------------------------------
**文   件   名: dmp_pid.h
**创   建   人: 何文超
**最后修改日期: 2018年10月26日
**描        述: PID控制头文件
**              
**--------------历史版本信息---------------------------------------------------
** 创建人: 何文超
** 版  本: v1.0
** 日　期: 2018年10月26日
** 描　述: 初始创建
**
**--------------当前版本修订---------------------------------------------------
** 修改人: 
** 日　期: 
** 描　述: 
**
**-----------------------------------------------------------------------------
******************************************************************************/
#ifndef _SPEED_PID_H_
#define _SPEED_PID_H_

#include "stm32f4xx.h"

//启动时的电机控制量增量幅度限制
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
