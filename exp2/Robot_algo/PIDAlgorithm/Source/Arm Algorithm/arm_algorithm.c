#include "Arm Algorithm/arm_algorithm.h"
#include "Servo/servo.h"
#include "debug uart/debug_usart.h"
#include <string.h>
#include <math.h>

#define ABS(x) ((x)>=0?(x):-(x))
#ifndef PI
  #define PI 3.141592653
#endif
int16_t Servo_Angle[4] = {0,0,0,0};               // 舵机角度设置数组
uint8_t Servo_Set_Flag = 0;                       // 舵机角度控制标志

/*********************************************************************************************
* 名称：arm_Coordinate_Set
* 功能：机器臂坐标算法
* 参数：坐标x、y（单位：cm）
* 返回：无
* 修改：
*********************************************************************************************/
void arm_Coordinate_Set(int16_t x, int16_t y) {
  #define L1 10.0 /*第一臂长度--cm*/
  #define L2 16.5 /*第二臂长度--cm*/
  double xx = x, yy = y;            //将整数坐标(x,y)转换为浮点数(xx,yy)
  double temp1, temp2, temp3;       //计算过程量
  double alpha, angle1, angle2;     //计算出的角度--弧度制
  int16_t servo1, servo2;           //计算出的角度--角度制
  char info[50];                    //调试串口输出的提示信息
  //通过坐标计算出舵机角度
  if(y!=0) {
    temp1 = (xx*xx + yy*yy + L1*L1 - L2*L2) / (2 * L1 * sqrt(xx*xx + yy*yy));
    temp2 = xx / yy;
    alpha = asin(temp1) - atan(temp2);
  } else {
    temp1 = (xx*xx + yy*yy + L1*L1 - L2*L2) / (2 * L1 * xx);
    alpha = acos(temp1);
  }
  temp3 = (yy - L1*sin(alpha)) / L2;
  angle1 = alpha - PI/4;
  angle2 = asin(temp3) - alpha;
  //输入坐标超出范围--提示“非法坐标”，不执行操作
  if(fabs(temp1)>1 || fabs(temp3)>1) {
    //sprintf(info, "Error: Illegal Coordinate!\r\n");
    //DEBUG_INFO(info, strlen(info));
    return;
  }
  //舵机角度必须在[-135,135]，即弧度制的[-3π/4, 3π/4]范围内
  angle1 = angle1>(3*PI/4) ? (3*PI/4) : (angle1<-(3*PI/4) ? -(3*PI/4) : angle1);
  angle2 = angle2>(3*PI/4) ? (3*PI/4) : (angle2<-(3*PI/4) ? -(3*PI/4) : angle2);
  //提示实际设置的坐标值
  x = L1*cos(PI/4+angle1) + L2*cos(PI/4+angle1+angle2);
  y = L1*sin(PI/4+angle1) + L2*sin(PI/4+angle1+angle2);
  //sprintf(info, "Set Coordinate: x=%d, y=%d\r\n", x, y);
  //DEBUG_INFO(info, strlen(info));
  //弧度制转换为角度制
  servo1 = angle1/PI*180;
  servo2 = angle2/PI*180;
  //新角度与原角度相等时，不重复操作
  if(servo1==Servo_Angle[3] && servo2==Servo_Angle[2]) {
    return;
  }
  Servo_Angle[3] = servo1;
  Servo_Angle[2] = servo2;
  Servo_Set_Flag = 0x0C;
  set_servoAngle(Servo_Angle, 2000, &Servo_Set_Flag);
}

/*********************************************************************************************
* 名称：arm_Init
* 功能：机器臂初始化
* 参数：无
* 返回：无
* 修改：
*********************************************************************************************/
void arm_Init(void) {
  servo_Init();
  Servo_Angle[3] = 97;
  Servo_Angle[2] = -135;
  Servo_Angle[1] = 0;
  Servo_Angle[0] = 0;
  Servo_Set_Flag = 0x0F;
  set_servoAngle(Servo_Angle, 2000, &Servo_Set_Flag);
}
