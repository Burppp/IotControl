#include "Arm Algorithm/arm_algorithm.h"
#include "Servo/servo.h"
#include "debug uart/debug_usart.h"
#include <string.h>
#include <math.h>

#define ABS(x) ((x)>=0?(x):-(x))
#ifndef PI
  #define PI 3.141592653
#endif
int16_t Servo_Angle[4] = {0,0,0,0};               // ����Ƕ���������
uint8_t Servo_Set_Flag = 0;                       // ����Ƕȿ��Ʊ�־

/*********************************************************************************************
* ���ƣ�arm_Coordinate_Set
* ���ܣ������������㷨
* ����������x��y����λ��cm��
* ���أ���
* �޸ģ�
*********************************************************************************************/
void arm_Coordinate_Set(int16_t x, int16_t y) {
  #define L1 10.0 /*��һ�۳���--cm*/
  #define L2 16.5 /*�ڶ��۳���--cm*/
  double xx = x, yy = y;            //����������(x,y)ת��Ϊ������(xx,yy)
  double temp1, temp2, temp3;       //���������
  double alpha, angle1, angle2;     //������ĽǶ�--������
  int16_t servo1, servo2;           //������ĽǶ�--�Ƕ���
  char info[50];                    //���Դ����������ʾ��Ϣ
  //ͨ��������������Ƕ�
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
  //�������곬����Χ--��ʾ���Ƿ����ꡱ����ִ�в���
  if(fabs(temp1)>1 || fabs(temp3)>1) {
    //sprintf(info, "Error: Illegal Coordinate!\r\n");
    //DEBUG_INFO(info, strlen(info));
    return;
  }
  //����Ƕȱ�����[-135,135]���������Ƶ�[-3��/4, 3��/4]��Χ��
  angle1 = angle1>(3*PI/4) ? (3*PI/4) : (angle1<-(3*PI/4) ? -(3*PI/4) : angle1);
  angle2 = angle2>(3*PI/4) ? (3*PI/4) : (angle2<-(3*PI/4) ? -(3*PI/4) : angle2);
  //��ʾʵ�����õ�����ֵ
  x = L1*cos(PI/4+angle1) + L2*cos(PI/4+angle1+angle2);
  y = L1*sin(PI/4+angle1) + L2*sin(PI/4+angle1+angle2);
  //sprintf(info, "Set Coordinate: x=%d, y=%d\r\n", x, y);
  //DEBUG_INFO(info, strlen(info));
  //������ת��Ϊ�Ƕ���
  servo1 = angle1/PI*180;
  servo2 = angle2/PI*180;
  //�½Ƕ���ԭ�Ƕ����ʱ�����ظ�����
  if(servo1==Servo_Angle[3] && servo2==Servo_Angle[2]) {
    return;
  }
  Servo_Angle[3] = servo1;
  Servo_Angle[2] = servo2;
  Servo_Set_Flag = 0x0C;
  set_servoAngle(Servo_Angle, 2000, &Servo_Set_Flag);
}

/*********************************************************************************************
* ���ƣ�arm_Init
* ���ܣ������۳�ʼ��
* ��������
* ���أ���
* �޸ģ�
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
