#include "Motor/motor.h"
#include "CarMove/CarMove.h"
#include "PID/speed_pid.h"
#include <math.h>
#include <string.h>

int8_t Car_Direction = 0;   // ����0-4�ֱ��Ӧֹͣ/ǰ��/����/��ת/��ת
int8_t Car_Speed = 0;       // �ٶȣ��ٶȷ�Χ0-100

/*********************************************************************************************
* ���ƣ�car_Init
* ���ܣ�С���˶����ܳ�ʼ��
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void car_Init(void) {
  motor_Init();
}

/*********************************************************************************************
* ���ƣ�car_Move
* ���ܣ�С���˶�����ִ�г���
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void car_Move(void) {
  extern float car_dir_init, imu2_yaw;
  switch(Car_Direction) {
    case 0: //ֹͣ
      set_motorSpeed(MOTOR_ALL, 0); //���е���ٶ���Ϊ0
      for(uint8_t i = 0; i < 6; i++) {
        xcar_speed_start_flag[i] = CAR_START_MOTOR_LIMIT_TIMES;
      }
      memset(motor_ctrl, 0, 6);
      car_dir_init = 0.f;
    break;
    case 1: //ǰ��
      set_motorSpeed(MOTOR_LEFT, Car_Speed); //������ٶ���Ϊ-Car_Speed
      set_motorSpeed(MOTOR_RIGHT, Car_Speed); //�Ҳ����ٶ���ΪCar_Speed
    case 2: //����
      //if(fabs(car_dir_init) < 0.0001f) {
      //  car_dir_init = imu2_yaw;
      //}
      set_motorSpeed(MOTOR_LEFT, -Car_Speed); //������ٶ���Ϊ-Car_Speed
      set_motorSpeed(MOTOR_RIGHT,-Car_Speed); //�Ҳ����ٶ���ΪCar_Speed
    break;
    case 3: //��ת
      set_motorSpeed(MOTOR_LEFT, -Car_Speed); //������ٶ���Ϊ-Car_Speed
      set_motorSpeed(MOTOR_RIGHT, Car_Speed); //�Ҳ����ٶ���ΪCar_Speed
    break;
    case 4: //��ת
      set_motorSpeed(MOTOR_LEFT, Car_Speed); //������ٶ���ΪCar_Speed
      set_motorSpeed(MOTOR_RIGHT, -Car_Speed); //�Ҳ����ٶ���Ϊ-Car_Speed
    break;
    default:
    break;
  }
}
