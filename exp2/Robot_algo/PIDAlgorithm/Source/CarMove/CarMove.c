#include "Motor/motor.h"
#include "CarMove/CarMove.h"
#include "PID/speed_pid.h"
#include <math.h>
#include <string.h>

int8_t Car_Direction = 0;   // 方向，0-4分别对应停止/前进/后退/左转/右转
int8_t Car_Speed = 0;       // 速度，速度范围0-100

/*********************************************************************************************
* 名称：car_Init
* 功能：小车运动功能初始化
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void car_Init(void) {
  motor_Init();
}

/*********************************************************************************************
* 名称：car_Move
* 功能：小车运动功能执行程序
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void car_Move(void) {
  extern float car_dir_init, imu2_yaw;
  switch(Car_Direction) {
    case 0: //停止
      set_motorSpeed(MOTOR_ALL, 0); //所有电机速度设为0
      for(uint8_t i = 0; i < 6; i++) {
        xcar_speed_start_flag[i] = CAR_START_MOTOR_LIMIT_TIMES;
      }
      memset(motor_ctrl, 0, 6);
      car_dir_init = 0.f;
    break;
    case 1: //前进
      set_motorSpeed(MOTOR_LEFT, Car_Speed); //左侧电机速度设为-Car_Speed
      set_motorSpeed(MOTOR_RIGHT, Car_Speed); //右侧电机速度设为Car_Speed
    case 2: //后退
      //if(fabs(car_dir_init) < 0.0001f) {
      //  car_dir_init = imu2_yaw;
      //}
      set_motorSpeed(MOTOR_LEFT, -Car_Speed); //左侧电机速度设为-Car_Speed
      set_motorSpeed(MOTOR_RIGHT,-Car_Speed); //右侧电机速度设为Car_Speed
    break;
    case 3: //左转
      set_motorSpeed(MOTOR_LEFT, -Car_Speed); //左侧电机速度设为-Car_Speed
      set_motorSpeed(MOTOR_RIGHT, Car_Speed); //右侧电机速度设为Car_Speed
    break;
    case 4: //右转
      set_motorSpeed(MOTOR_LEFT, Car_Speed); //左侧电机速度设为Car_Speed
      set_motorSpeed(MOTOR_RIGHT, -Car_Speed); //右侧电机速度设为-Car_Speed
    break;
    default:
    break;
  }
}
