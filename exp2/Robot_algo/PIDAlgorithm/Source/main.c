#include <stdio.h>
#include "stm32f4xx.h"
#include "delay/delay.h"
#include "wireless uart/wireless_usart.h"
#include "Wireless/wireless_protocol.h"
#include "CarMove/CarMove.h"
#include "MotorSpeed/motor_speed.h"
#include "imu2.h"
#include "PID/speed_pid.h"
//#include "OLED/oled.h"
#include "debug uart/debug_usart.h"
#include "Arm Algorithm/arm_algorithm.h"
#include "laser/laser.h"


float distance = 0;
uint8_t obstacle_flag = 0;
/*********************************************************************************************
*名称：main()
*功能：主函数
*参数：无
*返回：无
*修改：
*********************************************************************************************/
int main(void) {
  uint8_t newdata_flag = 0;                                     // 新数据标志位，收到调试串口设置的新速度后置位
  delay_init(168);                                              // 延时函数初始化
  wireless_usart_init(38400);                                   // 无线串口初始化波特率为 38400
  car_Init();                                                   // 小车运动模块初始化
  ms_Module_Init();                                             // 速度测量模块初始化
  imu2_init();                                                  // 角度测量模块初始化
  //OLED_Init();                                                  // OLED屏初始化：显示蓝牙模块MAC地址
  arm_Init();                                                   // 机器臂初始化
  laser_I2C_Init(2);
  laser_Init();
  
  Car_Direction = 1;
  Car_Speed = 40;
  while(1) {
    //角度测量模块运行程序
    imu2app();
    //判断到调试串口收到消息时，解析收到的消息
    if(Wireless_Usart_len > 0) {
      Wireless_Usart_len = 0;
      newdata_flag = wireless_protocol_parse(WIRELESS_USART_RX_BUF, strlen(WIRELESS_USART_RX_BUF));//消息内容是设置车轮速度时，返回1
    }
    
    //收到串口设置速度消息后，将各车轮速度设为更新值
    if(newdata_flag == 1) {
      newdata_flag = 0;
      car_Move();
    }
    
    distance = get_laserData();
    if(distance < 20 && obstacle_flag == 0)
    {
      obstacle_flag = 1;
    }
    
    //前进/后退时通过PID进行定速定向运行
    if(obstacle_flag == 0)
    {
      if(Car_Direction == 1) {          //前进：保持速度为Car_Speed【单位：cm/s】
      XCAR_PID_Control(Car_Speed);
      } else if(Car_Direction == 2) {   //后退：保持速度为-Car_Speed【单位：cm/s】
        XCAR_PID_Control(-Car_Speed);
      }
    }
    else if(obstacle_flag == 1)
    {
      //arm
      XCAR_PID_Control(0);
      arm_Coordinate_Set(25, 5);
      delay_ms(3000);
      Car_Direction = 2;
      arm_Init();
      delay_ms(3000);
      obstacle_flag = 0;
    }
    
  }
}
