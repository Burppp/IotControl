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
*���ƣ�main()
*���ܣ�������
*��������
*���أ���
*�޸ģ�
*********************************************************************************************/
int main(void) {
  uint8_t newdata_flag = 0;                                     // �����ݱ�־λ���յ����Դ������õ����ٶȺ���λ
  delay_init(168);                                              // ��ʱ������ʼ��
  wireless_usart_init(38400);                                   // ���ߴ��ڳ�ʼ��������Ϊ 38400
  car_Init();                                                   // С���˶�ģ���ʼ��
  ms_Module_Init();                                             // �ٶȲ���ģ���ʼ��
  imu2_init();                                                  // �ǶȲ���ģ���ʼ��
  //OLED_Init();                                                  // OLED����ʼ������ʾ����ģ��MAC��ַ
  arm_Init();                                                   // �����۳�ʼ��
  laser_I2C_Init(2);
  laser_Init();
  
  Car_Direction = 1;
  Car_Speed = 40;
  while(1) {
    //�ǶȲ���ģ�����г���
    imu2app();
    //�жϵ����Դ����յ���Ϣʱ�������յ�����Ϣ
    if(Wireless_Usart_len > 0) {
      Wireless_Usart_len = 0;
      newdata_flag = wireless_protocol_parse(WIRELESS_USART_RX_BUF, strlen(WIRELESS_USART_RX_BUF));//��Ϣ���������ó����ٶ�ʱ������1
    }
    
    //�յ����������ٶ���Ϣ�󣬽��������ٶ���Ϊ����ֵ
    if(newdata_flag == 1) {
      newdata_flag = 0;
      car_Move();
    }
    
    distance = get_laserData();
    if(distance < 20 && obstacle_flag == 0)
    {
      obstacle_flag = 1;
    }
    
    //ǰ��/����ʱͨ��PID���ж��ٶ�������
    if(obstacle_flag == 0)
    {
      if(Car_Direction == 1) {          //ǰ���������ٶ�ΪCar_Speed����λ��cm/s��
      XCAR_PID_Control(Car_Speed);
      } else if(Car_Direction == 2) {   //���ˣ������ٶ�Ϊ-Car_Speed����λ��cm/s��
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
