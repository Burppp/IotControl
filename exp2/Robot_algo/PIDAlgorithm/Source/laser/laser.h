#ifndef __LASER_H_
#define __LASER_H_

#define LASER_ADDR          0xE0
#define LASER_READ          0x51

#define FRONT               0x01
#define REAR                0x03

typedef struct{
  void (*LASER_I2C_IO_Init)();
  void (*LASER_I2C_Start)();
  void (*LASER_I2C_Stop)();
  unsigned char (*LASER_I2C_WaitAck)();
  void (*LASER_I2C_WriteByte)(unsigned char);
  unsigned char (*LASER_I2C_ReadByte)(unsigned char);
}LASER_I2C_t;


void laser_I2C_Init(unsigned char dir);
void laser_Init(void);
float get_laserData(void);



#endif
