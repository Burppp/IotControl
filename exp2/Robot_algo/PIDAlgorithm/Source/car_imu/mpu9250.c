#include "stm32f4xx.h"
#include <stdio.h>
#include "common/common.h"
#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "compass.h"
#include "mpu9250.h"

#define CS_L	  (GPIO_ResetBits(GPIOB, GPIO_Pin_12))
#define CS_H	  (GPIO_SetBits(GPIOB, GPIO_Pin_12))

void spiInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI2时钟
	
 /* Configure SPIy pins: SCK, MISO and MOSI ---------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  /* Configure SCK and MOSI pins as Alternate Function Push Pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  /* Configure MISO pin as Input Floating  */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  //GPIO_Init(GPIOB, &GPIO_InitStructure);  
	CS_H;

  

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;//SPI_BaudRatePrescaler_64;		//定义波特率预分频的值:波特率预分频值为8
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
  SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
  SPI_Cmd(SPI2, ENABLE); //使能SPI外设
	
}


static uint8_t spiRW(uint8_t ch)
{		 			 
 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
	
	SPI_I2S_SendData(SPI2, ch); //通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
 
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据	
}
/*
static void spiRead(uint8_t *buf, int len)
{
	int i;
	CS_L;
	for (i=0; i<len; i++) {
		buf[i] = spiRW(0xff);
	}
	CS_H;
}
static void spiWrite(uint8_t *buf, int len)
{
	int i;
	CS_L;
	for (i=0; i<len; i++) {
		spiRW(buf[i]);
	}
	CS_H;
}
*/
#define delayMicroseconds(x)	delayus(x)
#define spiTransferByte(bus, data) spiRW(data)
//#define spiBusReadRegister(bus, reg)	(spiRW(reg),spiRW(0xff))
#define IOHi(io)	CS_H
#define IOLo(io)	CS_L
#define busDevice_t	void
static void  spiTransfer_x(uint8_t* tx, uint8_t *rx, uint8_t len)	
{ 
	int i; 
	for (i=0;i<len; i++){ 
		if (tx!=NULL && rx != NULL) rx[i] = spiRW(tx[i]);		
		else if (tx == NULL && rx != NULL) rx[i] = spiRW(0xff);
		else if (tx != NULL && rx == NULL) spiRW(tx[i]);
	}		
}
#define spiTransfer(bus, tx, rx, len) spiTransfer_x(tx, rx, len)

int mpu9250SpiWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    IOLo(bus->busdev_u.spi.csnPin);;
    delayMicroseconds(1);
    spiTransferByte(bus->busdev_u.spi.instance, reg);
    spiTransferByte(bus->busdev_u.spi.instance, data);
    IOHi(bus->busdev_u.spi.csnPin);
    delayMicroseconds(1);

    return 1;
}
uint8_t spiBusReadRegister(const busDevice_t *bus, uint8_t reg)
{
	uint8_t d;
	  IOLo(bus->busdev_u.spi.csnPin);;
    delayMicroseconds(1);
    spiTransferByte(bus->busdev_u.spi.instance, reg | 0x80);
    d = spiTransferByte(bus->busdev_u.spi.instance, 0xff);
    IOHi(bus->busdev_u.spi.csnPin);
	return d;
}
 bool mpu9250SpiSlowReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    IOLo(bus->busdev_u.spi.csnPin);
    delayMicroseconds(1);
    spiTransferByte(bus->busdev_u.spi.instance, reg | 0x80); // read transaction
    spiTransfer(bus->busdev_u.spi.instance, NULL, data, length);
    IOHi(bus->busdev_u.spi.csnPin);
    delayMicroseconds(1);

    return true;
}

static void mpu9250AccAndGyroInit(gyroDev_t *gyro);

bool mpuGyroReadSPI(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = mpu9250SpiSlowReadRegisterBuffer(&gyro->bus, MPU_RA_GYRO_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}
bool mpuAccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = mpu9250SpiSlowReadRegisterBuffer(&acc->bus, MPU_RA_ACCEL_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}
uint8_t mpuGyroDLPF(gyroDev_t *gyro)
{
    uint8_t ret = 0;

    // If gyro is in 32KHz mode then the DLPF bits aren't used
    if (gyro->gyroRateKHz <= GYRO_RATE_8_kHz) {
        switch (gyro->hardware_lpf) {
            case GYRO_HARDWARE_LPF_1KHZ_SAMPLE:
                ret = 1;
                break;

            case GYRO_HARDWARE_LPF_NORMAL:
            default:
                ret = 0;
                break;
        }
    }
    return ret;
}
void mpu9250SpiGyroInit(gyroDev_t *gyro)
{
    //mpuGyroInit(gyro);

    mpu9250AccAndGyroInit(gyro);

    //spiResetErrorCounter(gyro->bus.busdev_u.spi.instance);

    //spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_FAST); //high speed now that we don't need to write to the slow registers

    mpuGyroReadSPI(gyro);

    if ((((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1)/* || spiGetErrorCounter(gyro->bus.busdev_u.spi.instance) != 0*/) {
        //spiResetErrorCounter(gyro->bus.busdev_u.spi.instance);
        //failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

void mpu9250SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
	//acc->acc_1G = 512 * 3;
}

int mpu9250SpiWriteRegisterVerify(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    mpu9250SpiWriteRegister(bus, reg, data);
    delayMicroseconds(100);

    uint8_t attemptsRemaining = 20;
    do {
        uint8_t in;
        mpu9250SpiSlowReadRegisterBuffer(bus, reg, &in, 1);
        if (in == data) {
            return 1;
        } else {
            //debug[3]++;
            mpu9250SpiWriteRegister(bus, reg, data);
            delayMicroseconds(100);
        }
    } while (attemptsRemaining--);
    return 0;
}

static void mpu9250AccAndGyroInit(gyroDev_t *gyro) {

    //spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATION); //low speed for writing to slow registers

    mpu9250SpiWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
    delayMicroseconds(150);

    mpu9250SpiWriteRegisterVerify(&gyro->bus, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);

    mpu9250SpiWriteRegisterVerify(&gyro->bus, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);

    mpu9250SpiWriteRegisterVerify(&gyro->bus, MPU_RA_CONFIG, mpuGyroDLPF(gyro));

    mpu9250SpiWriteRegisterVerify(&gyro->bus, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);

    mpu9250SpiWriteRegisterVerify(&gyro->bus, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    mpu9250SpiWriteRegisterVerify(&gyro->bus, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN

#if defined(USE_MPU_DATA_READY_SIGNAL)
    mpu9250SpiWriteRegisterVerify(&gyro->bus, MPU_RA_INT_ENABLE, 0x01); //this resets register MPU_RA_PWR_MGMT_1 and won't read back correctly.
#endif

    //spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_FAST);
}

uint8_t mpu9250SpiDetect(const busDevice_t *bus)
{

    //spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_INITIALIZATION); //low speed
    mpu9250SpiWriteRegister(bus, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);

    uint8_t attemptsRemaining = 20;
    do {
        delayMicroseconds(150);
        const uint8_t in = spiBusReadRegister(bus, MPU_RA_WHO_AM_I);
        if (in == MPU9250_WHO_AM_I_CONST || in == MPU9255_WHO_AM_I_CONST) {
            return 1;
        }
        if (!attemptsRemaining) {
            return 0;
        }
    } while (attemptsRemaining--);

    //spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_FAST);

    return 0;
}

bool mpu9250SpiAccDetect(accDev_t *acc)
{
    /*if (acc->mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }*/

    acc->initFn = mpu9250SpiAccInit;
    acc->readFn = mpuAccRead;

    return true;
}

bool mpu9250SpiGyroDetect(gyroDev_t *gyro)
{
    /*if (gyro->mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }*/

    gyro->initFn = mpu9250SpiGyroInit;
    gyro->readFn = mpuGyroReadSPI;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}

/************************************************* ak8963 ***********************************************/
#define AK8963_MAG_I2C_ADDRESS          0x0C

#define AK8963_DEVICE_ID                0x48

// Registers
#define AK8963_MAG_REG_WIA              0x00
#define AK8963_MAG_REG_INFO             0x01
#define AK8963_MAG_REG_ST1              0x02
#define AK8963_MAG_REG_HXL              0x03
#define AK8963_MAG_REG_HXH              0x04
#define AK8963_MAG_REG_HYL              0x05
#define AK8963_MAG_REG_HYH              0x06
#define AK8963_MAG_REG_HZL              0x07
#define AK8963_MAG_REG_HZH              0x08
#define AK8963_MAG_REG_ST2              0x09
#define AK8963_MAG_REG_CNTL1            0x0A
#define AK8963_MAG_REG_CNTL2            0x0B
#define AK8963_MAG_REG_ASCT             0x0C // self test
#define AK8963_MAG_REG_I2CDIS           0x0F
#define AK8963_MAG_REG_ASAX             0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAY             0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAZ             0x12 // Fuse ROM z-axis sensitivity adjustment value

#define READ_FLAG                       0x80
#define I2C_SLV0_EN                     0x80

#define ST1_DATA_READY                  0x01
#define ST1_DATA_OVERRUN                0x02

#define ST2_MAG_SENSOR_OVERFLOW         0x08

#define CNTL1_MODE_POWER_DOWN           0x00
#define CNTL1_MODE_ONCE                 0x01
#define CNTL1_MODE_CONT1                0x02
#define CNTL1_MODE_CONT2                0x06
#define CNTL1_MODE_SELF_TEST            0x08
#define CNTL1_MODE_FUSE_ROM             0x0F
#define CNTL1_BIT_14_BIT                0x00
#define CNTL1_BIT_16_BIT                0x10

#define CNTL2_SOFT_RESET                0x01

#define I2CDIS_DISABLE_MASK             0x1D

#define MPU6500_BIT_INT_ANYRD_2CLEAR        (1 << 4)
#define MPU6500_BIT_BYPASS_EN               (1 << 1)

#define spiBusWriteRegister							mpu9250SpiWriteRegister
#define spiBusReadRegisterBuffer				mpu9250SpiSlowReadRegisterBuffer


#define __disable_irq()
#define __enable_irq()

static int16_t parseMag(uint8_t *raw, int16_t gain) {
  int ret = (int16_t)(raw[1] << 8 | raw[0]) * gain / 256;
	if (ret > INT16_MAX) ret = INT16_MAX;
	if (ret < INT16_MIN) ret = INT16_MIN;
  return ret; //constrain(ret, INT16_MIN, INT16_MAX);
}

static bool ak8963SpiWriteRegisterDelay(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    spiBusWriteRegister(bus, reg, data);
    delayMicroseconds(10);
    return true;
}
static bool ak8963SlaveWriteRegister(const busDevice_t *slavedev, uint8_t reg, uint8_t data)
{
    //const busDevice_t *bus = slavedev->busdev_u.mpuSlave.master;

    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_ADDR, /*slavedev->busdev_u.mpuSlave.address*/AK8963_MAG_I2C_ADDRESS); // set I2C slave address for write
    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_REG, reg);                             // set I2C slave register
    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_DO, data);                             // set I2C sLave value
    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_CTRL, (1 & 0x0F) | I2C_SLV0_EN);       // write 1 byte
    return true;
}

static bool ak8963SlaveReadRegisterBuffer(const busDevice_t *slavedev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    //const busDevice_t *bus = slavedev->busdev_u.mpuSlave.master;

    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_ADDR, /*slavedev->busdev_u.mpuSlave.address*/AK8963_MAG_I2C_ADDRESS | READ_FLAG); // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_REG, reg);                             // set I2C slave register
    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_CTRL, (len & 0x0F) | I2C_SLV0_EN);     // read number of bytes
    delay(4);
    __disable_irq();
    bool ack = spiBusReadRegisterBuffer(/*bus*/NULL, MPU_RA_EXT_SENS_DATA_00, buf, len);            // read I2C
    __enable_irq();
    return ack;
}

static bool ak8963ReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *buf, uint8_t len)
{
//#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    //if (busdev->bustype == BUSTYPE_MPU_SLAVE) {
        return ak8963SlaveReadRegisterBuffer(busdev, reg, buf, len);
    //}
//#endif
    //return busReadRegisterBuffer(busdev, reg, buf, len);
}



static bool ak8963WriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
//#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    //if (busdev->bustype == BUSTYPE_MPU_SLAVE) {
        return ak8963SlaveWriteRegister(busdev, reg, data);
//    }
//#endif
 //   return busWriteRegister(busdev, reg, data);
}

typedef struct queuedReadState_s {
    bool waiting;
    uint8_t len;
    uint32_t readStartedAt;                                                                 // time read was queued in micros.
} queuedReadState_t;

static queuedReadState_t queuedRead = { false, 0, 0};

static bool ak8963SlaveStartRead(const busDevice_t *slavedev, uint8_t reg, uint8_t len)
{
    if (queuedRead.waiting) {
        return false;
    }

    //const busDevice_t *bus = slavedev->busdev_u.mpuSlave.master;

    queuedRead.len = len;

    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_ADDR, AK8963_MAG_I2C_ADDRESS/*slavedev->busdev_u.mpuSlave.address*/ | READ_FLAG);  // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_REG, reg);                             // set I2C slave register
    ak8963SpiWriteRegisterDelay(/*bus*/NULL, MPU_RA_I2C_SLV0_CTRL, (len & 0x0F) | I2C_SLV0_EN);    // read number of bytes

    queuedRead.readStartedAt = micros();
    queuedRead.waiting = true;

    return true;
}

static uint32_t ak8963SlaveQueuedReadTimeRemaining(void)
{
    if (!queuedRead.waiting) {
        return 0;
    }

    int32_t timeSinceStarted = micros() - queuedRead.readStartedAt;

    int32_t timeRemaining = 8000 - timeSinceStarted;

    if (timeRemaining < 0) {
        return 0;
    }

    return timeRemaining;
}

static bool ak8963SlaveCompleteRead(const busDevice_t *slavedev, uint8_t *buf)
{
    uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();

    //const busDevice_t *bus = slavedev->busdev_u.mpuSlave.master;

    if (timeRemaining > 0) {
        delayMicroseconds(timeRemaining);
    }

    queuedRead.waiting = false;

    spiBusReadRegisterBuffer(/*bus*/NULL, MPU_RA_EXT_SENS_DATA_00, buf, queuedRead.len);            // read I2C buffer
    return true;
}

static bool ak8963SlaveReadData(const busDevice_t *busdev, uint8_t *buf)
{
    typedef enum {
        CHECK_STATUS = 0,
        WAITING_FOR_STATUS,
        WAITING_FOR_DATA
    } ak8963ReadState_e;

    static ak8963ReadState_e state = CHECK_STATUS;

    bool ack = false;

    // we currently need a different approach for the MPU9250 connected via SPI.
    // we cannot use the ak8963SlaveReadRegisterBuffer() method for SPI, it is to slow and blocks for far too long.

    bool retry = true;

restart:
    switch (state) {
        case CHECK_STATUS: {
            ak8963SlaveStartRead(busdev, AK8963_MAG_REG_ST1, 1);
            state = WAITING_FOR_STATUS;
            return false;
        }

        case WAITING_FOR_STATUS: {
            uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();
            if (timeRemaining) {
                return false;
            }

            ack = ak8963SlaveCompleteRead(busdev, &buf[0]);

            uint8_t status = buf[0];

            if (!ack || (status & ST1_DATA_READY) == 0) {
                // too early. queue the status read again
                state = CHECK_STATUS;
                if (retry) {
                    retry = false;
                    goto restart;
               }
               return false;
            }

            // read the 6 bytes of data and the status2 register
            ak8963SlaveStartRead(busdev, AK8963_MAG_REG_HXL, 7);

            state = WAITING_FOR_DATA;
            return false;
        }

        case WAITING_FOR_DATA: {
            uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();
            if (timeRemaining) {
                return false;
            }

            ack = ak8963SlaveCompleteRead(busdev, &buf[0]);
            state = CHECK_STATUS;
        }
    }

    return ack;
}

static bool ak8963Read(magDev_t *mag, int16_t *magData)
{
    bool ack = false;
    uint8_t buf[7];

    const busDevice_t *busdev = &mag->busdev;

    ack = ak8963SlaveReadData(busdev, buf);


    uint8_t status2 = buf[6];
    if (!ack) {
        return false;
    }

    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_ONCE); // start reading again    uint8_t status2 = buf[6];

    if (status2 & ST2_MAG_SENSOR_OVERFLOW) {
        return false;
    }

    magData[Y] = parseMag(buf + 0, mag->magGain[X]);
    magData[X] = parseMag(buf + 2, mag->magGain[Y]);
    magData[Z] = -parseMag(buf + 4, mag->magGain[Z]);

    return true;
}
static bool ak8963Init(magDev_t *mag)
{
    uint8_t asa[3];
    uint8_t status;

    const busDevice_t *busdev = &mag->busdev;

    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN);               // power down before entering fuse mode
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_FUSE_ROM);                 // Enter Fuse ROM access mode
    ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ASAX, asa, sizeof(asa));                // Read the x-, y-, and z-axis calibration values

    mag->magGain[X] = asa[X] + 128;
    mag->magGain[Y] = asa[Y] + 128;
    mag->magGain[Z] = asa[Z] + 128;

    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN);               // power down after reading.

    // Clear status registers
    ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ST1, &status, 1);
    ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ST2, &status, 1);

    // Trigger first measurement
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_ONCE);
    return true;
}
static void ak8963BusInit(busDevice_t *busdev)
{


//#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))

        //rescheduleTask(TASK_COMPASS, TASK_PERIOD_HZ(40));

        // initialze I2C master via SPI bus
        ak8963SpiWriteRegisterDelay(/*busdev->busdev_u.mpuSlave.master*/NULL, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR | MPU6500_BIT_BYPASS_EN);
        ak8963SpiWriteRegisterDelay(/*busdev->busdev_u.mpuSlave.master*/NULL, MPU_RA_I2C_MST_CTRL, 0x0D); // I2C multi-master / 400kHz
        ak8963SpiWriteRegisterDelay(/*busdev->busdev_u.mpuSlave.master*/NULL, MPU_RA_USER_CTRL, 0x30);   // I2C master mode, SPI mode only
  
}
void ak8963BusDeInit(const busDevice_t *busdev)
{
/*    switch (busdev->bustype) {
#ifdef USE_MAG_AK8963
    case BUSTYPE_I2C:
        UNUSED(busdev);
        break;
#endif

#ifdef USE_MAG_SPI_AK8963
    case BUSTYPE_SPI:
        spiPreinitByIO(busdev->busdev_u.spi.csnPin);
        break;
#endif

#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_MPU_SLAVE: */
        ak8963SpiWriteRegisterDelay(/*busdev->busdev_u.mpuSlave.master*/NULL, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR);
        /*break;
#endif
    default:
        break;
    }*/
}
bool ak8963Detect(magDev_t *mag)
{
    uint8_t sig = 0;

    busDevice_t *busdev = &mag->busdev;

    //if ((busdev->bustype == BUSTYPE_I2C || busdev->bustype == BUSTYPE_MPU_SLAVE) && busdev->busdev_u.mpuSlave.address == 0) {
    //    busdev->busdev_u.mpuSlave.address = AK8963_MAG_I2C_ADDRESS;
    //}

    ak8963BusInit(busdev);

	
	
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL2, CNTL2_SOFT_RESET);                    // reset MAG
    delay(4);

    bool ack = ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_WIA, &sig, 1);               // check for AK8963

    if (ack && sig == AK8963_DEVICE_ID) // 0x48 / 01001000 / 'H'
    {
        mag->init = ak8963Init;
        mag->read = ak8963Read;

        return true;
    }

    ak8963BusDeInit(busdev);

    return false;
}

/////////////////////////////////////////////////////////////////
void mpu9250Test(void)
{
	static int init = 0;

	delayMicroseconds(10000);
	if (init == 0) {
		init = 1;
		spiInit();
	}
}
