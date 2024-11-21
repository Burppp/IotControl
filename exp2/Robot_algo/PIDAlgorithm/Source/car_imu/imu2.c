#define USE_MAG	1
#define sensors(x)	(1)

#define ENABLE_STATE(x) 	do{}while(0)
#define DISABLE_STATE(x)	do{}while(0)

#define XYZ_AXIS_COUNT 3
#include "imu2.h"
#include <stdint.h>
#include <math.h>
#include "common/common.h"
#include "sensor.h"
#include "common/axis.h"
#include "compass.h"
#include "common/maths.h"
#include "acceleration.h"
#include "gyro.h"

#define DEBUG
#undef  DEBUG
#ifdef DEBUG
int printk(const char *fmt, ...);
#define Dbg(...) do{printk(__VA_ARGS__);}while(0)
#else
#define Dbg(...) do{}while(0)
#endif
	
typedef struct {
    float w,x,y,z;
} quaternion;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}
typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;
#define EULER_INITIALIZE  { { 0, 0, 0 } }

typedef struct imuConfig_s {
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
} imuConfig_t;


#define SPIN_RATE_LIMIT 20

#define ATTITUDE_RESET_QUIET_TIME 250000   // 250ms - gyro quiet period after disarm before attitude reset
#define ATTITUDE_RESET_GYRO_LIMIT 15       // 15 deg/sec - gyro limit for quiet period
#define ATTITUDE_RESET_KP_GAIN    25.0     // dcmKpGain value to use during attitude reset
#define ATTITUDE_RESET_ACTIVE_TIME 500000  // 500ms - Time to wait for attitude to converge at high gain
#define GPS_COG_MIN_GROUNDSPEED 500        // 500cm/s minimum groundspeed for a gps heading to be considered valid


imuConfig_t imuConfig ={
    .dcm_kp = 100, //2500,                // 1.0 * 10000
    .dcm_ki = 0,                   // 0.003 * 10000
    .small_angle = 25,
};

typedef struct imuRuntimeConfig_s {
    float dcm_ki;
    float dcm_kp;
} imuRuntimeConfig_t;
imuRuntimeConfig_t imuRuntimeConfig;

static float smallAngleCosZ = 0;
// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
attitudeEulerAngles_t attitude = EULER_INITIALIZE;

float accAverage[XYZ_AXIS_COUNT];
float rMat[3][3];
 quaternion q = QUATERNION_INITIALIZE;
 quaternionProducts qP = QUATERNION_PRODUCTS_INITIALIZE;
user_data_t userdata;

static void imuQuaternionComputeProducts(quaternion *quat, quaternionProducts *quatProd)
{
    quatProd->ww = quat->w * quat->w;
    quatProd->wx = quat->w * quat->x;
    quatProd->wy = quat->w * quat->y;
    quatProd->wz = quat->w * quat->z;
    quatProd->xx = quat->x * quat->x;
    quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;
    quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;
    quatProd->zz = quat->z * quat->z;
}
static void imuComputeRotationMatrix(void){
    imuQuaternionComputeProducts(&q, &qP);

    rMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    rMat[0][1] = 2.0f * (qP.xy + -qP.wz);
    rMat[0][2] = 2.0f * (qP.xz - -qP.wy);

    rMat[1][0] = 2.0f * (qP.xy - -qP.wz);
    rMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    rMat[1][2] = 2.0f * (qP.yz + -qP.wx);

    rMat[2][0] = 2.0f * (qP.xz + -qP.wy);
    rMat[2][1] = 2.0f * (qP.yz - -qP.wx);
    rMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;

#if defined(SIMULATOR_BUILD) && !defined(USE_IMU_CALC) && !defined(SET_IMU_FROM_EULER)
    rMat[1][0] = -2.0f * (qP.xy - -qP.wz);
    rMat[2][0] = -2.0f * (qP.xz + -qP.wy);
#endif
}


/*
* Calculate RC time constant used in the accZ lpf.
*/
static float calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_cutoff)
{
    return 0.5f / (M_PIf * accz_lpf_cutoff);
}

static float calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

void imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value)
{
    imuRuntimeConfig.dcm_kp = imuConfig.dcm_kp / 10000.0f;
    imuRuntimeConfig.dcm_ki = imuConfig.dcm_ki / 10000.0f;

    smallAngleCosZ = cos_approx(degreesToRadians(imuConfig.small_angle));

   // fc_acc = calculateAccZLowPassFilterRCTimeConstant(5.0f); // Set to fix value
    //throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);
    
    //throttleAngleValue = throttle_correction_value;
}

void imuInit(void)
{


    imuComputeRotationMatrix();

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&imuUpdateLock, NULL) != 0) {
        printf("Create imuUpdateLock error!\n");
    }
#endif
}



static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz,
                                bool useCOG, float courseOverGround, const float dcmKpGain)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

    // Calculate general spin rate (rad/s)
    const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

    // Use raw heading error (from GPS or whatever else)
    float ex = 0, ey = 0, ez = 0;
    if (useCOG) {
        while (courseOverGround >  M_PIf) {
            courseOverGround -= (2.0f * M_PIf);
        }

        while (courseOverGround < -M_PIf) {
            courseOverGround += (2.0f * M_PIf);
        }

        const float ez_ef = (- sin_approx(courseOverGround) * rMat[0][0] - cos_approx(courseOverGround) * rMat[1][0]);

        ex = rMat[2][0] * ez_ef;
        ey = rMat[2][1] * ez_ef;
        ez = rMat[2][2] * ez_ef;
    }

#ifdef USE_MAG
    // Use measured magnetic field vector
    float recipMagNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && recipMagNorm > 0.01f) {
        // Normalise magnetometer measurement
        recipMagNorm = invSqrt(recipMagNorm);
        mx *= recipMagNorm;
        my *= recipMagNorm;
        mz *= recipMagNorm;

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles

        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
        const float bx = sqrtf(hx * hx + hy * hy);

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        const float ez_ef = -(hy * bx);

        // Rotate mag error vector back to BF and accumulate
        ex += rMat[2][0] * ez_ef;
        ey += rMat[2][1] * ez_ef;
        ez += rMat[2][2] * ez_ef;
    }
#else
    UNUSED(useMag);
    UNUSED(mx);
    UNUSED(my);
    UNUSED(mz);
#endif

    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if (imuRuntimeConfig.dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = imuRuntimeConfig.dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
}

static void imuUpdateEulerAngles(void)
{
//    quaternionProducts buffer;

    if (0/*FLIGHT_MODE(HEADFREE_MODE)*/) {
       /*imuQuaternionComputeProducts(&headfree, &buffer);

       attitude.values.roll = lrintf(atan2_approx((+2.0f * (buffer.wx + buffer.yz)), (+1.0f - 2.0f * (buffer.xx + buffer.yy))) * (1800.0f / M_PIf));
       attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(+2.0f * (buffer.wy - buffer.xz))) * (1800.0f / M_PIf));
       attitude.values.yaw = lrintf((-atan2_approx((+2.0f * (buffer.wz + buffer.xy)), (+1.0f - 2.0f * (buffer.yy + buffer.zz))) * (1800.0f / M_PIf)));
			*/
		} else {
       attitude.values.roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
       attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (1800.0f / M_PIf));
       attitude.values.yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf)));
    }

    if (attitude.values.yaw < 0)
        attitude.values.yaw += 3600;

    // Update small angle state
    if (rMat[2][2] > smallAngleCosZ) {
        ENABLE_STATE(SMALL_ANGLE);
    } else {
        DISABLE_STATE(SMALL_ANGLE);
    }
}

static bool imuIsAccelerometerHealthy(float *accAverage)
{
    float accMagnitudeSq = 0;
    for (int axis = 0; axis < 3; axis++) {
        const float a = accAverage[axis];
        accMagnitudeSq += a * a;
    }

    accMagnitudeSq = accMagnitudeSq * sq(acc.dev.acc_1G_rec);

    // Accept accel readings only in range 0.9g - 1.1g
    return (0.81f < accMagnitudeSq) && (accMagnitudeSq < 1.21f);
}
// Calculate the dcmKpGain to use. When armed, the gain is imuRuntimeConfig.dcm_kp * 1.0 scaling.
// When disarmed after initial boot, the scaling is set to 10.0 for the first 20 seconds to speed up initial convergence.
// After disarming we want to quickly reestablish convergence to deal with the attitude estimation being incorrect due to a crash.
//   - wait for a 250ms period of low gyro activity to ensure the craft is not moving
//   - use a large dcmKpGain value for 500ms to allow the attitude estimate to quickly converge
//   - reset the gain back to the standard setting
static float imuCalcKpGain(timeUs_t currentTimeUs, bool useAcc, float *gyroAverage)
{
#if 0	
    static bool lastArmState = false;
    static timeUs_t gyroQuietPeriodTimeEnd = 0;
    static timeUs_t attitudeResetTimeEnd = 0;
    static bool attitudeResetCompleted = false;
    float ret;
    bool attitudeResetActive = false;

    const bool armState = 0;//ARMING_FLAG(ARMED);
	
    if (!armState) {
        if (lastArmState) {   // Just disarmed; start the gyro quiet period
            gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
            attitudeResetTimeEnd = 0;
            attitudeResetCompleted = false;
        }

        // If gyro activity exceeds the threshold then restart the quiet period.
        // Also, if the attitude reset has been complete and there is subsequent gyro activity then
        // start the reset cycle again. This addresses the case where the pilot rights the craft after a crash.
        if ((attitudeResetTimeEnd > 0) || (gyroQuietPeriodTimeEnd > 0) || attitudeResetCompleted) {
            if ((fabsf(gyroAverage[X]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (fabsf(gyroAverage[Y]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (fabsf(gyroAverage[Z]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (!useAcc)) {

                gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
                attitudeResetTimeEnd = 0;
            }
        }
        if (attitudeResetTimeEnd > 0) {        // Resetting the attitude estimation
            if (currentTimeUs >= attitudeResetTimeEnd) {
                gyroQuietPeriodTimeEnd = 0;
                attitudeResetTimeEnd = 0;
                attitudeResetCompleted = true;
            } else {
                attitudeResetActive = true;
            }
        } else if ((gyroQuietPeriodTimeEnd > 0) && (currentTimeUs >= gyroQuietPeriodTimeEnd)) {
            // Start the high gain period to bring the estimation into convergence
            attitudeResetTimeEnd = currentTimeUs + ATTITUDE_RESET_ACTIVE_TIME;
            gyroQuietPeriodTimeEnd = 0;
        }
    }
    lastArmState = armState;

    if (attitudeResetActive) {
        ret = ATTITUDE_RESET_KP_GAIN;
    } else {
       ret = imuRuntimeConfig.dcm_kp;
       if (!armState) {
          ret = ret * 10.0f; // Scale the kP to generally converge faster when disarmed.
       }
    }
		return ret;
#else
		static unsigned int init = 0;
		if (init == 0) init = currentTimeUs + 500000;
		if (currentTimeUs < init) return ATTITUDE_RESET_KP_GAIN;
		else  return imuRuntimeConfig.dcm_kp;
#endif
    
}

void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
    static timeUs_t previousIMUUpdateTime;
    bool useAcc = false;
    bool useMag = false;
    bool useCOG = false; // Whether or not correct yaw via imuMahonyAHRSupdate from our ground course
    float courseOverGround = 0; // To be used when useCOG is true.  Stored in Radians

    const timeDelta_t deltaT = (int)currentTimeUs - (int)previousIMUUpdateTime;
    previousIMUUpdateTime = currentTimeUs;

	
		float mag_magADC[3]; //add xuzhy
	  static timeDelta_t lastMag = 0;
#ifdef USE_MAG
    if (sensors(SENSOR_MAG) && compassIsHealthy()
#ifdef USE_GPS_RESCUE
        && !gpsRescueDisableMag()
#endif
        ) {
        //磁力计关闭
        useMag = false;
				mag_magADC[X] = mag.magADC[X];
			  mag_magADC[Y] = mag.magADC[Y];
				mag_magADC[Z] = mag.magADC[Z];
				
				mag.magADC[X] = 0;
				mag.magADC[Y] = 0;
				mag.magADC[Z] = 0;
				lastMag = currentTimeUs;
					/*
				float r = sqrtf(mag_magADC[X]*mag_magADC[X] + mag_magADC[Y]*mag_magADC[Y]);
				float a =	acos(mag_magADC[X] /r) * 1800/ 3.1415926f;
				if (mag_magADC[Y] < 0)  a = 3600 - a;
					
				static float lastMa  = 0;
					a = lastMa * 0.9f + a * 0.1f;
				static int lastGa = 0;
				int dga = attitude.values.yaw - lastGa;
			  int dma = a - lastMa;
				lastGa = attitude.values.yaw ;
				lastMa = a;
				
				*/
				//Dbg("%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d\n", (int)mag_magADC[X],(int)mag_magADC[Y], (int)mag_magADC[Z], (int)attitude.values.yaw, (int)a, dga, dma, (int)r);
    } else {
				if (lastMag != 0 && currentTimeUs - lastMag > 100000) {
					void beepLong(int n);
					//beepLong(1);
				}
		}
#endif

    float gyroAverage[XYZ_AXIS_COUNT];
    if (!gyroGetAccumulationAverage(gyroAverage)) {
			return;
		}
		
    if (accGetAccumulationAverage(accAverage)) {
        useAcc = imuIsAccelerometerHealthy(accAverage);
    }

    imuMahonyAHRSupdate(deltaT * 1e-6f,
                        DEGREES_TO_RADIANS(gyroAverage[X]), DEGREES_TO_RADIANS(gyroAverage[Y]), DEGREES_TO_RADIANS(gyroAverage[Z]),
                        useAcc, accAverage[X], accAverage[Y], accAverage[Z],
                        useMag, mag_magADC[X], mag_magADC[Y], mag_magADC[Z],
                        useCOG, courseOverGround,  imuCalcKpGain(currentTimeUs, useAcc, gyroAverage));

    imuUpdateEulerAngles();

}

void imuUpdateAttitude(timeUs_t currentTimeUs)
{
    if (/*sensors(SENSOR_ACC) &&*/ acc.isAccelUpdatedAtLeastOnce) {
        //IMU_LOCK;

        imuCalculateEstimatedAttitude(currentTimeUs);
        //IMU_UNLOCK;
        
        // Update the throttle correction for angle and supply it to the mixer
        //int throttleAngleCorrection = 0;
        //if (throttleAngleValue && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && ARMING_FLAG(ARMED)) {
        //    throttleAngleCorrection = calculateThrottleAngleCorrection();
        //}
        //mixerSetThrottleAngleCorrection(throttleAngleCorrection);
			

    } else {
        acc.accADC[X] = 0;
        acc.accADC[Y] = 0;
        acc.accADC[Z] = 0;
    }
}



///////////////////////////////////////////////////////////////////////
#include <stdio.h>

uint32_t micros(void);
void delay(uint32_t millis) ;
void buzzer(int st);

void spiInit(void);

uint8_t mpu9250SpiDetect(const busDevice_t *bus);

bool gyroInit(void);
bool accInit(uint32_t gyroSamplingInverval);
bool compassInit(void);

void gyroUpdate(timeUs_t currentTimeUs);
void accUpdate(timeUs_t currentTimeUs, rollAndPitchTrims_t *rollAndPitchTrims);
void compassUpdate(timeUs_t currentTimeUs);

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void gyroStartCalibration(bool isFirstArmingCalibration);
 bool isGyroCalibrationComplete(void);
  bool accIsCalibrationComplete(void);
	void compassStartCalibration(void);
	bool isCompassCalibrationComplete();
	
rollAndPitchTrims_t rollAndPitchTrims;

void updateImu(void)
{
	imuUpdateAttitude(micros());
}

static uint8_t imuRun = 0;
	
void ImuTimerHandler(unsigned int tick)
{
	//static uint32_t tick = 0;
	if (imuRun == 0) return;
	timeUs_t ct = micros();
  
	gyroUpdate(ct);
	if (tick % 1 == 0) { //1000Hz
		accUpdate(ct, &rollAndPitchTrims);
	}
	if (tick % 25 == 0) {    //40Hz
		compassUpdate(ct); 
	}
}

static uint8_t mpu9250_initok = 0;
void imu2_init(void) {
	unsigned int ticktick =0;
  
	initialise(); //延时函数初始化
	spiInit();    //SPI初始化
  flash_read(FLASH_USER_START_ADDR, (uint8_t *)&userdata, sizeof(user_data_t));
  delay(1000);  //延时1秒
	
  mpu9250_initok = 0;
	if (0 ==mpu9250SpiDetect(NULL)) {
		return;
	}
	gyroInit();
	accInit(1000);

	while (compassInit() == 0) {
    if(++ticktick > 3) return;
    delay(5000);  //延时5秒
	}
	imuInit();
	imuConfigure(0, 0);
	imuRun = 1;
	
  delay(100);  //延时100ms
	gyroStartCalibration(true);
  ticktick = 0;
	while (!isGyroCalibrationComplete()){
		delay(1);  //延时1ms
		ImuTimerHandler(++ticktick);
    if(ticktick > 10000) return;  //10秒未校正完成--校正失败
	}
	delay(100);  //延时100ms
  mpu9250_initok = 1;
}

float imu2_yaw;
void imu2app(void)
{
  static uint32_t tick = 0, last_counter = 0;
  extern volatile uint32_t _counter;
  
  if(mpu9250_initok == 0) return;
  if(last_counter ==_counter) return;
  last_counter = _counter;
  
  ImuTimerHandler(tick);
  if(++tick % 10 == 0) {	
    updateImu();
    int yaw;			
    yaw = - attitude.values.yaw; 
    imu2_yaw = -(M_PIf+(yaw/10.0f * (M_PIf/180.0f)))*180/M_PIf;
  }
}
