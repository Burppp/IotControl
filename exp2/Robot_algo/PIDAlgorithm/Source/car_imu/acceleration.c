#include <stdint.h>
#include <stdbool.h>
#include "common/common.h"
#include "common/axis.h"
#include "accgyro.h"
#include "acceleration.h"
#include "common/filter.h"

#define UNUSED(x) (x) 

#define DEBUG
#undef DEBUG
#ifdef DEBUG
int printk(const char *fmt, ...);
#define Dbg(...) do{printk(__VA_ARGS__);}while(0)
#else
#define Dbg(...) do{}while(0)
#endif
extern user_data_t userdata;
	
	
acc_t acc;  
	
static unsigned int accumulatedMeasurementCount;
static float accumulatedMeasurements[XYZ_AXIS_COUNT];

   
static flightDynamicsTrims_t __t, *accelerationTrims=&__t;
static uint16_t accLpfCutHz = 0;
static biquadFilter_t accFilter[XYZ_AXIS_COUNT];
static uint16_t calibratingA = 0; 

static void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
   rollAndPitchTrims->values.roll = 0;
   rollAndPitchTrims->values.pitch = 0;
  
}
static accelerometerConfig_t* accelerometerConfig()
{
	static accelerometerConfig_t instance;
	static char init = 0;
	if (init == 0) {
				init = 1;
        instance.acc_lpf_hz = 10;
        instance.acc_hardware = ACC_DEFAULT;
        instance.acc_high_fsr = false;
	}			
	return &instance;
}

void accInitFilters(void)
{
    accLpfCutHz = accelerometerConfig()->acc_lpf_hz;
    if (acc.accSamplingInterval) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accFilter[axis], accLpfCutHz, acc.accSamplingInterval);
        }
    }
}

bool accDetect(accDev_t *dev)
{
	bool mpu9250SpiAccDetect(accDev_t *acc);
	return mpu9250SpiAccDetect(dev);
}
bool accInit(uint32_t gyroSamplingInverval)
{
    memset(&acc, 0, sizeof(acc));
    // copy over the common gyro mpu settings
    //acc.dev.bus = *gyroSensorBus();
    //acc.dev.mpuDetectionResult = *gyroMpuDetectionResult();
    acc.dev.acc_high_fsr = accelerometerConfig()->acc_high_fsr;

    // Copy alignment from active gyro, as all production boards use acc-gyro-combi chip.
    // Exceptions are STM32F3DISCOVERY and STM32F411DISCOVERY, and (may be) handled in future enhancement.
#if 0
    sensor_align_e alignment = gyroDeviceConfig(0)->alignment;
    const sensorAlignment_t* customAlignment = &gyroDeviceConfig(0)->customAlignment;

#ifdef USE_MULTI_GYRO
    if (gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_2) {
        alignment = gyroDeviceConfig(1)->alignment;
    
        customAlignment = &gyroDeviceConfig(1)->customAlignment;
    }
#endif
    acc.dev.accAlign = alignment;
    buildRotationMatrixFromAlignment(customAlignment, &acc.dev.rotationMatrix);
#endif
    if (!accDetect(&acc.dev/*, accelerometerConfig()->acc_hardware*/)) {
        return false;
    }

		
    acc.dev.acc_1G = 256; // set default
    acc.dev.initFn(&acc.dev); // driver initialisation
    acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;
    // set the acc sampling interval according to the gyro sampling interval
    switch (gyroSamplingInverval) {  // Switch statement kept in place to change acc sampling interval in the future
    case 500:
    case 375:
    case 250:
    case 125:
        acc.accSamplingInterval = 1000;
        break;
    case 1000:
    default:
        acc.accSamplingInterval = 1000;
    }
		
		//acc.accSamplingInterval = 100; //add liren 
		
		accLpfCutHz = accelerometerConfig()->acc_lpf_hz;
		
    if (accLpfCutHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accFilter[axis], accLpfCutHz, acc.accSamplingInterval);
        }
    }
		
		//calibratingA = CALIBRATING_ACC_CYCLES;
		
		accelerationTrims->raw[X] = userdata.acc[0]; 
		accelerationTrims->raw[Y] = userdata.acc[1];
		accelerationTrims->raw[Z] = userdata.acc[2];
		
    return true;
}

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingA = calibrationCyclesRequired;
}

 bool accIsCalibrationComplete(void)
{
    return calibratingA == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

static void performAcclerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle()) {
					  if (axis==0){
							Dbg("a: Calibration begin\n");
						}
						a[axis] = 0;
        }

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += acc.accADC[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        acc.accADC[axis] = 0;
        accelerationTrims->raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc.dev.acc_1G;

        resetRollAndPitchTrims(rollAndPitchTrims);
				Dbg("a: Calibration end\n");
				extern user_data_t userdata;
			  userdata.acc[0] = accelerationTrims->raw[X];
			  userdata.acc[1] = accelerationTrims->raw[Y];
			  userdata.acc[2] = accelerationTrims->raw[Z];
        //saveConfigAndNotify();
				flash_write(FLASH_USER_START_ADDR, (uint8_t *)&userdata, sizeof(user_data_t));
    }

    calibratingA--;
}

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC[X] -= accelerationTrims->raw[X];
    acc.accADC[Y] -= accelerationTrims->raw[Y];
    acc.accADC[Z] -= accelerationTrims->raw[Z];
}

void accUpdate(timeUs_t currentTimeUs, rollAndPitchTrims_t *rollAndPitchTrims)
{
    UNUSED(currentTimeUs);

    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }
    //Dbg("araw:%d,%d,%d\n",acc.dev.ADCRaw[X],acc.dev.ADCRaw[Y],acc.dev.ADCRaw[Z]);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        //DEBUG_SET(DEBUG_ACCELEROMETER, axis, acc.dev.ADCRaw[axis]);
        acc.accADC[axis] = acc.dev.ADCRaw[axis];
    }

    if (accLpfCutHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            acc.accADC[axis] = biquadFilterApply(&accFilter[axis], acc.accADC[axis]);
        }
    }

    /*if (acc.dev.accAlign == ALIGN_CUSTOM) {
        alignSensorViaMatrix(acc.accADC, &acc.dev.rotationMatrix);
    } else {
        alignSensorViaRotation(acc.accADC, acc.dev.accAlign);
    }*/

    if (!accIsCalibrationComplete()) {
        performAcclerationCalibration(rollAndPitchTrims);
    }

    //if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
    //    performInflightAccelerationCalibration(rollAndPitchTrims);
    //}

    applyAccelerationTrims(accelerationTrims);
		//Dbg("at:%d,%d,%d\n",(int)acc.accADC[X],(int)acc.accADC[Y],(int)acc.accADC[Z]);
    ++accumulatedMeasurementCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        accumulatedMeasurements[axis] += acc.accADC[axis];
    }
		acc.isAccelUpdatedAtLeastOnce = true;
}

bool accGetAccumulationAverage(float *accumulationAverage)
{
    if (accumulatedMeasurementCount > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accumulatedMeasurements[axis] / accumulatedMeasurementCount;
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

