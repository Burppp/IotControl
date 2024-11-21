#include <stdint.h>
#include <stdio.h>
#include "common/common.h"
#include "common/axis.h"
#include "gyro.h"
#include "common/filter.h"
#include "common/maths.h"
#include "accgyro.h"
#include "gyro_sync.h"

#define UNUSED(x) (x) 
#define DEBUG_NONE 0

#define DEBUG
#undef DEBUG
#ifdef DEBUG
int printk(const char *fmt, ...);
#define Dbg(...) do{printk(__VA_ARGS__);}while(0)
#else
#define Dbg(...) do{}while(0)
#endif


typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
} gyroLowpassFilter_t;

typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    int32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct gyroSensor_s {
    gyroDev_t gyroDev;
    gyroCalibration_t calibration;

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilter2ApplyFn;
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilterDynApplyFn;
    filterApplyFnPtr notchFilterDynApplyFn2;
    biquadFilter_t notchFilterDyn[XYZ_AXIS_COUNT];
    biquadFilter_t notchFilterDyn2[XYZ_AXIS_COUNT];

    // overflow and recovery
    timeUs_t overflowTimeUs;
    bool overflowDetected;
#ifdef USE_YAW_SPIN_RECOVERY
    timeUs_t yawSpinTimeUs;
    bool yawSpinDetected;
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_GYRO_DATA_ANALYSE
#define DYNAMIC_NOTCH_DEFAULT_CENTER_HZ 350
#define DYNAMIC_NOTCH_DEFAULT_CUTOFF_HZ 300
    gyroAnalyseState_t gyroAnalyseState;
#endif

    flight_dynamics_index_t gyroDebugAxis;
} gyroSensor_t;

gyro_t gyro;

static unsigned int accumulatedMeasurementTimeUs;
static float accumulatedMeasurements[XYZ_AXIS_COUNT];
static  bool overflowDetected;
static int gyroToUse = GYRO_CONFIG_USE_GYRO_1;

static timeUs_t accumulationLastTimeSampledUs;
static  gyroSensor_t gyroSensor1;
static float gyroPrevious[XYZ_AXIS_COUNT];

static  uint8_t gyroDebugMode = DEBUG_NONE;

bool firstArmingCalibrationWasStarted = false;

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor);

#ifdef STM32F10X
#define GYRO_SYNC_DENOM_DEFAULT 8
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20649) \
   || defined(USE_GYRO_SPI_ICM20689)
#define GYRO_SYNC_DENOM_DEFAULT 1
#else
#define GYRO_SYNC_DENOM_DEFAULT 3
#endif
/*
void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
	gyroSetCalibrationCycles(&gyroSensor1);
}*/
static  gyroConfig_t* gyroConfig()
{
	static gyroConfig_t _t, *gyroConfig=&_t;
	static char init = 0;
	if (init == 0) {
		init = 1;
	  gyroConfig->gyroCalibrationDuration = 125;        // 1.25 seconds
    gyroConfig->gyroMovementCalibrationThreshold = 48;
    gyroConfig->gyro_sync_denom = GYRO_SYNC_DENOM_DEFAULT;
    gyroConfig->gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
    gyroConfig->gyro_lowpass_type = FILTER_PT1;
    gyroConfig->gyro_lowpass_hz = 150;  // NOTE: dynamic lpf is enabled by default so this setting is actually
                                        // overridden and the static lowpass 1 is disabled. We can't set this
                                        // value to 0 otherwise Configurator versions 10.4 and earlier will also
                                        // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
    gyroConfig->gyro_lowpass2_type = FILTER_PT1;
    gyroConfig->gyro_lowpass2_hz = 250;
    gyroConfig->gyro_high_fsr = false;
    gyroConfig->gyro_to_use = GYRO_CONFIG_USE_GYRO_DEFAULT;
    gyroConfig->gyro_soft_notch_hz_1 = 0;
    gyroConfig->gyro_soft_notch_cutoff_1 = 0;
    gyroConfig->gyro_soft_notch_hz_2 = 0;
    gyroConfig->gyro_soft_notch_cutoff_2 = 0;
    gyroConfig->checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES;
    gyroConfig->gyro_offset_yaw = 0;
    gyroConfig->yaw_spin_recovery = true;
    gyroConfig->yaw_spin_threshold = 1950;
    gyroConfig->dyn_lpf_gyro_min_hz = 200;
    gyroConfig->dyn_lpf_gyro_max_hz = 500;
    gyroConfig->dyn_notch_range = DYN_NOTCH_RANGE_AUTO;
    gyroConfig->dyn_notch_width_percent = 8;
    gyroConfig->dyn_notch_q = 120;
    gyroConfig->dyn_notch_min_hz = 150;
    gyroConfig->gyro_filter_debug_axis = FD_ROLL;
	}
	return gyroConfig; 
}



static void gyroInitSensor(gyroSensor_t *gyroSensor/*, const gyroDeviceConfig_t *config*/)
{
    gyroSensor->gyroDebugAxis = gyroConfig()->gyro_filter_debug_axis;
    gyroSensor->gyroDev.gyro_high_fsr = gyroConfig()->gyro_high_fsr;
    //gyroSensor->gyroDev.gyroAlign = config->alignment;
    //buildRotationMatrixFromAlignment(&config->customAlignment, &gyroSensor->gyroDev.rotationMatrix);
    //gyroSensor->gyroDev.mpuIntExtiTag = config->extiTag;

    // Must set gyro targetLooptime before gyroDev.init and initialisation of filters
    gyro.targetLooptime = gyroSetSampleRate(&gyroSensor->gyroDev, gyroConfig()->gyro_hardware_lpf, gyroConfig()->gyro_sync_denom);
    gyroSensor->gyroDev.hardware_lpf = gyroConfig()->gyro_hardware_lpf;
    
	  gyroSensor->gyroDev.initFn(&gyroSensor->gyroDev);

    // As new gyros are supported, be sure to add them below based on whether they are subject to the overflow/inversion bug
    // Any gyro not explicitly defined will default to not having built-in overflow protection as a safe alternative.

    gyroSensor->gyroDev.gyroHasOverflowProtection = true;


    gyroInitSensorFilters(gyroSensor);

#ifdef USE_GYRO_DATA_ANALYSE
    gyroDataAnalyseStateInit(&gyroSensor->gyroAnalyseState, gyro.targetLooptime);
    
#endif
}

static bool gyroDetectSensor(gyroSensor_t *gyroSensor)
{
	bool mpu9250SpiGyroDetect(gyroDev_t *gyro);
	return mpu9250SpiGyroDetect(&gyroSensor->gyroDev);
}


void gyroInitLowpassFilterLpf(gyroSensor_t *gyroSensor, int slot, int type, uint16_t lpfHz)
{
    filterApplyFnPtr *lowpassFilterApplyFn;
    gyroLowpassFilter_t *lowpassFilter = NULL;

    switch (slot) {
    case FILTER_LOWPASS:
        lowpassFilterApplyFn = &gyroSensor->lowpassFilterApplyFn;
        lowpassFilter = gyroSensor->lowpassFilter;
        break;

    case FILTER_LOWPASS2:
        lowpassFilterApplyFn = &gyroSensor->lowpass2FilterApplyFn;
        lowpassFilter = gyroSensor->lowpass2Filter;
        break;

    default:
        return;
    }

    // Establish some common constants
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;
    const float gyroDt = gyro.targetLooptime * 1e-6f;

    // Gain could be calculated a little later as it is specific to the pt1/bqrcf2/fkf branches
    const float gain = pt1FilterGain(lpfHz, gyroDt);

    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn = nullFilterApply;

    // If lowpass cutoff has been specified and is less than the Nyquist frequency
    if (lpfHz && lpfHz <= gyroFrequencyNyquist) {
        switch (type) {
        case FILTER_PT1:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
            }
            break;
        case FILTER_BIQUAD:
#ifdef USE_DYN_LPF
            *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApplyDF1;
#else
            *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApply;
#endif
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterInitLPF(&lowpassFilter[axis].biquadFilterState, lpfHz, gyro.targetLooptime);
            }
            break;
        }
    }
}

static uint16_t calculateNyquistAdjustedNotchHz(uint16_t notchHz, uint16_t notchCutoffHz)
{
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;
    if (notchHz > gyroFrequencyNyquist) {
        if (notchCutoffHz < gyroFrequencyNyquist) {
            notchHz = gyroFrequencyNyquist;
        } else {
            notchHz = 0;
        }
    }

    return notchHz;
}


static void gyroInitFilterNotch1(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyroSensor->notchFilter1ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyroSensor->notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilter1[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

static void gyroInitFilterNotch2(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyroSensor->notchFilter2ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyroSensor->notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilter2[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor)
{
#if defined(USE_GYRO_SLEW_LIMITER)
    gyroInitSlewLimiter(gyroSensor);
#endif

    uint16_t gyro_lowpass_hz = gyroConfig()->gyro_lowpass_hz;

#ifdef USE_DYN_LPF
    if (gyroConfig()->dyn_lpf_gyro_min_hz > 0) {
        gyro_lowpass_hz = gyroConfig()->dyn_lpf_gyro_min_hz;
    }
#endif

    gyroInitLowpassFilterLpf(
      gyroSensor,
      FILTER_LOWPASS,
      gyroConfig()->gyro_lowpass_type,
      gyro_lowpass_hz
    );

    gyroInitLowpassFilterLpf(
      gyroSensor,
      FILTER_LOWPASS2,
      gyroConfig()->gyro_lowpass2_type,
      gyroConfig()->gyro_lowpass2_hz
    );

    gyroInitFilterNotch1(gyroSensor, gyroConfig()->gyro_soft_notch_hz_1, gyroConfig()->gyro_soft_notch_cutoff_1);
    gyroInitFilterNotch2(gyroSensor, gyroConfig()->gyro_soft_notch_hz_2, gyroConfig()->gyro_soft_notch_cutoff_2);
#ifdef USE_GYRO_DATA_ANALYSE
    gyroInitFilterDynamicNotch(gyroSensor);
#endif
#ifdef USE_DYN_LPF
    dynLpfFilterInit();
#endif
}

static bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor)
{
    return gyroSensor->calibration.cyclesRemaining == 0;
}

 bool isGyroCalibrationComplete(void)
{
    switch (gyroToUse) {
        case GYRO_CONFIG_USE_GYRO_1: {
            return isGyroSensorCalibrationComplete(&gyroSensor1);
        }
#ifdef USE_MULTI_GYRO
        case GYRO_CONFIG_USE_GYRO_2: {
            return isGyroSensorCalibrationComplete(&gyroSensor2);
        }
        case GYRO_CONFIG_USE_GYRO_BOTH: {
            return isGyroSensorCalibrationComplete(&gyroSensor1) && isGyroSensorCalibrationComplete(&gyroSensor2);
        }
#endif
    }
}

static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 1;
}

static int32_t gyroCalculateCalibratingCycles(void)
{
    return (gyroConfig()->gyroCalibrationDuration * 10000) / gyro.targetLooptime;
}

static bool isOnFirstGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == gyroCalculateCalibratingCycles();
}

static void gyroSetCalibrationCycles(gyroSensor_t *gyroSensor)
{
#if defined(USE_FAKE_GYRO) && !defined(UNIT_TEST)
    if (gyroSensor->gyroDev.gyroHardware == GYRO_FAKE) {
        gyroSensor->calibration.cyclesRemaining = 0;
        return;
    }
#endif
    gyroSensor->calibration.cyclesRemaining = gyroCalculateCalibratingCycles();
}

void gyroStartCalibration(bool isFirstArmingCalibration)
{
    if (!(isFirstArmingCalibration && firstArmingCalibrationWasStarted)) {
        gyroSetCalibrationCycles(&gyroSensor1);
#ifdef USE_MULTI_GYRO
        gyroSetCalibrationCycles(&gyroSensor2);
#endif

        if (isFirstArmingCalibration) {
            firstArmingCalibrationWasStarted = true;
        }
    }
}

bool isFirstArmingGyroCalibrationRunning(void)
{
    return firstArmingCalibrationWasStarted && !isGyroCalibrationComplete();
}
static void performGyroCalibration(gyroSensor_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
					if (axis==0)Dbg("g: Calibration begin\n");
            gyroSensor->calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyroDev.gyroZero[axis] = 0.0f;
					
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->calibration.sum[axis] += gyroSensor->gyroDev.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], gyroSensor->gyroDev.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW
            //if (axis == X) {
            //    DEBUG_SET(DEBUG_GYRO_RAW, DEBUG_GYRO_CALIBRATION, lrintf(stddev));
            //}
							//Dbg("stddev %f\n", stddev);
            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(gyroSensor);
                return;
            }

            // please take care with exotic boardalignment !!
            gyroSensor->gyroDev.gyroZero[axis] = gyroSensor->calibration.sum[axis] / gyroCalculateCalibratingCycles();
            if (axis == Z) {
              gyroSensor->gyroDev.gyroZero[axis] -= ((float)gyroConfig()->gyro_offset_yaw / 100);
            }
        }
    }
    if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
        //schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
        //if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags() & ~ARMING_DISABLED_CALIBRATING) == 0) {
            //beeper(BEEPER_GYRO_CALIBRATED);
        //}
			Dbg("g: Calibration end %d,%d,%d\n",(int)gyroSensor->gyroDev.gyroZero[X],(int)gyroSensor->gyroDev.gyroZero[Y],(int)gyroSensor->gyroDev.gyroZero[Z]);
    }
    --gyroSensor->calibration.cyclesRemaining;
}

#define GYRO_FILTER_FUNCTION_NAME filterGyro
#define GYRO_FILTER_DEBUG_SET(mode, index, value) { UNUSED(mode); UNUSED(index); UNUSED(value); }
#include "gyro_filter_impl.c"
#undef GYRO_FILTER_FUNCTION_NAME
#undef GYRO_FILTER_DEBUG_SET

#define GYRO_FILTER_FUNCTION_NAME filterGyroDebug
#define GYRO_FILTER_DEBUG_SET DEBUG_SET
#include "gyro_filter_impl.c"
#undef GYRO_FILTER_FUNCTION_NAME
#undef GYRO_FILTER_DEBUG_SET



static  void gyroUpdateSensor(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    if (!gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev)) {
        return;
    }
		//Dbg("g:%d,%d,%d\n", gyroSensor->gyroDev.gyroADCRaw[X],gyroSensor->gyroDev.gyroADCRaw[Y],gyroSensor->gyroDev.gyroADCRaw[Z]);
    gyroSensor->gyroDev.dataReady = false;

    if (isGyroSensorCalibrationComplete(gyroSensor)) {
        // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations

#if defined(USE_GYRO_SLEW_LIMITER)
        gyroSensor->gyroDev.gyroADC[X] = gyroSlewLimiter(gyroSensor, X) - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSlewLimiter(gyroSensor, Y) - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSlewLimiter(gyroSensor, Z) - gyroSensor->gyroDev.gyroZero[Z];
#else
        gyroSensor->gyroDev.gyroADC[X] = gyroSensor->gyroDev.gyroADCRaw[X] - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSensor->gyroDev.gyroADCRaw[Y] - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSensor->gyroDev.gyroADCRaw[Z] - gyroSensor->gyroDev.gyroZero[Z];
#endif
			//Dbg("g: %.1f, %.1f, %.1f\n",gyroSensor->gyroDev.gyroADC[X],gyroSensor->gyroDev.gyroADC[Y],gyroSensor->gyroDev.gyroADC[Z]);
   /*     if (gyroSensor->gyroDev.gyroAlign == ALIGN_CUSTOM) {
            alignSensorViaMatrix(gyroSensor->gyroDev.gyroADC, &gyroSensor->gyroDev.rotationMatrix);
        } else {
            alignSensorViaRotation(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
        }
			*/
    } else {
        performGyroCalibration(gyroSensor, gyroConfig()->gyroMovementCalibrationThreshold);
        // still calibrating, so no need to further process gyro data
        return;
    }

    if (gyroDebugMode == DEBUG_NONE) {
        filterGyro(gyroSensor);
			
			//Dbg("gf: %d, %d, %d\n",(int)gyroSensor->gyroDev.gyroADCf[X],(int)gyroSensor->gyroDev.gyroADCf[Y],(int)gyroSensor->gyroDev.gyroADCf[Z]);
    } else {
        filterGyroDebug(gyroSensor);
    }

#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow && !gyroHasOverflowProtection) {
        checkForOverflow(gyroSensor, currentTimeUs);
    }
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    if (gyroConfig()->yaw_spin_recovery) {
        checkForYawSpin(gyroSensor, currentTimeUs);
    }
#endif

#ifdef USE_GYRO_DATA_ANALYSE
    if (isDynamicFilterActive()) {
        gyroDataAnalyse(&gyroSensor->gyroAnalyseState, gyroSensor->notchFilterDyn, gyroSensor->notchFilterDyn2);
    }
#endif

#if (!defined(USE_GYRO_OVERFLOW_CHECK) && !defined(USE_YAW_SPIN_RECOVERY))
    UNUSED(currentTimeUs);
#endif
}

bool gyroInit(void)
{
#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow == GYRO_OVERFLOW_CHECK_YAW) {
        overflowAxisMask = GYRO_OVERFLOW_Z;
    } else if (gyroConfig()->checkOverflow == GYRO_OVERFLOW_CHECK_ALL_AXES) {
        overflowAxisMask = GYRO_OVERFLOW_X | GYRO_OVERFLOW_Y | GYRO_OVERFLOW_Z;
    } else {
        overflowAxisMask = 0;
    }
#endif

    gyroDebugMode = DEBUG_NONE;
    //useDualGyroDebugging = false;


    firstArmingCalibrationWasStarted = false;

    //gyroDetectionFlags = NO_GYROS_DETECTED;

    gyroToUse = gyroConfig()->gyro_to_use;

		gyroDetectSensor(&gyroSensor1);
		
    //if (gyroDetectSensor(&gyroSensor1, gyroDeviceConfig(0))) {
    //    gyroDetectionFlags |= DETECTED_GYRO_1;
   // }

#if defined(USE_MULTI_GYRO)
    if (gyroDetectSensor(&gyroSensor2, gyroDeviceConfig(1))) {
        gyroDetectionFlags |= DETECTED_GYRO_2;
    }
#endif

    //if (gyroDetectionFlags == NO_GYROS_DETECTED) {
    //    return false;
    //}

#if defined(USE_MULTI_GYRO)
    if ((gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH && !((gyroDetectionFlags & DETECTED_BOTH_GYROS) == DETECTED_BOTH_GYROS))
        || (gyroToUse == GYRO_CONFIG_USE_GYRO_1 && !(gyroDetectionFlags & DETECTED_GYRO_1))
        || (gyroToUse == GYRO_CONFIG_USE_GYRO_2 && !(gyroDetectionFlags & DETECTED_GYRO_2))) {
        if (gyroDetectionFlags & DETECTED_GYRO_1) {
            gyroToUse = GYRO_CONFIG_USE_GYRO_1;
        } else {
            gyroToUse = GYRO_CONFIG_USE_GYRO_2;
        }

        gyroConfigMutable()->gyro_to_use = gyroToUse;
    }

    // Only allow using both gyros simultaneously if they are the same hardware type.
    if (((gyroDetectionFlags & DETECTED_BOTH_GYROS) == DETECTED_BOTH_GYROS) && gyroSensor1.gyroDev.gyroHardware == gyroSensor2.gyroDev.gyroHardware) {
        gyroDetectionFlags |= DETECTED_DUAL_GYROS;
    } else if (gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        // If the user selected "BOTH" and they are not the same type, then reset to using only the first gyro.
        gyroToUse = GYRO_CONFIG_USE_GYRO_1;
        gyroConfigMutable()->gyro_to_use = gyroToUse;
    }

    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        gyroInitSensor(&gyroSensor2, gyroDeviceConfig(1));
        gyroHasOverflowProtection =  gyroHasOverflowProtection && gyroSensor2.gyroDev.gyroHasOverflowProtection;
        detectedSensors[SENSOR_INDEX_GYRO] = gyroSensor2.gyroDev.gyroHardware;
    }
#endif

    if (gyroToUse == GYRO_CONFIG_USE_GYRO_1 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        gyroInitSensor(&gyroSensor1/*, gyroDeviceConfig(0)*/);
        //gyroHasOverflowProtection =  gyroHasOverflowProtection && gyroSensor1.gyroDev.gyroHasOverflowProtection;
        //detectedSensors[SENSOR_INDEX_GYRO] = gyroSensor1.gyroDev.gyroHardware;
    }

		//gyroSetCalibrationCycles(&gyroSensor1);
		
    return true;
}

void gyroUpdate(timeUs_t currentTimeUs)
{
    const timeDelta_t sampleDeltaUs = currentTimeUs - accumulationLastTimeSampledUs;
    accumulationLastTimeSampledUs = currentTimeUs;
    accumulatedMeasurementTimeUs += sampleDeltaUs;

    switch (gyroToUse) {
    case GYRO_CONFIG_USE_GYRO_1:
        gyroUpdateSensor(&gyroSensor1, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor1)) {
            gyro.gyroADCf[X] = gyroSensor1.gyroDev.gyroADCf[X];
            gyro.gyroADCf[Y] = gyroSensor1.gyroDev.gyroADCf[Y];
            gyro.gyroADCf[Z] = gyroSensor1.gyroDev.gyroADCf[Z];
#ifdef USE_GYRO_OVERFLOW_CHECK
            overflowDetected = gyroSensor1.overflowDetected;
#endif
#ifdef USE_YAW_SPIN_RECOVERY
            yawSpinDetected = gyroSensor1.yawSpinDetected;
#endif
        }
        /*if (useDualGyroDebugging) {
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyroSensor1.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyroSensor1.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X]));
            DEBUG_SET(DEBUG_DUAL_GYRO, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y]));
            DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 0, lrintf(gyro.gyroADCf[X]));
            DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 1, lrintf(gyro.gyroADCf[Y]));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 0, lrintf(gyroSensor1.gyroDev.gyroADC[X] * gyroSensor1.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 1, lrintf(gyroSensor1.gyroDev.gyroADC[Y] * gyroSensor1.gyroDev.scale));
        }*/
        break;
#ifdef USE_MULTI_GYRO
    case GYRO_CONFIG_USE_GYRO_2:
        gyroUpdateSensor(&gyroSensor2, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor2)) {
            gyro.gyroADCf[X] = gyroSensor2.gyroDev.gyroADCf[X];
            gyro.gyroADCf[Y] = gyroSensor2.gyroDev.gyroADCf[Y];
            gyro.gyroADCf[Z] = gyroSensor2.gyroDev.gyroADCf[Z];
#ifdef USE_GYRO_OVERFLOW_CHECK
            overflowDetected = gyroSensor2.overflowDetected;
#endif
#ifdef USE_YAW_SPIN_RECOVERY
            yawSpinDetected = gyroSensor2.yawSpinDetected;
#endif
        }
        /*if (useDualGyroDebugging) {
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyroSensor2.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyroSensor2.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO, 2, lrintf(gyroSensor2.gyroDev.gyroADCf[X]));
            DEBUG_SET(DEBUG_DUAL_GYRO, 3, lrintf(gyroSensor2.gyroDev.gyroADCf[Y]));
            DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 2, lrintf(gyro.gyroADCf[X]));
            DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 3, lrintf(gyro.gyroADCf[Y]));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 2, lrintf(gyroSensor2.gyroDev.gyroADC[X] * gyroSensor2.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 3, lrintf(gyroSensor2.gyroDev.gyroADC[Y] * gyroSensor2.gyroDev.scale));
        }*/
        break;
    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroUpdateSensor(&gyroSensor1, currentTimeUs);
        gyroUpdateSensor(&gyroSensor2, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor1) && isGyroSensorCalibrationComplete(&gyroSensor2)) {
            gyro.gyroADCf[X] = (gyroSensor1.gyroDev.gyroADCf[X] + gyroSensor2.gyroDev.gyroADCf[X]) / 2.0f;
            gyro.gyroADCf[Y] = (gyroSensor1.gyroDev.gyroADCf[Y] + gyroSensor2.gyroDev.gyroADCf[Y]) / 2.0f;
            gyro.gyroADCf[Z] = (gyroSensor1.gyroDev.gyroADCf[Z] + gyroSensor2.gyroDev.gyroADCf[Z]) / 2.0f;
#ifdef USE_GYRO_OVERFLOW_CHECK
            overflowDetected = gyroSensor1.overflowDetected || gyroSensor2.overflowDetected;
#endif
#ifdef USE_YAW_SPIN_RECOVERY
            yawSpinDetected = gyroSensor1.yawSpinDetected || gyroSensor2.yawSpinDetected;
#endif
        }

        /*if (useDualGyroDebugging) {
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyroSensor1.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyroSensor1.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X]));
            DEBUG_SET(DEBUG_DUAL_GYRO, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y]));
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyroSensor2.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyroSensor2.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO, 2, lrintf(gyroSensor2.gyroDev.gyroADCf[X]));
            DEBUG_SET(DEBUG_DUAL_GYRO, 3, lrintf(gyroSensor2.gyroDev.gyroADCf[Y]));
            DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 1, lrintf(gyro.gyroADCf[X]));
            DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 2, lrintf(gyro.gyroADCf[Y]));
            DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X] - gyroSensor2.gyroDev.gyroADCf[X]));
            DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y] - gyroSensor2.gyroDev.gyroADCf[Y]));
            DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 2, lrintf(gyroSensor1.gyroDev.gyroADCf[Z] - gyroSensor2.gyroDev.gyroADCf[Z]));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 0, lrintf(gyroSensor1.gyroDev.gyroADC[X] * gyroSensor1.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 1, lrintf(gyroSensor1.gyroDev.gyroADC[Y] * gyroSensor1.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 2, lrintf(gyroSensor2.gyroDev.gyroADC[X] * gyroSensor2.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 3, lrintf(gyroSensor2.gyroDev.gyroADC[Y] * gyroSensor2.gyroDev.scale));
        }*/
        break;
#endif
    }

    if (!overflowDetected) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            // integrate using trapezium rule to avoid bias
            accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyro.gyroADCf[axis]) * sampleDeltaUs;
            gyroPrevious[axis] = gyro.gyroADCf[axis];
        }
    }

}


bool gyroGetAccumulationAverage(float *accumulationAverage)
{
    if (accumulatedMeasurementTimeUs > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementTimeUs = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}
