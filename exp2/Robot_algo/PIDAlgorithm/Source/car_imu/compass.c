
#include <stdint.h>
#include <stdio.h>
#include "common/common.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "sensor.h"
#include "compass.h"

#define DEBUG
#undef  DEBUG
#ifdef DEBUG
int printk(const char *fmt, ...);
#define Dbg(...) do{printk(__VA_ARGS__);}while(0)
#else
#define Dbg(...) do{}while(0)
#endif
extern user_data_t userdata;

	
magDev_t magDev;
mag_t mag;

#define CALIBRATE_MAG	1
static int magCalibrate = 0;
static flightDynamicsTrims_t _mz, *magZero=&_mz;
static int magInit = 0;
static int16_t magADCRaw[XYZ_AXIS_COUNT];

	
//static int32_t magADCSum[XYZ_AXIS_COUNT];
bool ak8963Detect(magDev_t *mag);
static bool compassDetect(magDev_t *dev, uint8_t *alignment)
{
	return ak8963Detect(dev);
}
void compassStartCalibration(void)
{
	magCalibrate = 1;
}
bool isCompassCalibrationComplete(void)
{
	return magCalibrate == 0;
}

bool compassInit(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    // calculate magnetic declination
    mag.magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.

    uint8_t alignment;

    if (!compassDetect(&magDev, &alignment)) {
        return false;
    }
	
    const int16_t deg = 0; //compassConfig()->mag_declination / 100;
    const int16_t min = 0; //compassConfig()->mag_declination % 100;
    mag.magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
    //LED1_ON;
    magDev.init(&magDev);
    //LED1_OFF;
    magInit = 1;
	 /*
    magDev.magAlignment = alignment;

    if (compassConfig()->mag_alignment != ALIGN_DEFAULT) {
        magDev.magAlignment = compassConfig()->mag_alignment;
    }
		*/
    
    //buildRotationMatrixFromAlignment(&compassConfig()->mag_customAlignment, &magDev.rotationMatrix);

		magZero->raw[X] = userdata.mag[0]; 
		magZero->raw[Y] = userdata.mag[1];
		magZero->raw[Z] = userdata.mag[2];
						
    return true;
}
int compassIsHealthy(void)
{
	return (mag.magADC[X] != 0) && (mag.magADC[Y] != 0) && (mag.magADC[Z] != 0);
}

void compassUpdate(timeUs_t currentTimeUs)
{
    static timeUs_t tCal = 0;
    static flightDynamicsTrims_t magZeroTempMin;
    static flightDynamicsTrims_t magZeroTempMax;

    if(!magDev.read(&magDev, magADCRaw)) {
			//void beepLong(int n);
			//beepLong(1);
			return;
		}
	  //Dbg("%d,%d,%d\n",magADCRaw[X],magADCRaw[Y], magADCRaw[Z]);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {  
			mag.magADC[axis] = magADCRaw[axis];
			//mag.magADC[axis] = (mag.magADC[axis] + magADCRaw[axis]) / 2;
    }
   /* if (magDev.magAlignment == ALIGN_CUSTOM) {
        alignSensorViaMatrix(mag.magADC, &magDev.rotationMatrix);
    } else {
        alignSensorViaRotation(mag.magADC, magDev.magAlignment);
    }
	*/
    //flightDynamicsTrims_t *magZero = &compassConfigMutable()->magZero;
    if (magCalibrate == 1) {
				tCal = currentTimeUs;
        for (int axis = 0; axis < 3; axis++) {
            magZero->raw[axis] = 0;
            magZeroTempMin.raw[axis] = mag.magADC[axis];
            magZeroTempMax.raw[axis] = mag.magADC[axis];
        }
        //DISABLE_STATE(CALIBRATE_MAG);
				magCalibrate = 2;
				
    }

    if (magInit) {              // we apply offset only once mag calibration is done
        mag.magADC[X] -= magZero->raw[X];
        mag.magADC[Y] -= magZero->raw[Y];
        mag.magADC[Z] -= magZero->raw[Z];
    }

    if (tCal != 0) {
        if ((currentTimeUs - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            //LED0_TOGGLE;
            for (int axis = 0; axis < 3; axis++) {
                if (mag.magADC[axis] < magZeroTempMin.raw[axis])
                    magZeroTempMin.raw[axis] = mag.magADC[axis];
                if (mag.magADC[axis] > magZeroTempMax.raw[axis])
                    magZeroTempMax.raw[axis] = mag.magADC[axis];
            }
        } else {
            tCal = 0;
            for (int axis = 0; axis < 3; axis++) {
                magZero->raw[axis] = (magZeroTempMin.raw[axis] + magZeroTempMax.raw[axis]) / 2; // Calculate offsets
            }
						magCalibrate = 0;
            //saveConfigAndNotify();
						extern user_data_t userdata;
						userdata.mag[0] = magZero->raw[X];
						userdata.mag[1] = magZero->raw[Y];
						userdata.mag[2] = magZero->raw[Z];
            flash_write(FLASH_USER_START_ADDR, (uint8_t *)&userdata, sizeof(user_data_t));
        }
    }
}
