#ifndef __COMPASS_H__
#define __COMPASS_H__

#include "common/axis.h"
#include "sensor.h"

typedef struct mag_s {
    float magADC[XYZ_AXIS_COUNT];
    float magneticDeclination;
} mag_t;

extern mag_t mag;
typedef void * busDevice_t;
typedef struct magDev_s {
    sensorMagInitFuncPtr init;                              // initialize function
    sensorMagReadFuncPtr read;                              // read 3 axis data function
    //extiCallbackRec_t exti;
    busDevice_t busdev;
    //sensor_align_e magAlignment;
    //fp_rotationMatrix_t rotationMatrix;
    //ioTag_t magIntExtiTag;
    int16_t magGain[3];
} magDev_t;

int compassIsHealthy(void);
#endif
