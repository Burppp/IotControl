
#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <stdbool.h>
#include "common/millisecondtimer.h"
#include "common/ms_timer.h"
#include "Flash/flash.h"

typedef unsigned int timeUs_t;

typedef unsigned int timeDelta_t;

typedef struct user_data{ 
	int16_t  acc[3];
	int16_t  mag[3];
}user_data_t;

#endif
