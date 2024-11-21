#ifndef __DEBUG_PROTOCOL_H_
#define __DEBUG_PROTOCOL_H_
#include "stm32f4xx.h"

uint8_t debug_protocol_parse(char *pdata, uint16_t len);

#endif
