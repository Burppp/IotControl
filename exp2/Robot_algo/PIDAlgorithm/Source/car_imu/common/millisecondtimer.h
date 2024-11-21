#ifdef __cplusplus
extern "C" {
#endif
#ifndef _MILLISECONDTIMER_H_
#define _MILLISECONDTIMER_H_
#include <stdint.h>

void initialise(void);
void delay(uint32_t millis_);
void delayus(uint32_t uillis);
uint32_t millis(void);
void reset(void);
void timerHandler(void);
#endif // _MILLISECONDTIMER_H_ 
#ifdef __cplusplus
}
#endif



