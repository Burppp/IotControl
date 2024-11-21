#ifdef __cplusplus
extern "C" {
#endif

#include "common/millisecondtimer.h"
#include "common/ms_timer.h"
	
volatile uint32_t _counter;

void initialise(void) 
{
	_counter = 0;
	ms_Timer_Init();
}

void delay(uint32_t millis) 
{
	uint32_t target;
	
	target = _counter + millis;
	while(_counter < target);
}
uint32_t micros(void)
{
	return _counter*1000 + ms_Timer_GetCount();
}
void delayus(uint32_t uillis)
{ 
	uint32_t target;
	target = micros() + uillis;
	while(micros() <= target);
}

void timerHandler(void)
{
	_counter++;
}

uint32_t millis(void) 
{
	return _counter;
}

void reset(void) 
{
	_counter = 0;
}

#ifdef __cplusplus
}
#endif
