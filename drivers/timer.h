/* 
* timer.h
*
* Created: 22.12.2019 14:28:26
* Author: Tobias
*/


#ifndef __TIMER_H__
#define __TIMER_H__

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

enum TIMER {TIM1=1, TIM3=3, TIM4=4};

class Timer
{
//variables
public:
protected:
private:

//functions
public:
	void start();
	void stop();
	void reset();
	uint32_t read_ms();
	uint32_t read_us();
	Timer();
	~Timer();
protected:
private:
	TIMER TIMn;
	uint16_t temp_ticks;
	uint16_t temp_OVF;
	bool running;
	void store_current_values();
	
}; //timer

#endif //__TIMER_H__
