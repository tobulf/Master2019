/*
 * timer.h
 *
 * Created: 11.09.2019 11:16:33
 *  Author: Tobias
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <avr/interrupt.h> 
#include <stdbool.h>
#include "util/delay.h"

extern "C" {
	#include "Debug.h"
};

class timer {
	public:
	timer();
	void start_timer(void);
	void reset_timer(void);
	void set_time_out(uint16_t ms);
	bool time_out(void);
	protected:
	
	private:
	long time;
	long start_time;
	bool reset;
};



#endif /* TIMER_H_ */