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
	uint16_t time;
	uint8_t start_s;
	uint16_t start_ms;
	uint16_t start_us;
	bool reset;
};



#endif /* TIMER_H_ */