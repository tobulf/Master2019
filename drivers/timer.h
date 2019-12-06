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
/**
 * @brief Timer class
 * 
 */


class timer {
	public:
	/**
	 * @brief Construct a new timer object
	 * 
	 */
	timer();
	/**
	 * @brief start the timer: 1000ms default if not set.
	 * 
	 */
	void start_timer(void);
	/**
	 * @brief reset the timer.
	 * 
	 */
	void reset_timer(void);
	/**
	 * @brief Set the time-out for the timer
	 * 
	 * @param ms 
	 */
	void set_time_out(uint16_t ms);
	/**
	 * @brief polling function to check if timer has timed out.
	 * 
	 * @return true 
	 * @return false 
	 */
	bool time_out(void);
	protected:
	
	private:
	long time;
	long start_time;
	bool reset;
};



#endif /* TIMER_H_ */