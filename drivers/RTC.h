/*
 * RTC.h
 *
 * Created: 11.09.2019 11:16:33
 *  Author: Tobias
 */ 


#ifndef RTC_H_
#define RTC_H_

#include <avr/interrupt.h>
#include <util/atomic.h> 
#include <stdbool.h>
#include <avr/sleep.h>

extern "C" {
	#include "Debug.h"
};

class RTC {
	public:
	RTC();
	void set_epoch(uint32_t epoch);
	void set_time(uint64_t timestamp);
	uint32_t get_epoch();
	uint64_t get_timestamp();
	void set_alarm_period(uint16_t period);
	void start_alarm();
	void stop_alarm();
	bool get_alarm_status();
	protected:
	
	private:
	uint16_t time;
};



#endif /* RTC_H_ */