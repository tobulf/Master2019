/*
 * RTC.h
 *
 * Created: 11.09.2019 11:16:33
 *  Author: Tobias
 */ 


#ifndef RTC_H_
#define RTC_H_

#include <avr/interrupt.h> 
#include <stdbool.h>
#include "util/delay.h"

extern "C" {
	#include "Debug.h"
};

class RTC {
	public:
	RTC();
	void set_time(uint64_t epoch);
	uint32_t get_epoch();
	uint64_t get_timestamp();
	protected:
	
	private:
	uint16_t time;
};



#endif /* RTC_H_ */