/*
 * RTC.cpp
 *
 * Created: 11.09.2019 11:16:19
 *  Author: Tobias
 */ 

#include "RTC.h"


uint32_t seconds = 0;
uint16_t millis = 0;
uint16_t micros = 0;
uint8_t OVF0 = 0;
uint8_t cnt0 = 0;
bool OVF2 = false;

uint32_t alarm_start = 0;
uint16_t alarm_period = 10000;
bool alarm = false;
bool alarm_active = false;

RTC::RTC(){
	/* initialize timer 0 */
	TIMSK0 |= (1 << TOIE0);
	TCCR0B |= (1 << CS00);
	TCCR0B &= ~((0 << CS02) | (0 << CS01));
	/*Initialize timer 2*/
	ASSR |= (1 << AS2);
	TCCR2A = 0;
	TIMSK2 |= 0b00000001;
	TCCR2B = 0b00000100;  
	time = 1000;
};

void RTC::set_time(uint64_t timestamp){
	cli();
	//Reset timer values:
	TCNT0 = 0;
	TCNT2 = 0;
	seconds = (uint32_t)(timestamp/1000000);
	millis = (uint16_t)((timestamp-((uint64_t)seconds*1000000))/1000);
	micros = (uint16_t)((timestamp-((uint64_t)seconds*1000000)-(millis*1000)));
	sei();
};

void RTC::set_epoch(uint32_t epoch){
	cli();
	//Reset timer values:
	TCNT0 = 0;
	TCNT2 = 0;
	sei();
	seconds = epoch;
	millis = 0;
	micros = 0;
};

uint32_t RTC::get_epoch(void){
	return seconds;
};

uint64_t RTC::get_timestamp(void){
	uint64_t timestamp = ((uint64_t)seconds*1000000) +((uint32_t)millis*1000)+(micros);
	return timestamp;
};
void RTC::set_alarm_period(uint16_t period){
	alarm_period = period;
};
void RTC::start_alarm(){
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		alarm_active = true;
	}
};

void RTC::stop_alarm(){
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		alarm_active = false;
	}
};

bool RTC::get_alarm_status(){
	if (alarm){
		ATOMIC_BLOCK(ATOMIC_FORCEON){
			alarm = false;
		}
		return true;
	}
	else{
		return false;
	}
};

ISR(TIMER0_OVF_vect){
	cli();
	OVF0 ++;
	if (OVF0 == 7){
		millis++;
		micros = 0;
		OVF0 = 0;
		cnt0++;
		if (cnt0 == 52){
			millis--;
			cnt0=0;
		}
	}
	else  {
		micros = micros + 139;
	}
	sei();
};

ISR(TIMER2_OVF_vect){
	cli();
	if (OVF2){
		seconds++;
		sei();
		ATOMIC_BLOCK(ATOMIC_FORCEON){
		millis = 0;
		micros = 0;
		OVF0 = 0;
		cnt0 = 0;
		OVF2 = false;
		}
		if (alarm_active){
			if((seconds - alarm_start) > alarm_period){
				sleep_disable();
				ATOMIC_BLOCK(ATOMIC_FORCEON){
					alarm_start = seconds;
					alarm = true;
				}
			}
		}
	}
	else {
		OVF2 = true;
		sei();
		}
};