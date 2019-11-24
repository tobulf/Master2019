/*
 * timer.cpp
 *
 * Created: 11.09.2019 11:16:19
 *  Author: Tobias
 */ 

#include "timer.h"

long millis = 0;
uint8_t OVF = 0;
uint8_t cnt = 0;

timer::timer(){
	TIMSK0 |= (1 << TOIE0);
	TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00);
	time = 1000;
	start_time = 0;
	reset = true;
};

void timer::start_timer(void){
	if (reset){
		start_time = millis;
		reset = false;
	}
};

void timer::set_time_out(uint16_t ms){
	time = ms;
};

bool timer::time_out(void){
	cli();
	long diff = millis - start_time;
	if (diff < 0){
		diff = millis + start_time;
		}
	sei();
	if(diff >= time){
		reset_timer();
		start_timer();
		return true;
	}
	else {
		return false;
	}
};

void timer::reset_timer(void){
	reset = true;
}

ISR(TIMER0_OVF_vect){
	cli();
	OVF ++;
	if (OVF == 7){
		millis++;
		OVF = 0;
		cnt++;
		if (cnt == 36){
			millis++;
			cnt=0;
		}
	}
	sei();
};