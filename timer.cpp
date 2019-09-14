/*
 * timer.cpp
 *
 * Created: 11.09.2019 11:16:19
 *  Author: Tobias
 */ 

#include "timer.h"

long ticks = 0;
uint8_t OVF = 0;
uint8_t cnt = 0;

timer::timer(){
	TIMSK0 |= (1 << TOIE0);
	TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00);
	time = 1000;
	start_time = 3;
	reset = true;
};

void timer::start_timer(void){
	if (reset){
		start_time = ticks/5;
		reset = false;
	}
};

void timer::set_time_out(uint16_t ms){
	time = ms;
};

bool timer::time_out(void){
	cli();
	long diff = (ticks/5) - start_time;
	if (diff < 0){
		diff = (ticks/5) + start_time;
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
	ticks++;
	cnt++;
	if (cnt == 255){
		OVF ++;
		cnt = 0;
		if (OVF == 5){
			ticks += 220;
			OVF = 0;
		}
	}
	sei();
};