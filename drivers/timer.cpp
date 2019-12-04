/*
 * timer.cpp
 *
 * Created: 11.09.2019 11:16:19
 *  Author: Tobias
 */ 

#include "timer.h"


uint32_t seconds = 0;
uint16_t millis = 0;
uint16_t micros = 0;
uint8_t OVF = 0;
uint8_t cnt = 0;
uint16_t OVF_tot = 0;

timer::timer(){
	TIMSK0 |= (1 << TOIE0);
	TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00);
	time = 1000;
	start_ms = 0;
	reset = true;
};

void timer::start_timer(void){
	if (reset){
		start_ms = millis;
		reset = false;
	}
};

void timer::set_time_out(uint16_t ms){
	time = ms;
};

bool timer::time_out(void){
	cli();
	uint16_t diff = millis - start_ms;
	if (diff < 0){
		diff = millis + start_ms;
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
	OVF_tot++;
	if (OVF == 7){
		micros = micros + 139;
		micros = 0;
		millis++;
		OVF = 0;
		cnt++;
		if (cnt == 36){
			millis++;
			cnt=0;
		}
	}
	else if (OVF_tot == 7200){
		seconds++;
		millis = 0;
		micros = 0;
		OVF = 0;
		cnt = 0;
		OVF_tot = 0;
	}
	else  {
		micros = micros + 139;
	}
	
	sei();
};