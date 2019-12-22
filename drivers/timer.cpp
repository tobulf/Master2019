/* 
* timer.cpp
*
* Created: 22.12.2019 14:28:26
* Author: Tobias
*/


#include "timer.h"

uint8_t instances = 0;
uint16_t OVF1 = 0;
uint16_t OVF3 = 0;
uint16_t OVF4 = 0;

Timer::Timer(){
	instances ++;
	// Enable overflow interrupt and save instance(max 3 objects):
	switch (instances){
	case 1:
		TIMn = TIM1;
		TIMSK1 |= (1 << TOIE1);
		break;
	case 2:
		TIMn = TIM3;
		TIMSK3 |= (1 << TOIE3);
		break;
	case 3:
		TIMn = TIM4;
		TIMSK4 |= (1 << TOIE4);
		break;
	default:
		break;
	}
	running = false;
} 


Timer::~Timer(){
	instances--;
	switch (TIMn){
		case TIM1:
			//stop timer:
			cli();
			TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
			sei();
			TIMSK1 &= ~(1 << TOIE1);
			//Clear counter registers:
			TCNT1 = 0;
			OVF1 = 0;
			break;
		case TIM3:
			cli();
			TCCR3B &= ~((1<<CS32) | (1<<CS31) | (1<<CS30));
			sei();
			TIMSK3 &= ~(1 << TOIE3);
			TCNT3 = 0;

			OVF3 = 0;
			break;
		case TIM4:
			cli();
			TCCR4B &= ~((1<<CS42) | (1<<CS41) | (1<<CS40));
			sei();
			TIMSK4 &= ~(1 << TOIE4);
			TCNT4 = 0;
			OVF4 = 0;
			break;
		default:
			break;
	}

}

void Timer::start(void){
	switch (TIMn){
		case TIM1:
			cli();
			TCCR1B &= ((1<<CS12) | (1<<CS11));
			TCCR1B |= (1<<CS10);
			sei();
			break;
		case TIM3:
			TCCR3B &= ((1<<CS32) | (1<<CS32));
			TCCR3B |= (1<<CS30);
			break;
		case TIM4:
			TCCR4B &= ((1<<CS42) | (1<<CS41));
			TCCR4B |= (1<<CS40);
			break;
		default:
			break;
		}
		running = true;
};

void Timer::stop(){
	//Stop the timer:
	switch (TIMn){
		case TIM1:
			cli();
			TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
			sei();
			temp_ticks = TCNT1;
			temp_OVF = OVF1;
			break;
		case TIM3:
			cli();
			TCCR3B &= ~((1<<CS32) | (1<<CS31) | (1<<CS30));
			sei();
			temp_ticks = TCNT3;
			temp_OVF = OVF3;
			break;
		case TIM4:
			cli();
			TCCR4B &= ~((1<<CS42) | (1<<CS41) | (1<<CS40));
			sei();
			temp_ticks = TCNT4;
			temp_OVF = OVF4;
			break;
		default:
			break;
	}
	running = false;
};

void Timer::reset(){
	switch (TIMn){
		case TIM1:
			//Clear counter registers and values:
			cli();
			TCNT1 = 0;
			OVF1 = 0;
			sei();
			break;
		case TIM3:
			cli();
			TCNT3 = 0;
			OVF3 = 0;
			sei();
			break;
		case TIM4:
			cli();
			TCNT4 = 0;
			OVF4 = 0;
			sei();
			break;
		default:
			break;
	}
};

void Timer::store_current_values(){
	switch (TIMn){
		case TIM1:
			cli();
			temp_ticks = TCNT1;
			temp_OVF = OVF1;
			sei();
			break;
		case TIM3:
			cli();
			temp_ticks = TCNT3;
			temp_OVF = OVF3;
			sei();
			break;
		case TIM4:
			cli();
			temp_ticks = TCNT4;
			temp_OVF = OVF4;
			sei();
			break;
		default:
			break;
		}
};

uint32_t Timer::read_ms(){
	uint32_t micros = read_us();
	return micros/1000;
};

uint32_t Timer::read_us(){
	if (running){
		store_current_values();
	}
	uint32_t micros;
	uint16_t fraq1 = temp_OVF/2;
	uint16_t fraq2 = temp_ticks/256;
	micros = (fraq2*139)+ (temp_ticks-fraq2*256)/2 + ((35555*temp_OVF)+fraq1+fraq1/20+fraq1/200+fraq1/2000);
	return micros;
};

ISR(TIMER1_OVF_vect){
	cli();
	OVF1++;
	sei();
};
ISR(TIMER3_OVF_vect){
	cli();
	OVF3++;
	sei();
};
ISR(TIMER4_OVF_vect){
	cli();
	OVF4++;
	sei();
};