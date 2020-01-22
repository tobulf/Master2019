/*
 * interrupt_button.cpp
 *
 * Created: 21.01.2020 18:19:20
 *  Author: Tobias
 */ 

#include "interrupt_button.h"

void interrupt_button_init(){
	EICRA &= ~(1<<ISC10);
	EICRA |= (1<<ISC11);
	DDRD  |= (0<<3);
	PORTD |= (0<<3);
	MCUCR &= ~(1<<PUD);
};

void interrupt_button_enable(){
	EIMSK |= (1<<INT1);
};

void interrupt_button_disable(){
	EIMSK &= ~(1<<INT1);
};