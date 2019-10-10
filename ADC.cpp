/*
 * ADC.cpp
 *
 * Created: 03.10.2019 16:12:30
 *  Author: Tobias
 */ 
#include "ADC.h"

adc::adc(){
	/*disable input 2-7.*/
	DIDR0 = (1<<ADC0D) | (1<<ADC1D); 
}

void adc::enable(){
	ADCSRA |= (1<<ADEN);
}

void adc::disable(){
	ADCSRA &= ~(1<<ADEN);
}