/*
 * ADC.cpp
 *
 * Created: 03.10.2019 16:12:30
 *  Author: Tobias
 */ 
#include "ADC.h"

adc::adc(){
	/*disable input 2-7.*/
	ADMUX = (1<<REFS0);
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(0<<ADPS1)|(0<<ADPS0);
	//DIDR0 = (1<<ADC0D) | (1<<ADC1D); 
}

void adc::enable(){
	ADCSRA |= (1<<ADEN);
}

void adc::disable(){
	ADCSRA &= ~(1<<ADEN);
}

uint16_t adc::read(uint8_t ch){
	// select the corresponding channel 0~7
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	
	// start single convertion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes ’0? again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}