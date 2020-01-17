/*
 * ADC.cpp
 *
 * Created: 03.10.2019 16:12:30
 *  Author: Tobias
 */ 
#include "ADC.h"
#include "debug.h"

uint16_t val = 0;
bool rdy = false;

adc::adc(){
	value = &val;
	ADMUX = (1<<REFS0);
	ADCSRA = (1<<ADPS2)|(0<<ADPS1)|(0<<ADPS0) | (1<<ADIE);
	//DIDR0 = (1<<ADC0D) | (1<<ADC1D); 
}

void adc::enable(){
	ADCSRA |= (1<<ADEN);
}

void adc::disable(){
	ADCSRA &= ~(1<<ADEN);
}

void adc::start_convertion(uint8_t ch){
	// select the corresponding channel 0~7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	// start single convertion
	ADCSRA |= (1<<ADSC);
}

uint8_t adc::get_battery_lvl(void){
	enable();
	start_convertion(1);
	while(!read());
	disable();
	int level = (int)((val-BATTERY_OFFSET)/BATTERY_GAIN);
	if (level > 100){
		return 100;
	}
	else if (level < 0){
		return 0;
	}
	else{
		return (uint8_t) level;
	}
}


uint16_t adc::get_light_lvl(void){
	enable();
	start_convertion(0);
	while(!read());
	disable();
	int level = (int)((val-LIGHTSENSOR_OFFSET)/LIGHTSENSOR_GAIN);
	if (level > 100){
		return level;
	}
	else if (level < 0){
		return level;
	}
	else{
		return level;
	}
}


bool adc::read(void){
	if(rdy){
		rdy = false;	
		return true;
	}
	else {
		return false;
	}
}

ISR(ADC_vect){
	val = ADC;
	rdy = true;
}