/*
 * ADC.h
 *
 * Created: 03.10.2019 16:12:40
 *  Author: Tobias
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdbool.h>
#include "power_management.h"

class adc
{
public:
	adc();
	uint16_t get_battery_lvl(void);
	uint16_t get_light_lvl(void);
private:
	void disable();
	void enable();
	void start_convertion(uint8_t ch);
	uint16_t read(void);
	uint16_t *value;
	
};




#endif /* ADC_H_ */