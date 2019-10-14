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
#include <stdbool.h>

class adc
{
public:
	adc();
	void disable();
	void enable();
	void start_convertion(uint8_t ch);
	uint16_t read(void);
	uint16_t *value;
protected:
private:
	
};




#endif /* ADC_H_ */