/*
 * ADC.h
 *
 * Created: 03.10.2019 16:12:40
 *  Author: Tobias
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

class adc
{
public:
	adc();
	void disable();
	void enable();
	uint16_t read(uint8_t ch);
protected:
private:
};




#endif /* ADC_H_ */