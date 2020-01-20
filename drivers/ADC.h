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
#include "WDT.h"
#include "power_management.h"

/**
 * @brief Offsett and gain constants for ADC.
 * 
 */
#define BATTERY_GAIN 3.75
#define BATTERY_OFFSET 610
#define LIGHTSENSOR_GAIN 9.8
#define LIGHTSENSOR_OFFSET 0


/**
 * @brief Object to use ADC.
 * 
 */
class adc
{
public:
	/**
	 * @brief Construct a new adc object
	 * 
	 */
	adc();
	/**
	 * @brief Get the battery lvl in percent.
	 * 
	 * @return uint16_t 
	 */
	uint8_t get_battery_lvl(void);
	/**
	 * @brief Get the light lvl in percent.
	 * 
	 * @return uint16_t 
	 */
	uint8_t get_light_lvl(void);
	/**
	 * @brief Disable ADC.
	 * 
	 */
	void disable();
	/**
	 * @brief Enable ADC.
	 * 
	 */
	void enable();
	/**
	 * @brief Start convertion on ch, ch range: 0 - 7.
	 * 
	 * @param ch 
	 */
	void start_convertion(uint8_t ch);
	/**
	 * @brief Boolean to poll if adc val is ready.
	 * 
	 * @return true 
	 * @return false 
	 */
	bool read(void);
	/**
	 * @brief The last read adc value.
	 * 
	 */
	uint16_t *value;
private:

	
};




#endif /* ADC_H_ */