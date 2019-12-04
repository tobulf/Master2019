/*
 * WDT.h
 *
 * Created: 03.10.2019 12:46:45
 *  Author: Tobias
 */ 


#ifndef WDT_H_
#define WDT_H_

#include <avr/interrupt.h> 
#include "Debug.h"

#define WDT_reset() __asm__ __volatile__ ("wdr")


enum wdt_prescaler {WDT_16MS_PRESCALER= 0, 
	WDT_32MS_PRESCALER,
	WDT_64MS_PRESCALER, 
	WDT_128MS_PRESCALER, 
	WDT_025S_PRESCALER, 
	WDT_05S_PRESCALER, 
	WDT_1S_PRESCALER, 
	WDT_2S_PRESCALER, 
	WDT_4S_PRESCALER, 
	WDT_8S_PRESCALER};

void WDT_set_prescaler(wdt_prescaler prescaler);
void WDT_stop();
void WDT_INT_enable();
void WDT_INT_RST_enable();
void WDT_RST_enable();


#endif /* WDT_H_ */