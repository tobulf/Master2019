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

void WDT_off(void);
void WDT_set_prescaler();
void WDT_INT_enable();
void WDT_INT_RST_enable();
void WDT_RST_enable();

#endif /* WDT_H_ */