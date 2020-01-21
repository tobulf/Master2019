/*
 * WDT.h
 *
 * Created: 03.10.2019 12:46:45
 *  Author: Tobias
 */ 


#ifndef WDT_H_
#define WDT_H_

#include <avr/interrupt.h> 
#include <avr/wdt.h>
#include <avr/io.h>

void WDT_off();
void wdt_INT_enable();
void wdt_INT_RST_enable();
void wdt_RST_enable();
void wdt_set_to_8s();
void wdt_set_to_4s();
void wdt_set_to_2s();


#endif /* WDT_H_ */