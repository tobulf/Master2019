/*
 * WDT.c
 *
 * Created: 03.10.2019 12:46:32
 *  Author: Tobias
 */ 

#include "WDT.h"

uint8_t timeouts = 0;
uint8_t max_timeouts = 0;

void WDT_off(void){
	cli();
	wdt_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE */
	/* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCSR = 0x00;
	sei();
};

void WDT_reset(){
	wdt_reset();
	timeouts = 0;
};

void wdt_INT_enable(){
	WDTCSR &= ~(1<<WDE);
	WDTCSR |= (1<<WDIE);
};

void wdt_INT_RST_enable(){
	WDTCSR |= (1<<WDE);
	WDTCSR |= (1<<WDIE);
};

void wdt_RST_enable(){
	WDTCSR  |= (1<<WDE);
	WDTCSR &= ~(1<<WDIE);
};

void wdt_set_to_1s(){
	wdt_enable(WDTO_1S);
	wdt_reset();
	WDTCSR &= ~((1<<WDP3) | (1<<WDP0));
	WDTCSR |= (1<<WDP2) | (1<<WDP1);
};

void wdt_set_to_2s(){
	wdt_enable(WDTO_2S);
	wdt_reset();
	WDTCSR |= (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
	WDTCSR &= ~((1<<WDP3));
};

void wdt_set_to_4s(){
	wdt_enable(WDTO_4S);
	wdt_reset();
	WDTCSR |= (1<<WDP3);
	WDTCSR &= ~((1<<WDP2) | (1<<WDP1) | (1<<WDP0));
};

void wdt_set_to_8s(){
	wdt_enable(WDTO_8S);
	wdt_reset();
	WDTCSR |= (1<<WDP3) | (1<<WDP0);
	WDTCSR &= ~((1<<WDP2) | (1<<WDP1));
	max_timeouts = 0;
};

void wdt_set_to_16s(){
	wdt_enable(WDTO_8S);
	wdt_reset();
	WDTCSR |= (1<<WDP3) | (1<<WDP0);
	WDTCSR &= ~((1<<WDP2) | (1<<WDP1));
	max_timeouts = 1;
};

void wdt_set_to_24s(){
	wdt_enable(WDTO_8S);
	wdt_reset();
	WDTCSR |= (1<<WDP3) | (1<<WDP0);
	WDTCSR &= ~((1<<WDP2) | (1<<WDP1));
	max_timeouts = 2;
};


ISR(WDT_vect){
	cli();
	timeouts++;
	wdt_set_to_8s();
	sei();
	if (timeouts > max_timeouts){
		wdt_RST_enable();
		wdt_set_to_1s();
		wdt_enable(WDTO_15MS);
	}
};