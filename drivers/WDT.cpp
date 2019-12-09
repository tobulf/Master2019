/*
 * WDT.c
 *
 * Created: 03.10.2019 12:46:32
 *  Author: Tobias
 */ 

#include "WDT.h"

void wdt_off(void){
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
