/*
 * WDT.c
 *
 * Created: 03.10.2019 12:46:32
 *  Author: Tobias
 */ 

#include "WDT.h"

void WDT_off(void){
	cli();
	WDT_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE */
	/* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCSR = 0x00;
	sei();
};

void WDT_set_prescaler(){
	cli();
	WDT_reset();
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
	sei();
};

void WDT_INT_enable(){
	WDTCSR &= ~(1<<WDE);
	WDTCSR |= (1<<WDIE);
};

void WDT_INT_RST_enable(){
	WDTCSR |= (1<<WDE);
	WDTCSR |= (1<<WDIE);
};

void WDT_RST_enable(){
	WDTCSR  |= (1<<WDE);
	WDTCSR &= ~(1<<WDIE);
};
