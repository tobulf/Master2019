/*
 * power_management.cpp
 *
 * Created: 25.11.2019 16:28:29
 *  Author: Tobias
 */ 
#include "power_management.h"

void enable_power_save(){
	SMCR &= ~(1<<SM2);
	SMCR |= (1<<SM1) | (1<<SM0);
}
void enable_power_down(){
	SMCR &= ~(1<<SM2) & ~(1<<SM0);
	SMCR |= (1<<SM1);
}
void enable_standby(){
	SMCR |= (1<<SM2) | (1<<SM1);
	SMCR &= ~(1<<SM0);
}
void enable_idle(){
	SMCR = 0x00;
}
void enable_adc_NR(){
	SMCR &= ~(1<<SM2) & ~(1<<SM1);
	SMCR |= (1<<SM0);
}



