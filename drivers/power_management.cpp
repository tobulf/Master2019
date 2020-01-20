/*
 * power_management.cpp
 *
 * Created: 25.11.2019 16:28:29
 *  Author: Tobias
 */ 
#include "power_management.h"

void enable_power_save(){
	SMCR = 0x03;
}
void enable_power_down(){
	SMCR = 0x02;
}
void enable_standby(){
	SMCR = 0x06;
}
void enable_idle(){
	SMCR = 0x00;
}



