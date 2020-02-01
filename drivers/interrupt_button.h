/*
 * interrupt_button.h
 *
 * Created: 21.01.2020 18:19:35
 *  Author: Tobias
 */ 


#ifndef INTERRUPT_BUTTON_H_
#define INTERRUPT_BUTTON_H_

#include <avr/io.h>

void interrupt_button_init();

void interrupt_button_enable();

void interrupt_button_disable();


#endif /* INTERRUPT_BUTTON_H_ */