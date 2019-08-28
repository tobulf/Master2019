/*
 * LED_driver.cpp
 *
 * Created: 09.10.2018 22:09:31
 *  Author: tobiasu
 */ 


#include "LED_driver.h"
#include <util/delay.h>

/* Handy macros: */
#define set_bit(reg, bit ) (reg |= (1 << bit))
#define clear_bit(reg, bit ) (reg &= ~(1 << bit))
#define test_bit(reg, bit ) (reg & (1 << bit))



LED_driver::LED_driver(/* args */){
    /*Set Pin 0 to 2 to output: */
    DDRA = (1<<DDB0)|(1<<DDB1)|(1<<DDB2);
    /*Set all pins Low*/
    clear_bit(PORTA, 0); clear_bit(PORTA, 1); clear_bit(PORTA, 2); 
};


void LED_driver::reset(){
	PORTA &= ~(1 << GREEN);
	PORTA &= ~(1 << RED);
	PORTA &= ~(1 << YELLOW);
	
};


void LED_driver::toogle(led color){
    
    switch (color){
        case GREEN:
            PORTA ^= (1 << GREEN);
            break;
        
        case RED:
            PORTA ^= (1 << RED);
            break;

        case YELLOW:
            PORTA ^= (1 << YELLOW);
            break;

        default:
            break;
    }
};




void LED_driver::timed_toogle(led color, int ms){
    toogle(color);
    for(int n = 0; n < ms; n++){
        _delay_ms(1);
    }
    toogle(color);
};
