/*
 * LED_driver.h
 *
 * Created: 09.10.2018 22:09:45
 *  Author: tobiasu
 */ 


#ifndef LED_DRIVER_H_
#define LED_DRIVER_H_

#include <util/delay.h>
#include <avr/io.h>

enum led {YELLOW = 0, GREEN, RED};


class LED_driver
{
private:
    /* data */
public:
    LED_driver();
    void timed_toogle(led color, int ms);
    void toogle(led color);
	void reset();
};







#endif /* LED_DRIVER_H_ */