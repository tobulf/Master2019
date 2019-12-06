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
#include <util/delay.h>

enum led {GREEN= 0, YELLOW, RED};

/**
 * @brief LED class object
 * 
 */
class LED_driver
{
private:
    /* data */
public:
    /**
    * @brief Construct a new led driver object
    * 
    */
    LED_driver();
    /**
     * @brief turn on a led for ms duration.
     * 
     * @param color 
     * @param ms 
     */
    void timed_toogle(led color, int ms);
    /**
     * @brief Toogle one led.
     * 
     * @param color 
     */
    void toogle(led color);
    /**
     * @brief Turn all leds off.
     * 
     */
	void reset();
};







#endif /* LED_DRIVER_H_ */