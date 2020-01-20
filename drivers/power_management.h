/*
 * power_management.h
 *
 * Created: 25.11.2019 16:29:13
 *  Author: Tobias
 */ 


#ifndef POWER_MANAGEMENT_H_
#define POWER_MANAGEMENT_H_
#include <avr/io.h>
#include <avr/sleep.h>


/**
 * @brief Enable power save mode.
 * 
 */
void enable_power_save();
/**
 * @brief Enable power down mode.
 * 
 */
void enable_power_down();
/**
 * @brief Enable standby.
 * 
 */
void enable_standby();
/**
 * @brief Enable idle mode.
 * 
 */
void enable_idle();


#endif /* POWER_MANAGEMENT_H_ */