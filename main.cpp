/*
 * Master-node.cpp
 *
 * Created: 28.08.2019 12:08:38
 * Author : Tobias
 */ 

#include <avr/interrupt.h> 
#include <util/delay.h>
#include <avr/io.h>
#include <util/delay_basic.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include "utils.h"
#include "drivers/power_management.h"
#include "drivers/RN2483.h"
#include "drivers/RTC.h"
#include "drivers/SPI_driver.h"
#include "drivers/LED_driver.h"
#include "drivers/LoRa_cfg.h"
#include "drivers/ADC.h"
#include "drivers/WString.h"
#include "drivers/WDT.h"
#include "drivers/EEPROM.h"
#include "drivers/i2cmaster.h"
#include "drivers/mpu6050.h"
#include "drivers/EEPROM.h"
#include "drivers/MemoryAdresses.h"



/* Since FILES, and FDEV doesn't work in C++, a workaround had to be made to enable printf:
   This is considered a bug by the WinAvr team, however has not been fixed.
*/

extern "C" {
	#include "drivers/Debug.h"

};


adc AnalogIn;
RTC rtc;
RN2483 radio;
LED_driver LED;
uint64_t new_timestamp = 10000000000;
bool sync = true;
bool sleep = false;

int main (void){
	LED.toogle(RED);
	/* Enable interrupts */
	sei();
	USART_init();
	LED.toogle(RED);
	mpu6050_init();
	mpu6050_disable_interrupt();
	mpu6050_gyroDisabled();
	mpu6050_accDisabled();
	mpu6050_tempSensorDisabled();
	mpu6050_setSleepEnabled();
	radio.sleep();
	AnalogIn.disable();
	EICRA |= (1<<ISC11);
	EICRA |= (1<<ISC10);
	EIMSK |= (1<<INT1);
	PCICR |= (1<<PCIE1)| (1<<PCIE3);
	PCMSK1|= (1<<PCINT13);
	PCMSK3|= (1<<PCINT31);
	PORTD |= (0<<3) | (0<<7);
	PORTB |= (0<<5);
	LED.toogle(RED);
	LED.toogle(GREEN);
	//sei();
	/*Wait for sync: */
	while (!sync);
	while (true){
		printf("HELLO \n");
		if(sleep){
			enable_power_save();
			sleep_enable();
			sleep_mode();
		}
			
	}
   }
	

ISR(INT1_vect){
	cli();
	sleep_disable();
	if(sync){
		if (sleep){
			printf("woke up! \n");
			LED.toogle(GREEN);
			sleep = false;
		}
		else{
			sleep = true;
			printf("going to sleep...\n");
			LED.toogle(GREEN);
		}
	}
	sei();
	
};

ISR(PCINT1_vect){
	cli();
	if(!sync){
		sync = true;
		rtc.set_time(new_timestamp);
	}
	sei();
};

ISR(PCINT3_vect){
	LED.toogle(YELLOW);
	cli();
	AnalogIn.enable();
	uint64_t timestamp = rtc.get_timestamp();
	sei();
	uint8_t bat = AnalogIn.get_battery_lvl();
	AnalogIn.disable();
	uint32_t sec = timestamp / 1000000;
	uint32_t us = uint32_t(timestamp-(uint64_t)sec*1000000);
	printf("Bat %d %s Time: %lu s %lu us \n", bat, "%", sec, us);
	LED.toogle(YELLOW);
};
	