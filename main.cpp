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
#include <util/atomic.h>
#include "drivers/power_management.h"
#include "drivers/Timer.h"
#include "drivers/RN2483.h"
#include "drivers/RTC.h"
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
#include "EventQueue.h"



/* Since FILES, and FDEV doesn't work in C++, a workaround had to be made to enable printf:
   This is considered a bug by the WinAvr team, however has not been fixed.
*/

extern "C" {
	#include "drivers/Debug.h"
};

uint8_t buf[2];

adc AnalogIn;
LED_driver Leds;
RN2483 radio;
Timer timer;


int main (void){
	sei();
	Leds.toogle(YELLOW);
	USART_init();
	mpu6050_init();
	mpu6050_setSleepEnabled();
	if (!radio.init_OTAA(appEui,appKey)){
		Leds.toogle(RED);	
	};
	radio.set_DR(5);
	radio.set_duty_cycle(1, 0);
	buf[0] = AnalogIn.get_battery_lvl();
	printf("Bat: %d \n",buf[0]);
	Leds.toogle(GREEN);
	radio.TX_bytes(buf,1,1);
	Leds.toogle(GREEN);
	timer.start();
	while (true){
		
		if(timer.read_ms()>60000){
			Leds.reset();
			Leds.toogle(GREEN);
			buf[0]=AnalogIn.get_battery_lvl();
			timer.reset();
			while(!radio.TX_bytes(buf,1,1));
			Leds.toogle(GREEN);
		}
		else{
			Leds.toogle(YELLOW);
			_delay_ms(500);
			Leds.toogle(YELLOW);
			_delay_ms(500);
		}
	}
};
	

ISR(INT0_vect){
	
};

ISR(INT1_vect){
};

ISR(WDT_vect){
	wdt_disable();
	sleep_disable();
};
