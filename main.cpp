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
#include "drivers/Timer.h"



/* Since FILES, and FDEV doesn't work in C++, a workaround had to be made to enable printf:
   This is considered a bug by the WinAvr team, however has not been fixed.
*/

extern "C" {
	#include "drivers/Debug.h"

};

/* Handy macros: */
#define set_bit(reg, bit ) (reg |= (1 << bit))
#define clear_bit(reg, bit ) (reg &= ~(1 << bit))
#define test_bit(reg, bit ) (reg & (1 << bit))

uint16_t threshold = 0;
bool joined = false;

#define CONVERTED_DATA

adc AnalogIn;
LED_driver Leds;
RTC rtc;
RN2483 radio;
const uint8_t uplink_buf_length = 13;
uint8_t uplink_buf[uplink_buf_length];
uint8_t* downlink_buf;
uint64_t new_timestamp;
uint32_t t_callback;
Timer callback_timer;
uint32_t sec;
uint32_t us;
uint64_t timestamp;
bool print = false;
bool sync = false;

void disablePrint(){
	PCICR &= ~(1<<PCIE3);
}
void enablePrint(){
	PCICR |= (1<<PCIE3);
}
int main (void){
	sei();
	Leds.toogle(RED);
	USART_init();
	printf("Booting... \n");
	//EEPROM_init();
	joined = radio.init_OTAA(appEui, appKey, devEui);
	radio.set_DR(5);
	radio.set_duty_cycle(0);
	radio.sleep();
	rtc.set_alarm_period(60);
	rtc.start_alarm();
	Leds.toogle(RED);
	EICRA |= (1<<ISC11);
	EICRA |= (1<<ISC10);
	EIMSK |= (1<<INT1);
	PCICR |= (1<<PCIE3);
	PCMSK3|= (1<<PCINT31);
	PORTD |= (0<<3) | (0<<7);
	
	
	/* Enable interrupts */
	//sei();
	uint16_t dataadc = 0;
	while (true){
		//printf("Epoch: %lu \n", timestamp);
		if (!sync){	
			//printf("Bat %d \n", dataadc);
			Leds.toogle(GREEN);
			radio.wake();
			//5, 154, 220, 62, 115, 197, 162: 1577645614 ms: 810 us: 530
			uplink_buf[0] = 1;
			uplink_buf[1] = 5;
			uplink_buf[2] = 154;
			uplink_buf[3] = 220;
			uplink_buf[4] = 62;
			uplink_buf[5] = 115;
			uplink_buf[6] = 197;
			uplink_buf[7] = 162;
			callback_timer.start();
			if(radio.TX_bytes(uplink_buf, uplink_buf_length, 1)){
				callback_timer.stop();
				if (radio.unread_downlink()){
					t_callback = callback_timer.read_us();
					downlink_buf = radio.read_downlink_buf();
					convert_sync_response(downlink_buf, new_timestamp, t_callback);
					rtc.set_time(new_timestamp);	
					sync = true;
				}
			}
			printf("callback time: %lu \n", t_callback);
			callback_timer.stop();
			callback_timer.reset();
			Leds.toogle(GREEN);
			radio.sleep();
			}
		if (print){
			print = false;
			printf("%lu, %lu\n",sec,us);
			_delay_ms(1000);
			enablePrint();
		}
		if (rtc.get_alarm_status()){
			sync = false;
		}
		_delay_ms(1);
		}
   }
	


ISR(INT1_vect){
	if (sync){
		sync = false;
		printf("Retry sync...\n");
	}
};

ISR(PCINT3_vect){
	cli();
	disablePrint();
	timestamp = rtc.get_timestamp();
	sei();
	sec = timestamp / 1000000;
	us = uint32_t(timestamp-(uint64_t)sec*1000000);
	print = true;
};

ISR(WDT_vect){
	wdt_disable();
	sleep_disable();
};
