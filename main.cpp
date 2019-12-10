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
const uint8_t uplink_buf_length = 10;
uint8_t uplink_buf[uplink_buf_length];
uint8_t* downlink_buf;


int main (void){
	sei();
	Leds.toogle(RED);
	USART_init();
	//EEPROM_init();
	joined = radio.init_OTAA(appEui,appKey);
	radio.set_DR(0);
	radio.set_duty_cycle(1, 0);
	radio.sleep();
	/* Enable interrupts */
	//sei();
	uint8_t time = 0;
	uint16_t dataadc = 0;
	while (true){
		dataadc = AnalogIn.get_battery_lvl();
		uint32_t timestamp = rtc.get_epoch();
		printf("Epoch: %lu \n", timestamp);
		if (true){	
			time=0;
			printf("Bat %d \n", dataadc);
			Leds.toogle(GREEN);
			radio.wake();
			uplink_buf[1] = dataadc;
			timestamp = rtc.get_epoch();
			downlink_buf = radio.TX_bytes(uplink_buf, uplink_buf_length, 1);
			timestamp = rtc.get_epoch() - timestamp;
			for (uint8_t i = 0; i<8; i++){
				printf("%d ", downlink_buf[i]);
			}
			printf(" RTT:  %lu\n",timestamp);
			Leds.toogle(GREEN);
			radio.sleep();
			_delay_ms(5000);
			}
		}
   }
	

ISR(INT0_vect){
	threshold++;
	printf("Gyro interrupt \n");
	mpu6050_set_interrupt_thrshld(threshold);
};

ISR(INT1_vect){
	printf("Dummy interrupt \n");
};
ISR(WDT_vect){
	wdt_disable();
	sleep_disable();
};
