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
#include "drivers/timer.h"
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

uint16_t threshold = 38;
bool joined = false;

#define CONVERTED_DATA

adc AnalogIn;
LED_driver Leds;
timer stopwatch;
RN2483 radio;

int main (void){
	sei();
	Leds.toogle(RED);
	USART_init();
	//EEPROM_init();
	//joined = radio.init_OTAA(appEui,appKey);
	//radio.set_DR(0);
	//radio.set_duty_cycle(1, 0);
	//radio.sleep();
	mpu6050_init();
	/* Enable interrupts */
	//sei();
	#ifdef CONVERTED_DATA	
	double a = 0;double b = 0;double c = 0;double d = 0;double e = 0;double f = 0;double t;
	#else
	int16_t a = 0;int16_t b = 0;int16_t c = 0;int16_t d = 0;int16_t e = 0;int16_t f = 0;int16_t t;
	#endif
	stopwatch.set_time_out(50);
	uint8_t time = 0;
	uint16_t dataadc = 0;
	while (true){
		dataadc = AnalogIn.get_battery_lvl();
		#ifdef CONVERTED_DATA
		mpu6050_getConvAccData(&a, &b, &c);
		mpu6050_getConvTempData(&t);
		mpu6050_getConvGyroData(&d, &e, &f);
		#else
		mpu6050_getRawGyroData(&a, &b, &c);
		mpu6050_getRawTempData(&t);
		mpu6050_getRawAccData(&d, &e, &f);
		#endif
		if (stopwatch.time_out()){	
			//printf("Adc: %d \r\n",(dataadc));
			time++;
			uint8_t data = mpu6050_testConnection();
			if(data && time==1){
				time=0;
				//printf("%s %i \n", "Adc:", dataadc);
				#ifdef CONVERTED_DATA
				mpu6050_getConvAccData(&a, &b, &c);
				mpu6050_getConvTempData(&t);
				mpu6050_getConvGyroData(&d, &e, &f);
				_delay_ms(10);
				int a2 = (int)(a + 0.5 - (a<0));
				int b2 = (int)(b + 0.5 - (b<0));
				int c2 = (int)(c + 0.5 - (c<0));
				int t2 = (int)(t + 0.5 - (t<0));
				int d2 = (int)(d + 0.5 - (d<0));
				int e2 = (int)(e + 0.5 - (e<0));
				int f2 = (int)(f + 0.5 - (f<0));
				//printf("ax: %i ay: %i az: %i temp: %i gx: %i gy: %i gz: %i \n", a2, b2, c2, t2, d2, e2, f2);
				#else
				mpu6050_getRawAccData(&a, &b, &c);
				mpu6050_getRawTempData(&t);
				mpu6050_getRawGyroData(&d, &e, &f);
				_delay_ms(10);
				printf("ax: %i ay: %i az: %i temp: %i gx: %i gy: %i gz: %i \n", a, b, c, t, d, e, f);
				#endif
				printf("Bat %d \n", dataadc);
				//radio.wake();
				Leds.toogle(GREEN);
				//radio.TX_string(String(420), 1);
				Leds.toogle(GREEN);
				//radio.sleep();
			}
		}
	}
	
}

ISR(INT0_vect){
	threshold++;
	mpu6050_set_interrupt_thrshld(threshold);
};




