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
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
	mpu6050_init();
	mpu6050_normalPower_mode();
	//mpu6050_lowPower_mode();
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, 1);
	//mpu6050_accdisabled();
	//mpu6050_gyroEnabled();
	//mpu6050_tempSensorDisabled();
	/* Enable interrupts */
	//sei();
	#ifdef CONVERTED_DATA	
	double a = 0;double b = 0;double c = 0;double d = 0;double e = 0;double f = 0;double t = 0;
	#else
	int16_t a = 0;int16_t b = 0;int16_t c = 0;int16_t d = 0;int16_t e = 0;int16_t f = 0;int16_t t = 0;
	#endif
	//stopwatch.set_time_out(50);
	uint8_t time = 0;
	while (true){
		#ifdef CONVERTED_DATA
		mpu6050_getConvAccData(&a, &b, &c);
		mpu6050_getConvTempData(&t);
		mpu6050_getConvGyroData(&d, &e, &f);
		#else
		mpu6050_getRawGyroData(&a, &b, &c);
		mpu6050_getRawTempData(&t);
		mpu6050_getRawAccData(&d, &e, &f);
		#endif
		if (true){	
			time++;
			uint8_t data = mpu6050_testConnection();
			if(data && time==1){
				time=0;
				//printf("%s %i \n", "Adc:", dataadc);
				#ifdef CONVERTED_DATA
				mpu6050_getConvAccData(&a, &b, &c);
				mpu6050_getConvTempData(&t);
				mpu6050_getConvGyroData(&d, &e, &f);
				//mpu6050_getConvTempData(&t);
				//mpu6050_getConvGyroData(&d, &e, &f);
				_delay_ms(10);
				int a2 = (int)(a + 0.5 - (a<0));
				int b2 = (int)(b + 0.5 - (b<0));
				int c2 = (int)(c + 0.5 - (c<0));
				int t2 = (int)(t + 0.5 - (t<0));
				int d2 = (int)(d + 0.5 - (d<0));
				int e2 = (int)(e + 0.5 - (e<0));
				int f2 = (int)(f + 0.5 - (f<0));
				printf("ax: %i ay: %i az: %i temp: %i gx: %i gy: %i gz: %i \n", a2, b2, c2, t2, d2, e2, f2);
				#else
				mpu6050_getRawAccData(&a, &b, &c);
				mpu6050_getRawTempData(&t);
				mpu6050_getRawGyroData(&d, &e, &f);
				_delay_ms(10);
				printf("ax: %i ay: %i az: %i ", a2, b2, c2);
				//printf("temp: %i ", t2);
				//printf("gx: %i gy: %i gz: %i", d2, e2, f2);
				printf("\n");
				mpu6050_getRawAccData(&a, &b, &c);
				//mpu6050_getRawTempData(&t);
				//mpu6050_getRawGyroData(&d, &e, &f);
				_delay_ms(10);
				printf("ax: %i ay: %i az: %i ", a, b, c);
				//printf("temp: %i ", t);
				//printf("gx: %i gy: %i gz: %i ", d, e, f);
				printf(" \n");
				#endif
				}
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
