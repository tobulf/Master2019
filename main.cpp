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

int main (void){
	USART_init();
	adc AnalogIn;
	LED_driver Leds;
	Leds.toogle(RED);
	timer stopwatch;
	mpu6050_init();
	/* Enable interrupts */
	sei();	
	double a = 0;double b = 0;double c = 0;double d = 0;double e = 0;double f = 0;
	stopwatch.set_time_out(1000);
	uint16_t dataadc = 0;
	mpu6050_init_interrupt();
	uint8_t data = mpu6050_testConnection();
	printf("%i \n", data);
	AnalogIn.enable();
	while (true){
		AnalogIn.start_convertion(0);
		dataadc = AnalogIn.read();
		mpu6050_getConvData(&a, &b, &c, &d, &e, &f);
		if (stopwatch.time_out()){	
			data = mpu6050_testConnection();
			if(data){
				printf("%s %i \n", "Adc:", dataadc);
				_delay_ms(10);
				int d2 = (int)(d + 0.5 - (d<0));
				int e2 = (int)(e + 0.5 - (e<0));
				int f2 = (int)(f + 0.5 - (f<0));
				printf("%s %i %s %i %s %i \n","x: ", d2,"y: ", e2,"z: ", f2);
				printf("%s %d \n", "ADC: ",dataadc);
			}
		}
	}
	
}

ISR(INT0_vect){
	threshold++;
	printf("Ext_int %u \r\n", threshold);
	mpu6050_set_interrupt_thrshld(threshold);
}

