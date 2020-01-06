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

bool joined = false;

#define CONVERTED_DATA
#define INTERRUPT_GYRO

adc AnalogIn;
LED_driver Leds;
RN2483 radio;

int main (void){
	sei();
	Leds.toogle(RED);
	USART_init();
	mpu6050_init();
	//mpu6050_lowPower_mode();
	mpu6050_normalPower_mode();
	/* Enable interrupts */
	//sei();
	#ifdef CONVERTED_DATA	
	int c = 0;int t = 0;
	#else
	int16_t c = 0;int16_t t = 0;
	#endif
	//stopwatch.set_time_out(50);
	uint8_t time = 0;
	while (true){
		#ifdef INTERRUPT_GYRO
		#ifdef CONVERTED_DATA
		mpu6050_getConvAccData(&c);
		mpu6050_getConvTempData(&t);
		#endif
		if (true){	
			time++;
			uint8_t data = mpu6050_testConnection();
			if(data && time==1){
				time=0;
				//printf("%s %i \n", "Adc:", dataadc);
				#ifdef CONVERTED_DATA
				mpu6050_getConvAccData(&c);
				mpu6050_getConvTempData(&t);
				_delay_ms(10);
				int c2 = (int)(c + 0.5 - (c<0));
				int t2 = (int)(t + 0.5 - (t<0));
				//printf("az: %i temp: %i \n", c2, t2);
				#endif
				}
			}
			#endif
		}
   }
	

ISR(INT0_vect){
	cli();
	uint8_t interrupt = mpu6050_get_interrupt_status();
	Leds.toogle(RED);
	if ((interrupt & (1 << 6))){
		printf("Motion interrupt %d \n", interrupt);
		mpu6050_enable_RAW_RDY_interrupt();

	}
	else{
		printf("Data rdy %d \n", interrupt);
		_delay_ms(1000);
		mpu6050_enable_motion_interrupt();
	}
	sei();
	//mpu6050_enable_RAW_RDY_interrupt();
};

ISR(INT1_vect){
	printf("Dummy interrupt \n");
};
ISR(WDT_vect){
	wdt_disable();
	sleep_disable();
};
