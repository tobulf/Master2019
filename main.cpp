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
#include <avr/wdt.h>
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

bool gyro_data = false;

volatile uint8_t buf[14];
int16_t x;
int32_t x_mean = 0;
int16_t y;
int32_t y_mean = 0;
int16_t z;
int32_t z_mean = 0;
uint16_t size;
int16_t temperature_raw;
int16_t temperature_converted;
int16_t temp_offset = 0;

adc AnalogIn;
LED_driver Leds;
RN2483 radio;
RTC rtc;
Timer timer;

int main (void){
	USART_init();
	wdt_reset();
	mpu6050_init();
	mpu6050_normalPower_mode();
	Leds.toogle(RED);
	_delay_ms(1000);
	mpu6050_tempSensorEnabled();
	mpu6050_getRawTempData(&temperature_raw);
	mpu6050_tempSensorDisabled();
	mpu6050_FIFO_reset();
	mpu6050_FIFO_enable();
	mpu6050_get_interrupt_status();
	mpu6050_enable_FIFO_OVF_interrupt();
	mpu6050_enable_pin_interrupt();
	while (!gyro_data){
		_delay_ms(1);
	};
	Leds.toogle(RED);
	Leds.toogle(YELLOW);
	mpu6050_get_FIFO_length(&size);
	uint8_t times = 0;
/*	size = 170;*/
	while (size>4){
		mpu6050_FIFO_pop_raw(&x, &y, &z);
		x_mean = x_mean + x;
		y_mean = y_mean + y;
		z_mean = z_mean + z;
		times ++;
		mpu6050_get_FIFO_length(&size);
	}
	bool temp_calibrated = false;
	while(!temp_calibrated){
		temperature_converted = ((temperature_raw-temp_offset)/340);
		if (temperature_converted > 20){
			temp_offset = temp_offset + 340;
		}
		else if (temperature_converted < 20){
			temp_offset = temp_offset - 340;
		}
		else{
			break;
		}
	}
	Leds.toogle(YELLOW);
	x_mean = x_mean/times;
	y_mean = y_mean/times;
	z_mean = z_mean/times;
 	EEPROM_write_int16(MPU6050_CALIBRATED_AXOFFSET, (int16_t)x_mean);
 	EEPROM_write_int16(MPU6050_CALIBRATED_AYOFFSET, (int16_t)y_mean);
 	EEPROM_write_int16(MPU6050_CALIBRATED_AZOFFSET, (int16_t)z_mean);
 	EEPROM_write_int16(MPU6050_CALIBRATED_TEMPOFFSET, (int16_t)temp_offset);
 	EEPROM_write(MPU6050_CALIBRATED, 1);
	Leds.toogle(GREEN);
	printf("Calculated: %li %li %li %i \n", x_mean, y_mean, z_mean, temp_offset);
	mpu6050_init();
	mpu6050_tempSensorEnabled();
	while(true){
		mpu6050_getConvTempData(&temperature_converted);
		mpu6050_getConvAccData(&x,&y,&z);
		printf("%i %i %i %i \n", x, y, z, temperature_converted);
		_delay_ms(100);
	};

}
	

ISR(INT0_vect){
	cli();
	mpu6050_disable_pin_interrupt();
	sei();
	mpu6050_FIFO_stop();
	mpu6050_disable_interrupt();
	mpu6050_get_interrupt_status();
	gyro_data = true;
};

ISR(INT1_vect){
	printf("Dummy interrupt \n");
};

ISR(WDT_vect){
	WDT_off();
	printf("WDT! \n");
};
