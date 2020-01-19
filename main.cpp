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
enum MESSAGE_HEADERS {SYNC = 1, EVENT = 2, APPEND_DATA=4, DUMMY_MSG=8};
enum STATE {ACTIVE=1, SLEEP, DMY_EVENT, ACC_EVENT};
STATE cur_state;
STATE prev_state;

bool joined = false;
uint8_t radio_buf[70];

adc AnalogIn;
LED_driver Leds;
RN2483 radio;
RTC rtc;
Timer timer;


int main (void){
	sei();
	Leds.toogle(RED);
	USART_init();
	mpu6050_init();
	joined = radio.init_OTAA(appEui,appKey);
	if (!joined){
		Leds.toogle(YELLOW);
	}
	radio.set_DR(5);
	radio.set_duty_cycle(0);
	radio.sleep();
	mpu6050_normalPower_mode();
	mpu6050_set_interrupt_mot_thrshld(15);
	mpu6050_set_interrupt_mot_dur(1);
	mpu6050_enable_motion_interrupt();
	mpu6050_enable_pin_interrupt();
	cur_state = ACTIVE;
	prev_state = ACTIVE;
	Leds.toogle(RED);
	int16_t x;
	int16_t y;
	int16_t z;
	while (true){
		switch (cur_state){
			case ACTIVE:
				Leds.turn_on(GREEN);
				if (prev_state!=ACTIVE){
					//Send the data:
					Leds.turn_on(YELLOW);
					radio.wake();
					uint16_t size;
					mpu6050_get_FIFO_length(&size);
					//Read and send max 960 bytes of data: 8s recording.
					bool sent = false;
					for (uint8_t i = 3; i<=50;i = i + 6){
						mpu6050_FIFO_pop(&x, &y, &z);
						radio_buf[i]=(uint8_t)((x>>8) & 0xFF);
						radio_buf[i+1]=(uint8_t)(x & 0xFF);
						radio_buf[i+2]=(uint8_t)((y>>8) & 0xFF);
						radio_buf[i+3]=(uint8_t)(y & 0xFF);
						radio_buf[i+4]=(uint8_t)((z>>8) & 0xFF);
						radio_buf[i+5]=(uint8_t)(z & 0xFF);
					}
					while (!sent){
						sent = radio.TX_bytes(radio_buf, 51 ,1);
					}
					mpu6050_get_FIFO_length(&size);
					radio_buf[0] = APPEND_DATA;
					while(size>64){
						sent = false;
						for (uint8_t i = 1; i<=48;i = i + 6){
							mpu6050_FIFO_pop(&x, &y, &z);
							radio_buf[i]=(uint8_t)((x>>8) & 0xFF);
							radio_buf[i+1]=(uint8_t)(x & 0xFF);
							radio_buf[i+2]=(uint8_t)((y>>8) & 0xFF);
							radio_buf[i+3]=(uint8_t)(y & 0xFF);
							radio_buf[i+4]=(uint8_t)((z>>8) & 0xFF);
							radio_buf[i+5]=(uint8_t)(z & 0xFF);
						}
						while (!sent){
							sent = radio.TX_bytes(radio_buf, 49 ,1);
						}
						mpu6050_get_FIFO_length(&size);
						
					}
					radio.sleep();
					prev_state = ACTIVE;
					Leds.reset();
					Leds.turn_on(GREEN);
					mpu6050_FIFO_reset();
					mpu6050_enable_motion_interrupt();
					mpu6050_enable_pin_interrupt();
					break;
				}
				else{
					cur_state = SLEEP;
					prev_state = ACTIVE;
					mpu6050_lowPower_mode();
					Leds.reset();
					enable_power_save();
					sleep_enable();
					sleep_mode();
				}
				break;
			case SLEEP:
				if(cur_state!=SLEEP){
					break;
				}
				else{
					//check for sync here.(set sync=false;)
					enable_power_save();
					sleep_enable();
					sleep_mode();
				}	
				break;
			case ACC_EVENT:
				AnalogIn.enable();
				_delay_ms(1);
				radio_buf[0] = EVENT;
				radio_buf[1] = AnalogIn.get_battery_lvl();
				radio_buf[2] = 1;
				AnalogIn.disable();
				enable_power_save();
				sleep_enable();
				sleep_mode();
				prev_state = ACC_EVENT;
				cur_state = ACTIVE;
 				break;
			case DMY_EVENT:
				break;
			default:
				break;
			}
		}
	}
	

ISR(INT0_vect){
	cli();
	mpu6050_disable_pin_interrupt();
	sei();
	sleep_disable();
	mpu6050_disable_interrupt();
	Leds.turn_on(YELLOW);
	uint8_t interrupt = mpu6050_get_interrupt_status();
	if ((interrupt & (1 << 6)) && cur_state!=ACC_EVENT){
		prev_state = cur_state;
		cur_state = ACC_EVENT;
		mpu6050_normalPower_mode();
		mpu6050_FIFO_reset();
		mpu6050_FIFO_enable();
		mpu6050_enable_FIFO_OVF_interrupt();
		mpu6050_enable_pin_interrupt();
	}
	else {
		mpu6050_FIFO_stop();
	}

};

ISR(INT1_vect){
	printf("Dummy interrupt \n");
};

ISR(WDT_vect){
	wdt_disable();
	sleep_disable();
};
