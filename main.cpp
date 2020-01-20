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
enum MESSAGE_PORTS {SYNC = 1, EVENT = 2, APPEND_DATA=4, KEEP_ALIVE=8};
enum STATE {IDLE, ALIVE_TRANSMIT, DATA_TRANSMIT, SLEEP};
STATE cur_state;
STATE prev_state;

bool joined = false;
bool gyro_data = false;
bool fifo_started = false;
bool sent = false;

uint8_t radio_buf[70];
int16_t x;
int16_t y;
int16_t z;
uint32_t timestamp;
uint32_t alive_timestamp;

// int main (void){
// 	sei();
// 	wdt_reset();
// 	LED_driver temp;
// 	temp.toogle(RED);
// 	wdt_reset();
// 	wdt_enable(WDTO_8S);
// 	wdt_set_to_8s();
// 	wdt_RST_enable();
// 	USART_init();
// 	wdt_reset();
//  	while(true){
// 		wdt_reset();
// 		printf("yea boi \n");
// 		_delay_ms(5000);
// 		temp.toogle(YELLOW);
// 		wdt_reset();
// 		_delay_ms(5000);
// 		printf("WORKS! \n");
// 		wdt_reset();
// 		WDT_off();
// 		_delay_ms(10000);
// 		printf("now to...");
// 		
// 	}
// 	WDT_off();
// };

adc AnalogIn;
LED_driver Leds;
RN2483 radio;
RTC rtc;
Timer timer;

int main (void){
	sei();
	wdt_enable(WDTO_8S);
	wdt_set_to_8s();
	wdt_RST_enable();
	wdt_reset();
	Leds.toogle(RED);
	USART_init();
	wdt_reset();
	while (!joined){
		joined = radio.init_OTAA(appEui,appKey);
		wdt_reset();
	}
	radio.set_duty_cycle(0);
	radio.set_RX_window_size(1000);
	radio.sleep();
	wdt_reset();
	mpu6050_init();
	mpu6050_normalPower_mode();
	mpu6050_set_interrupt_mot_thrshld(10);
	mpu6050_get_interrupt_status();
	mpu6050_enable_motion_interrupt();
	mpu6050_enable_pin_interrupt();
	mpu6050_lowPower_mode();
	wdt_reset();
	rtc.set_alarm_period(120);
	rtc.start_alarm();
	cur_state = ALIVE_TRANSMIT;
	Leds.toogle(RED);
	wdt_reset();
	while (true){
		switch (cur_state){
			case DATA_TRANSMIT:
				wdt_reset();
				Leds.reset();
				Leds.turn_on(YELLOW);
				Leds.turn_on(GREEN);
				gyro_data = false;
				//Send the data:
				Leds.turn_on(YELLOW);
				AnalogIn.enable();
				radio.wake();
				uint16_t size;
				mpu6050_get_FIFO_length(&size);
				//Read and send max 960 bytes of data: 8s recording.
				radio_buf[0]=(uint8_t)((timestamp>>24) & 0xFF);
				radio_buf[1]=(uint8_t)((timestamp>>16) & 0xFF);
				radio_buf[2]=(uint8_t)((timestamp>>8) & 0xFF);
				radio_buf[3]=(uint8_t)(timestamp & 0xFF);
				radio_buf[4] = AnalogIn.get_battery_lvl();
				radio_buf[5] = AnalogIn.get_light_lvl();
				AnalogIn.disable();
				sent = false;
				wdt_reset();
				for (uint8_t i = 6; i<=53;i = i + 6){
					mpu6050_FIFO_pop(&x, &y, &z);
					radio_buf[i]=(uint8_t)((x>>8) & 0xFF);
					radio_buf[i+1]=(uint8_t)(x & 0xFF);
					radio_buf[i+2]=(uint8_t)((y>>8) & 0xFF);
					radio_buf[i+3]=(uint8_t)(y & 0xFF);
					radio_buf[i+4]=(uint8_t)((z>>8) & 0xFF);
					radio_buf[i+5]=(uint8_t)(z & 0xFF);
				}
				wdt_reset();
				while (!sent){
					sent = radio.TX_bytes(radio_buf, 53, EVENT);
					wdt_reset();
				}
				mpu6050_get_FIFO_length(&size);
				while(size>64){
					sent = false;
					for (uint8_t i = 0; i<=47;i = i + 6){
						mpu6050_FIFO_pop(&x, &y, &z);
						radio_buf[i]=(uint8_t)((x>>8) & 0xFF);
						radio_buf[i+1]=(uint8_t)(x & 0xFF);
						radio_buf[i+2]=(uint8_t)((y>>8) & 0xFF);
						radio_buf[i+3]=(uint8_t)(y & 0xFF);
						radio_buf[i+4]=(uint8_t)((z>>8) & 0xFF);
						radio_buf[i+5]=(uint8_t)(z & 0xFF);
					}
					while (!sent){
						wdt_reset();
						sent = radio.TX_bytes(radio_buf, 48 , APPEND_DATA);
					}
					mpu6050_get_FIFO_length(&size);
				}
				radio.sleep();
				wdt_reset();
				mpu6050_FIFO_reset();
				mpu6050_enable_motion_interrupt();
				mpu6050_enable_pin_interrupt();
				mpu6050_lowPower_mode();
				cur_state = IDLE;
				break;
			
				
			case ALIVE_TRANSMIT:
				wdt_reset();
				Leds.reset();
				Leds.turn_on(RED);
				Leds.turn_on(GREEN);
				mpu6050_disable_pin_interrupt();
				AnalogIn.enable();
				wdt_reset();
				radio.wake();
				mpu6050_normalPower_mode();
				mpu6050_getRawAccData(&x,&y,&z);
				wdt_reset();
				alive_timestamp = rtc.get_epoch();
				radio_buf[0]=(uint8_t)((alive_timestamp>>24) & 0xFF);
				radio_buf[1]=(uint8_t)((alive_timestamp>>16) & 0xFF);
				radio_buf[2]=(uint8_t)((alive_timestamp>>8) & 0xFF);
				radio_buf[3]=(uint8_t)(alive_timestamp & 0xFF);
				radio_buf[4] = AnalogIn.get_battery_lvl();
				radio_buf[5] = AnalogIn.get_light_lvl();
				radio_buf[6]=(uint8_t)((x>>8) & 0xFF);
				radio_buf[7]=(uint8_t)(x & 0xFF);
				radio_buf[8]=(uint8_t)((y>>8) & 0xFF);
				radio_buf[9]=(uint8_t)(y & 0xFF);
				radio_buf[10]=(uint8_t)((z>>8) & 0xFF);
				radio_buf[11]=(uint8_t)(z & 0xFF);
				AnalogIn.disable();
				wdt_reset();
				sent = false;
				// Set DR to 5, try to send on the highest and then decrease if fail.
				for (uint8_t DR = 6; DR > 0; DR--){
					radio.set_DR(DR-1);
					for (uint8_t i = 0; i<3;i++){
						sent = radio.TX_bytes(radio_buf, 13, KEEP_ALIVE);
						wdt_reset();
						if (sent){break;}
					}
					if (sent){break;}
				}
				wdt_reset();
				radio.sleep();
				mpu6050_enable_motion_interrupt();
				mpu6050_enable_pin_interrupt();
				mpu6050_lowPower_mode();
				cur_state = IDLE;
				break;
				
			case IDLE:
				wdt_reset();
				wdt_set_to_8s();
				if(gyro_data){
					cur_state = DATA_TRANSMIT;
				}
				else if (rtc.get_alarm_status()){
					cur_state = ALIVE_TRANSMIT;
				}
				else{
					cur_state = SLEEP;
					wdt_reset();
					WDT_off();
				}
				Leds.reset();
				break;
				
			case SLEEP:
				enable_power_down();
				sleep_enable();
				sleep_mode();
				cur_state = IDLE;
				break;
				
			default:
				printf("Something went wrong... \n");
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
	if ((interrupt & (1 << 6)) && !fifo_started){
		fifo_started = true;
		timestamp = rtc.get_epoch();
		mpu6050_normalPower_mode();
		mpu6050_FIFO_reset();
		mpu6050_FIFO_enable();
		mpu6050_enable_FIFO_OVF_interrupt();
		mpu6050_enable_pin_interrupt();
	}
	else {
		mpu6050_FIFO_stop();
		fifo_started = false;
		gyro_data = true;
	}

};

ISR(INT1_vect){
	printf("Dummy interrupt \n");
};

ISR(WDT_vect){
	WDT_off();
	printf("WDT! \n");
};
