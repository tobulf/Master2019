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

enum STATE {ACTIVE=1, SLEEP, DMY_EVENT, ACC_EVENT};
STATE cur_state;
STATE prev_state;

bool joined = false;
bool acc_buf_full = false;
int16_t acc_data;
uint8_t acc_sample = 0;
int16_t acc_data_buf[20];
uint8_t test_arr[6] = {1,2,3,4,5,6};

adc AnalogIn;
LED_driver Leds;

RTC rtc;
Timer timer;
EventQueue queue;

void handle_acc_event(){
	Leds.turn_on(YELLOW);
	_delay_ms(10);
	mpu6050_enable_data_rdy_interrupt();
	uint32_t timestamp = 0/*rtc.get_epoch()*/;
	mpu6050_enable_pin_interrupt();
	timer.start();
	while(!acc_buf_full){
// 		if (queue.length()>=2){
// 				Leds.toogle(RED);
// 				radio.set_RX_window_size(0);
// 				uint8_t* data = queue.pop_event();
// 				queue.pop_event();
// 				radio.TX_bytes(data, 88, 1);
// 				Leds.toogle(RED);
// 				radio.set_RX_window_size(1000);
// 		}
	};
	
	acc_buf_full = false;
	queue.push_event(timestamp, acc_data_buf);
	timer.stop();
	STATE temp = cur_state;
	cur_state = prev_state;
	prev_state = temp;
	printf("Sample: ");
	for (uint8_t i = 0; i<20;i++){
		printf("%i ",acc_data_buf[i]);
	}
	printf("us: %lu\n",timer.read_us());
	printf("Events left: %d \n", queue.length());
	//for (uint8_t n = 0; n<44;n++){
	//	printf("%d ", data[n]);
	//}
	//printf("\n");
	timer.reset();
	mpu6050_enable_motion_interrupt();
	mpu6050_enable_pin_interrupt();
}

int main (void){
	sei();
	Leds.toogle(RED);
	USART_init();
	RN2483 radio;
	mpu6050_init();
	joined = radio.init_OTAA(appEui,appKey);
	if (!joined){
		Leds.toogle(YELLOW);
	}
	radio.set_DR(5);
	radio.set_duty_cycle(1, 0);
	radio.TX_bytes(test_arr,6,1);
	//mpu6050_lowPower_mode();
	mpu6050_normalPower_mode();
	mpu6050_set_interrupt_mot_thrshld(25);
	mpu6050_enable_motion_interrupt();
	mpu6050_enable_pin_interrupt();
	cur_state = ACTIVE;
	prev_state = ACTIVE;
	Leds.toogle(RED);
	while (true){
		switch (cur_state){
			case ACTIVE:
				Leds.turn_on(GREEN);
				if (prev_state!=ACTIVE){
					//wait, might be more acc_events...
					_delay_ms(1000);
					prev_state = ACTIVE;
					Leds.reset();
					break;
				}
				else{
					_delay_ms(1);
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
				handle_acc_event();
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
	mpu6050_disable_interrupt();
	uint8_t interrupt = mpu6050_get_interrupt_status();
	if ((interrupt & (1 << 6)) && cur_state!=ACC_EVENT){
		prev_state = cur_state;
		cur_state = ACC_EVENT;
	}
	else if(interrupt!=0){
		if (acc_sample<20){
			mpu6050_getConvAccData(&acc_data);
			acc_data_buf[acc_sample] = acc_data;
			acc_sample++;
			mpu6050_enable_interrupt();
			mpu6050_enable_pin_interrupt();
		}
		else{
			acc_sample = 0;
			acc_buf_full = true;
			mpu6050_disable_pin_interrupt();
		}
	}
	else{
		mpu6050_enable_pin_interrupt();
		mpu6050_enable_interrupt();
	}

};

ISR(INT1_vect){
	printf("Dummy interrupt \n");
};
ISR(WDT_vect){
	wdt_disable();
	sleep_disable();
};
