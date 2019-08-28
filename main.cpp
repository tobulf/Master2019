/*
 * Master-node.cpp
 *
 * Created: 28.08.2019 12:08:38
 * Author : Tobias
 */ 

#include <util/delay.h>
#include <util/delay_basic.h>
#include <stdbool.h>
#include "RN2483.h"
#include "LED_driver.h"
#include "LoRa_cfg.h"
#include "WString.h"

/* Since FILES, and FDEV doesn't work in C++, a workaround had to be made to enable printf:
   This is considered a bug by the WinAvr team, however has not been fixed.
*/

extern "C" {
	#include "Debug.h"
};

/* Handy macros: */
#define set_bit(reg, bit ) (reg |= (1 << bit))
#define clear_bit(reg, bit ) (reg &= ~(1 << bit))
#define test_bit(reg, bit ) (reg & (1 << bit))

int main (void){
	
	USART_init();
	RN2483 fisk;
	bool something = fisk.init_OTAA(appEui, appKey);
	while (true){
		fisk.TX_bytes(String("7020"),1);
		_delay_ms(30000);
		_delay_ms(30000);
	}

}
