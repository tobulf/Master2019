/*
 * EEPROM.h
 *
 * Created: 14.10.2019 17:16:03
 *  Author: Tobias
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

#include <avr/io.h>
#include <stdbool.h>
#include "MemoryAdresses.h"

void EEPROM_init();
void EEPROM_reset();
void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);



#endif /* EEPROM_H_ */