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

/**
 * @brief Initialize EEPROM
 * 
 */
void EEPROM_init();
/**
 * @brief EEPROM Reset all EEPROM-register to standard value(255).
 * 
 */
void EEPROM_reset();
/**
 * @brief Write ucData to uiAddress
 * 
 * @param uiAddress 
 * @param ucData 
 */
void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
void EEPROM_write_int16(unsigned int uiAddress, int16_t ucData);

/**
 * @brief read from uiAdress return unsigned char data.
 * 
 * @param uiAddress 
 * @return unsigned char 
 */
unsigned char EEPROM_read(unsigned int uiAddress);

int16_t EEPROM_read_int16(unsigned int uiAddress);



#endif /* EEPROM_H_ */