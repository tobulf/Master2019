/*
 * CPPFile1.cpp
 *
 * Created: 14.10.2019 17:15:34
 *  Author: Tobias
 */ 

#include "EEPROM.h"

bool writing = false;
bool reading = false;

void EEPROM_init(){
	/*If eeprom has never been used, initialize it by writing zero to all values once.*/
	if (EEPROM_read(EEPROM_is_initialized)==255){
		for(uint16_t i = EEPROM_start; i < EEPROM_end; i++){
		//	EEPROM_write(0 ,i);
		};
		//EEPROM_write(EEPROM_is_initialized,1);
	};
};
void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address and data registers */
	EEAR = uiAddress;
	EEDR = ucData;
	/* Write logical one to EEMWE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEWE */
	EECR |= (1<<EEPE);
}

void EEPROM_write_int16(unsigned int uiAddress, int16_t ucData){
	EEPROM_write(uiAddress ,(uint8_t)((ucData>>8) & 0xFF));
	EEPROM_write(uiAddress+1 ,(uint8_t)(ucData & 0xFF));
}

unsigned char EEPROM_read(unsigned int uiAddress){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	return EEDR;
}
int16_t EEPROM_read_int16(unsigned int uiAddress){
	int16_t temp = (int16_t)((uint16_t)(EEPROM_read(uiAddress) << 8) | (uint8_t)EEPROM_read(uiAddress+1));
	return temp;
}
void EEPROM_reset(){
	for(uint16_t i = 0; i < EEPROM_end; i++){
		EEPROM_write(255,i);
	};
}

