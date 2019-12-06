/*
 * MemoryAdresses.h
 *
 * Created: 14.10.2019 17:40:55
 *  Author: Tobias
 */ 

/**
 * @brief manualy defined memory adresses for EEPROM.
 * 
 */

#ifndef MEMORYADRESSES_H_
#define MEMORYADRESSES_H_
/*EEPROM MEMORY ADRESSES, 0 - 1023 or 0x00 - 0x3FF */
/* define start and end of eeprom */
#define EEPROM_start 0x01
#define EEPROM_end 0x02
/* Define register names */
#define EEPROM_is_initialized 0x00
#define DEVICE_IS_JOINED 0x01
#define MCU_REBOOTED 0x02



#endif /* MEMORYADRESSES_H_ */