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
#define MPU6050_CALIBRATED 0x01
#define MPU6050_CALIBRATED_AXOFFSET 0x40
#define MPU6050_CALIBRATED_AYOFFSET 0x42
#define MPU6050_CALIBRATED_AZOFFSET 0x44
#define MPU6050_CALIBRATED_TEMPOFFSET 0x46


#endif /* MEMORYADRESSES_H_ */