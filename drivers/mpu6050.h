/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.

References:
  - most of the code is a port of the arduino mpu6050 library by Jeff Rowberg
    https://github.com/jrowberg/i2cdevlib
  - Mahony complementary filter for attitude estimation
    http://www.x-io.co.uk
*/


#ifndef MPU6050_H_
#define MPU6050_H_

#include <avr/io.h>
#include "mpu6050registers.h"
#include "EEPROM.h"
#include "MemoryAdresses.h"

#ifdef __cplusplus
extern "C" {
#endif

//i2c settings
#define MPU6050_I2CFLEURYPATH "i2cmaster.h" //define the path to i2c fleury lib
#define MPU6050_I2CINIT 0 //init i2c

//definitions
#define MPU6050_ADDR (0x68 <<1) //device address - 0x68 pin low (GND), 0x69 pin high (VCC)

//enable the getattitude functions
//because we do not have a magnetometer, we have to start the chip always in the same position
//then to obtain your object attitude you have to apply the aerospace sequence
//0 disabled
//1 mahony filter
//2 dmp chip processor
#define MPU6050_GETATTITUDE 0


//#define MPU6050_GYRO_LSB_250 131.0
//#define MPU6050_GYRO_LSB_500 65.5
//#define MPU6050_GYRO_LSB_1000 32.8
//#define MPU6050_GYRO_LSB_2000 16.4
//
//
//#define MPU6050_ACCEL_LSB_2 16384.0
//#define MPU6050_ACCEL_LSB_4 8192.0
//#define MPU6050_ACCEL_LSB_8 4096.0
//#define MPU6050_ACCEL_LSB_16 2048.0


// Set gain for calibration

#define MPU6050_AXGAIN 1//16.384
#define MPU6050_AYGAIN 1//16.384
#define MPU6050_AZGAIN 1//16.384
#define MPU6050_GXOFFSET 0
#define MPU6050_GYOFFSET 0
#define MPU6050_GZOFFSET 0
#define MPU6050_GXGAIN 16.4
#define MPU6050_GYGAIN 16.4
#define MPU6050_GZGAIN 16.4
#define MPU6050_TEMPGAIN 340


//functions
extern void mpu6050_init(void);
extern uint8_t mpu6050_testConnection(void);


void mpu6050_getRawGyroData(int16_t* gx, int16_t* gy, int16_t* gz);
void mpu6050_getRawAccData(int16_t* ax, int16_t* ay, int16_t* az);
void mpu6050_getRawTempData(int16_t* t);
void mpu6050_getConvGyroData(double* axg, double* ayg, double* azg);
void mpu6050_getConvAccData(int16_t* gxds, int16_t* gyds, int16_t* gzds);
void mpu6050_FIFO_enable(void);
void mpu6050_FIFO_disable(void);
void mpu6050_FIFO_stop();
void mpu6050_FIFO_reset(void);
void mpu6050_get_FIFO_length(uint16_t* length);
void mpu6050_FIFO_pop(int16_t* gxds, int16_t* gyds, int16_t* gzds);
void mpu6050_FIFO_pop_raw(int16_t* gxds, int16_t* gyds, int16_t* gzds);
void mpu6050_getConvTempData(int*ta);

extern void mpu6050_setSleepDisabled(void);
extern void mpu6050_setSleepEnabled(void);
void mpu6050_reset(void);
void mpu6050_tempSensorDisabled(void);
void mpu6050_tempSensorEnabled(void);

extern int8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
extern int8_t mpu6050_readByte(uint8_t regAddr, uint8_t *data);
extern void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data);
extern void mpu6050_writeByte(uint8_t regAddr, uint8_t data);
extern int8_t mpu6050_readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
extern int8_t mpu6050_readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);
extern void mpu6050_writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
extern void mpu6050_writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
void mpu6050_init_interrupt(void);
void mpu6050_set_interrupt_mot_thrshld(uint8_t threshold);
void mpu6050_set_interrupt_mot_dur(uint8_t duration);
void mpu6050_enable_motion_interrupt(void);
void mpu6050_enable_FIFO_OVF_interrupt(void);
void mpu6050_enable_data_rdy_interrupt(void);
uint8_t mpu6050_get_interrupt_status(void);
void mpu6050_disable_interrupt(void);
void mpu6050_enable_interrupt(void);
void mpu6050_disable_pin_interrupt(void);
void mpu6050_enable_pin_interrupt(void);
void mpu6050_gyroEnabled(void);
void mpu6050_gyroDisabled(void);
void mpu6050_accEnabled(void);
void mpu6050_accDisabled(void);
void mpu6050_accZEnabled(void);
void mpu6050_accZDisabled(void);
void mpu6050_lowPower_mode(void);
void mpu6050_normalPower_mode(void);

#ifdef __cplusplus
}
#endif

#endif
