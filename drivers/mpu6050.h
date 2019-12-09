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
#include "power_management.h"
#include "WDT.h"

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

//definitions for raw data
//gyro and acc scale
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_2000
#define MPU6050_ACCEL_FS MPU6050_ACCEL_FS_16

#define MPU6050_GYRO_LSB_250 131.0
#define MPU6050_GYRO_LSB_500 65.5
#define MPU6050_GYRO_LSB_1000 32.8
#define MPU6050_GYRO_LSB_2000 16.4
#if MPU6050_GYRO_FS == MPU6050_GYRO_FS_250
#define MPU6050_GGAIN MPU6050_GYRO_LSB_250
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_500
#define MPU6050_GGAIN MPU6050_GYRO_LSB_500
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_1000
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_2000
#endif

#define MPU6050_ACCEL_LSB_2 16384.0
#define MPU6050_ACCEL_LSB_4 8192.0
#define MPU6050_ACCEL_LSB_8 4096.0
#define MPU6050_ACCEL_LSB_16 2048.0
#if MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_2
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_4
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_8
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_16
#endif

// Set gain for calibration
#define MPU6050_AXOFFSET 0
#define MPU6050_AYOFFSET 2048
#define MPU6050_AZOFFSET -2048
#define MPU6050_AXGAIN 2048.0
#define MPU6050_AYGAIN 2048.0
#define MPU6050_AZGAIN 2048.0
#define MPU6050_GXOFFSET -33
#define MPU6050_GYOFFSET -33
#define MPU6050_GZOFFSET -8
#define MPU6050_GXGAIN 16.4
#define MPU6050_GYGAIN 16.4
#define MPU6050_GZGAIN 16.4
#define MPU6050_TEMPOFFSET -9800
#define MPU6050_TEMPGAIN 340

//functions
extern void mpu6050_init(void);
extern uint8_t mpu6050_testConnection(void);


void mpu6050_getRawGyroData(int16_t* gx, int16_t* gy, int16_t* gz);
void mpu6050_getRawAccData(int16_t* ax, int16_t* ay, int16_t* az);
void mpu6050_getRawTempData(int16_t* t);
void mpu6050_getConvGyroData(double* axg, double* ayg, double* azg);
void mpu6050_getConvAccData(double* gxds, double* gyds, double* gzds);
void mpu6050_getConvTempData(double*ta);

extern void mpu6050_setSleepDisabled(void);
extern void mpu6050_setSleepEnabled(void);
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
void mpu6050_set_interrupt_thrshld(uint16_t threshold);
void mpu6050_enable_interrupt(void);
void mpu6050_disable_interrupt(void);

#ifdef __cplusplus
}
#endif

#endif
