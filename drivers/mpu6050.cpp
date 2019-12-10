/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/


#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Debug.h"
#include "mpu6050.h"

//path to i2c fleury lib
#include "i2cmaster.h"

volatile uint8_t buffer[14];

/*
 * read bytes from chip register
 */
int8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length, uint8_t *data) {
	uint8_t i = 0;
	int8_t count = 0;
	if(length > 0) {
		//request register
		i2c_start(MPU6050_ADDR | I2C_WRITE);
		i2c_write(regAddr);
		_delay_us(10);
		//read data
		i2c_start(MPU6050_ADDR | I2C_READ);
		for(i=0; i<length; i++) {
			count++;
			if(i==length-1){
				data[i] = i2c_readNak();
			}
			else{
				data[i] = i2c_readAck();
			}
		}
		i2c_stop();
	}
	return count;
}

/*
 * read 1 byte from chip register
 */
int8_t mpu6050_readByte(uint8_t regAddr, uint8_t *data) {
    return mpu6050_readBytes(regAddr, 1, data);
}

/*
 * write bytes to chip register
 */
void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data) {
	if(length > 0) {
		//write data
		i2c_start(MPU6050_ADDR | I2C_WRITE);
		i2c_write(regAddr); //reg
		for (uint8_t i = 0; i < length; i++) {
			i2c_write((uint8_t) data[i]);
		}
		i2c_stop();
	}
}

/*
 * write 1 byte to chip register
 */
void mpu6050_writeByte(uint8_t regAddr, uint8_t data) {
    return mpu6050_writeBytes(regAddr, 1, &data);
}

/*
 * read bits from chip register
 */
int8_t mpu6050_readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    int8_t count = 0;
    if(length > 0) {
		uint8_t b;
		if ((count = mpu6050_readByte(regAddr, &b)) != 0) {
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			b &= mask;
			b >>= (bitStart - length + 1);
			*data = b;
		}
    }
    return count;
}

/*
 * read 1 bit from chip register
 */
int8_t mpu6050_readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = mpu6050_readByte(regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/*
 * write bit/bits to chip register
 */
void mpu6050_writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
	if(length > 0) {
		uint8_t b = 0;
		if (mpu6050_readByte(regAddr, &b) != 0) { //get current data
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			mpu6050_writeByte(regAddr, b);
		}
	}
}

/*
 * write one bit to chip register
 */
void mpu6050_writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    mpu6050_readByte(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    mpu6050_writeByte(regAddr, b);
}

/*
 * set sleep disabled
 */
void mpu6050_setSleepDisabled() {
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
	//wake up delay needed sleep disabled
	enable_power_down();
	sleep_enable();
	wdt_enable(WDTO_120MS);
	wdt_INT_enable();
	sleep_mode();
}

/*
 * set sleep enabled
 */
void mpu6050_setSleepEnabled(void) {
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1);
}

void mpu6050_tempSensorDisabled(void){
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, 1);
}

void mpu6050_tempSensorEnabled(void){
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, 0);
}
 /* test connection to chip
 */
uint8_t mpu6050_testConnection(void) {
	mpu6050_readBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, (uint8_t *)buffer);
	if(buffer[0] == 0x34)
		return 1;
	else
		return 0;
}

/*
 * initialize the accel and gyro
 */
void mpu6050_init(void) {
	//allow mpu6050 chip clocks to start up
	enable_power_down();
	sleep_enable();
	wdt_enable(WDTO_120MS);
	wdt_INT_enable();
	sleep_mode();
	//set sleep disabled
	mpu6050_setSleepDisabled();
	// Initialize external interrupt
	mpu6050_init_interrupt();
	//set clock source
	//  it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source)
	//  as the clock reference for improved stability
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	//set DLPF bandwidth to 42Hz
	mpu6050_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
    //set sampe rate
	mpu6050_writeByte(MPU6050_RA_SMPLRT_DIV, 4); //1khz / (1 + 4) = 200Hz
	//set gyro range
	mpu6050_writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS);
	//set accel range
	mpu6050_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS);
	//disable multi master i2c
	mpu6050_writeBits(MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, 1, 0);
	//disable DMP
	mpu6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT,0);
}

void mpu6050_getRawGyroData(int16_t* gx, int16_t* gy, int16_t* gz) {
	/* Read data*/
	mpu6050_readBytes(MPU6050_RA_GYRO_XOUT_H, 6, (uint8_t *)buffer);
	*gx = (((int16_t)buffer[0]) << 8) | buffer[1];
	*gy = (((int16_t)buffer[2]) << 8) | buffer[3];
	*gz = (((int16_t)buffer[4]) << 8) | buffer[5];
}

void mpu6050_getRawAccData(int16_t* ax, int16_t* ay, int16_t* az) {
	/* Read data*/
	mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_H, 6, (uint8_t *)buffer);
	*ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	*az = (((int16_t)buffer[4]) << 8) | buffer[5];
}

void mpu6050_getRawTempData(int16_t* t) {
	mpu6050_readBytes(MPU6050_RA_TEMP_OUT_H, 2, (uint8_t *)buffer);
	*t = (((int16_t)buffer[0]) << 8) | buffer[1];
}

void mpu6050_getConvGyroData(double* axg, double* ayg, double* azg){
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	mpu6050_getRawGyroData(&ax, &ay, &az);
	*axg = (double)(ax-MPU6050_AXOFFSET)/MPU6050_AXGAIN;
	*ayg = (double)(ay-MPU6050_AYOFFSET)/MPU6050_AYGAIN;
	*azg = (double)(az-MPU6050_AZOFFSET)/MPU6050_AZGAIN;
}

void mpu6050_getConvAccData(double* gxds, double* gyds, double* gzds){
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	mpu6050_getRawGyroData(&gx, &gy, &gz);
    *gxds = (double)(gx-MPU6050_GXOFFSET)/MPU6050_GXGAIN;
    *gyds = (double)(gy-MPU6050_GYOFFSET)/MPU6050_GYGAIN;
    *gzds = (double)(gz-MPU6050_GZOFFSET)/MPU6050_GZGAIN;
}

void mpu6050_getConvTempData(double*ta){
	int16_t  t = 0;
	mpu6050_getRawTempData(&t);
	*ta  = (double)(t-MPU6050_TEMPOFFSET)/MPU6050_TEMPGAIN;
}

/* Added a driver for motion detection interrupt. Found on a arduino forum: https://forum.arduino.cc/index.php?topic=364758.0 */

void mpu6050_init_interrupt() {
	/* Enable ext-interrupt ISR0: */
	EICRA |= (1<<ISC00);
	EICRA &= ~(1<<ISC01);
	EIMSK |= (1<<INT0);
	PORTD |= (0<<2);
	/* Motion duration: LSB = 1ms */
	mpu6050_writeByte(MPU6050_RA_MOT_DUR,1);
	/* Motion threshold: 0x20 default, LSB = 4mg */
	mpu6050_writeByte(MPU6050_RA_MOT_THR, 0x20);
	_delay_ms(1);
	/* set HPF to HOLD settings */
	mpu6050_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, 0b111);
	/* Configure interrupt 1: active low */
	mpu6050_writeBits(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 2, 0b10);
}

void mpu6050_disable_interrupt(){
	mpu6050_writeByte(MPU6050_RA_INT_ENABLE,0x00);
}

void mpu6050_enable_interrupt(){
	mpu6050_writeByte(MPU6050_RA_INT_ENABLE,0x40);
}

void mpu6050_set_interrupt_thrshld(uint8_t threshold) {
	/*LSB = 4mg, range: 0 - 1020mg*/
	mpu6050_writeByte(MPU6050_RA_MOT_THR, threshold);
}

void mpu6050_gyroEnabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, 3, 0b000);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, 0);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, 0);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, 0);
}

void mpu6050_gyroDisabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, 3, 0b111);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, 1);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, 1);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, 1);
}

void mpu6050_accEnabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, 3, 0b000);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, 0);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, 0);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, 0);
}

void mpu6050_accDisabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, 3, 0b111);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, 1);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, 1);
	//mpu6050_writeBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, 1);
}

void mpu6050_lowPower_mode(){
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, 0);
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
	mpu6050_accEnabled();
	mpu6050_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, 0);
	mpu6050_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, 0);
	mpu6050_enable_interrupt();
	
	mpu6050_gyroDisabled();
	mpu6050_tempSensorDisabled();
	
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR2_LP_WAKE_CTRL_BIT, 1);
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR2_LP_WAKE_CTRL_BIT-1, 1);
	
	//mpu6050_tempSensorDisabled();
	//mpu6050_writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, 3);
	
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, 1);

	/* Enable accelerometer and disable temp+gyro */
	
}

void mpu6050_normalPower_mode(){
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, 0);
	mpu6050_accEnabled();
	mpu6050_gyroEnabled();
	mpu6050_tempSensorEnabled();
}