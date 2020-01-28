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
int MPU6050_AXOFFSET = 675;
int MPU6050_AYOFFSET = -65;
int MPU6050_AZOFFSET = 15200;
int MPU6050_TEMPOFFSET = -9800;
uint8_t interrupt_byte;
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
	_delay_ms(100);
}
/*
 * set sleep enabled
 */
void mpu6050_setSleepEnabled(void) {
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1);
}

void mpu6050_reset(){
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

void mpu6050_tempSensorDisabled(void){
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, 1);
}

void mpu6050_tempSensorEnabled(void){
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, 0);
	_delay_ms(100);
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
	_delay_ms(100);
	//set sleep disabled
	mpu6050_setSleepDisabled();
	//disable gyro and acc:
	mpu6050_tempSensorDisabled();
	mpu6050_gyroDisabled();
	mpu6050_accEnabled();
	// Initialize external interrupt
	mpu6050_init_interrupt();
	//set clock source
	//  it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source)
	//  as the clock reference for improved stability
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	//set DLPF bandwidth to 42Hz
	mpu6050_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
	//set sample rate
	mpu6050_writeByte(MPU6050_RA_SMPLRT_DIV, 49); //1khz / (1 + 49) = 20Hz
	//set gyro range
	mpu6050_writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_2000);
	//set accel range +-2g
	uint8_t cur_sense = EEPROM_read(MPU6050_CUR_SENSITIVITY);
	mpu6050_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, cur_sense);
	//disable multi master i2c
	mpu6050_writeBits(MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, 1, 0);
	//disable DMP
	mpu6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT,0);
	uint8_t calibrated = EEPROM_read(MPU6050_CALIBRATED);
	if (calibrated==1){
		switch (cur_sense){
			case TWO_G:
			MPU6050_AXOFFSET = (int)EEPROM_read_int16(MPU6050_CALIBRATED_AXOFFSET);
			MPU6050_AYOFFSET = (int)EEPROM_read_int16(MPU6050_CALIBRATED_AYOFFSET);
			MPU6050_AZOFFSET = (int)EEPROM_read_int16(MPU6050_CALIBRATED_AZOFFSET);
			MPU6050_TEMPOFFSET = (int)EEPROM_read_int16(MPU6050_CALIBRATED_TEMPOFFSET);
			break;
			case FOUR_G:
			MPU6050_AXOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AXOFFSET))/2;
			MPU6050_AYOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AYOFFSET))/2;
			MPU6050_AZOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AZOFFSET))/2;
			MPU6050_TEMPOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_TEMPOFFSET))/2;
			break;
			case EIGHT_G:
			MPU6050_AXOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AXOFFSET))/4;
			MPU6050_AYOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AYOFFSET))/4;
			MPU6050_AZOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AZOFFSET))/4;
			MPU6050_TEMPOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_TEMPOFFSET))/4;
			break;
			case SIXTEEN_G:
			MPU6050_AXOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AXOFFSET))/8;
			MPU6050_AYOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AYOFFSET))/8;
			MPU6050_AZOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AZOFFSET))/8;
			MPU6050_TEMPOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_TEMPOFFSET))/8;
			break;
			default:
			break;
		}
	}
}

void mpu6050_set_sensitivity(MPU6050_SENSITIVITY sensitivity){
	EEPROM_write(MPU6050_CUR_SENSITIVITY, sensitivity);
	mpu6050_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, sensitivity);
	switch (sensitivity){
		case TWO_G:
		MPU6050_AXOFFSET = (int)EEPROM_read_int16(MPU6050_CALIBRATED_AXOFFSET);
		MPU6050_AYOFFSET = (int)EEPROM_read_int16(MPU6050_CALIBRATED_AYOFFSET);
		MPU6050_AZOFFSET = (int)EEPROM_read_int16(MPU6050_CALIBRATED_AZOFFSET);
		MPU6050_TEMPOFFSET = (int)EEPROM_read_int16(MPU6050_CALIBRATED_TEMPOFFSET);
		break;
		case FOUR_G:
		MPU6050_AXOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AXOFFSET))/2;
		MPU6050_AYOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AYOFFSET))/2;
		MPU6050_AZOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AZOFFSET))/2;
		MPU6050_TEMPOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_TEMPOFFSET))/2;
		break;
		case EIGHT_G:
		MPU6050_AXOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AXOFFSET))/4;
		MPU6050_AYOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AYOFFSET))/4;
		MPU6050_AZOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AZOFFSET))/4;
		MPU6050_TEMPOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_TEMPOFFSET))/4;
		break;
		case SIXTEEN_G:
		MPU6050_AXOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AXOFFSET))/8;
		MPU6050_AYOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AYOFFSET))/8;
		MPU6050_AZOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_AZOFFSET))/8;
		MPU6050_TEMPOFFSET = ((int)EEPROM_read_int16(MPU6050_CALIBRATED_TEMPOFFSET))/8;
		break;
		default:
		break;
	}
};

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
	*axg = (double)(ax-MPU6050_GXOFFSET)/MPU6050_GXGAIN;
	*ayg = (double)(ay-MPU6050_GYOFFSET)/MPU6050_GYGAIN;
	*azg = (double)(az-MPU6050_GZOFFSET)/MPU6050_GZGAIN;
}

void mpu6050_getConvAccData(int16_t* gxds, int16_t* gyds, int16_t* gzds){
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	mpu6050_getRawAccData(&gx, &gy, &gz);
	*gxds = (int16_t)(gx-MPU6050_AXOFFSET)/MPU6050_AXGAIN;
	*gyds = (int16_t)(gy-MPU6050_AYOFFSET)/MPU6050_AYGAIN;
	*gzds = (int16_t)(gz-MPU6050_AZOFFSET)/MPU6050_AZGAIN;
}
void mpu6050_FIFO_enable(){
	mpu6050_writeBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, 1);
	mpu6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT,1);
}

void mpu6050_FIFO_stop(){
	mpu6050_writeBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, 0);
	//delete the 4 first bytes, since OVF means that the first 2 byte is lost.
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
}

void mpu6050_FIFO_disable(){
	mpu6050_writeBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, 0);
	mpu6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT,0);
}

void mpu6050_FIFO_reset(){
	mpu6050_FIFO_disable();
	mpu6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT,1);
}

void mpu6050_get_FIFO_length(uint16_t* length){
	mpu6050_readBytes(MPU6050_RA_FIFO_COUNTH, 2, (uint8_t *)buffer);
	*length = (((int16_t)buffer[0]) << 8) | buffer[1];
}

void mpu6050_FIFO_pop(int16_t* gxds, int16_t* gyds, int16_t* gzds){
	int16_t gtemp = 0;
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = (((int16_t)buffer[0]) << 8);
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = gtemp | buffer[0];
    *gxds = (int16_t)(gtemp-MPU6050_AXOFFSET)/MPU6050_AXGAIN;
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = (((int16_t)buffer[0]) << 8);
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = gtemp | buffer[0];
	*gyds = (int16_t)(gtemp-MPU6050_AYOFFSET)/MPU6050_AYGAIN;
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = (((int16_t)buffer[0]) << 8);
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = gtemp | buffer[0];
	*gzds = (int16_t)(gtemp-MPU6050_AZOFFSET)/MPU6050_AZGAIN;
}

void mpu6050_FIFO_pop_raw(int16_t* gxds, int16_t* gyds, int16_t* gzds){
	int16_t gtemp = 0;
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = (((int16_t)buffer[0]) << 8);
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = gtemp | buffer[0];
	*gxds = gtemp;
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = (((int16_t)buffer[0]) << 8);
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = gtemp | buffer[0];
	*gyds = gtemp;
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = (((int16_t)buffer[0]) << 8);
	mpu6050_readByte(MPU6050_RA_FIFO_R_W, (uint8_t *)buffer);
	gtemp = gtemp | buffer[0];
	*gzds = gtemp;
}


void mpu6050_getConvTempData(int16_t*ta){
	int16_t  t = 0;
	mpu6050_getRawTempData(&t);
	*ta  = (int16_t)(t-MPU6050_TEMPOFFSET)/MPU6050_TEMPGAIN;
}

/* Added a driver for motion detection interrupt. Found on a arduino forum: https://forum.arduino.cc/index.php?topic=364758.0 */

void mpu6050_init_interrupt() {
	/* Enable ext-interrupt ISR0: */
	EICRA &= ~(1<<ISC00);
	EICRA |= (1<<ISC11);
	DDRD  |= (0<<2);
	PORTD |= (0<<2);
	MCUCR &= ~(1<<PUD);
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
	mpu6050_writeByte(MPU6050_RA_INT_ENABLE,interrupt_byte);
}

void mpu6050_disable_pin_interrupt(){
	EIMSK &= ~(1<<INT0);
}
void mpu6050_enable_pin_interrupt(){
	EIMSK |= (1<<INT0);
}

void mpu6050_enable_motion_interrupt(){
	mpu6050_writeByte(MPU6050_RA_INT_ENABLE, 0x40);
	interrupt_byte = 0x40;
}

void mpu6050_enable_data_rdy_interrupt(){
	mpu6050_writeByte(MPU6050_RA_INT_ENABLE,0x01);
	interrupt_byte = 0x01;
}

void mpu6050_enable_FIFO_OVF_interrupt(){
	mpu6050_writeByte(MPU6050_RA_INT_ENABLE,0x10);
	interrupt_byte = 0x10;
}


uint8_t mpu6050_get_interrupt_status(){
	uint8_t status = 0;
	mpu6050_readByte(MPU6050_RA_INT_STATUS, &status);
	return status;
}

void mpu6050_set_interrupt_mot_thrshld(uint8_t threshold) {
	/*LSB = 4mg, range: 0 - 1020mg*/
	mpu6050_writeByte(MPU6050_RA_MOT_THR, threshold);
}

void mpu6050_set_interrupt_mot_dur(uint8_t duration) {
	/*LSB = 1ms, range: 0 - 255ms*/
	mpu6050_writeByte(MPU6050_RA_MOT_DUR, duration);
}


void mpu6050_gyroEnabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, 3, 0b000);
}

void mpu6050_gyroDisabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, 3, 0b111);
}

void mpu6050_accEnabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, 3, 0b000);
}

void mpu6050_accDisabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, 3, 0b111);
}

void mpu6050_accZEnabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, 1, 0b0);
}

void mpu6050_accZDisabled(){
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, 1, 0b1);
}

void mpu6050_lowPower_mode(){
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, 1);
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_2,MPU6050_PWR2_LP_WAKE_CTRL_BIT,MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, 0b11);
}

void mpu6050_normalPower_mode(){
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, 0);
	mpu6050_writeByte(MPU6050_RA_SMPLRT_DIV, 49); //1khz / (1 + 49) = 20Hz
	mpu6050_tempSensorEnabled();
}