/*
 * utils.cpp
 *
 * Created: 13.12.2019 18:02:50
 *  Author: Tobias
 */ 
#include "utils.h"
#include "drivers/Debug.h"

void convert_sync_response(uint8_t* buf, uint64_t &timestamp, uint32_t t_callback){
	timestamp = (((uint64_t)0x00) << 56)
	|(((uint64_t)buf[0]) << 48)
	|(((uint64_t)buf[1]) << 40)
	|(((uint64_t)buf[2]) << 32)
	|(((uint64_t)buf[3]) << 24)
	|(((uint64_t)buf[4]) << 16)
	|(((uint64_t)buf[5]) << 8)
	| buf[6];
	uint32_t temp = (((uint32_t)0) << 24)
	|(((uint32_t)buf[7]) << 16)
	|(((uint32_t)buf[8]) << 8) 
	| buf[9];
	temp = temp + ((uint32_t)buf[10]*1000);
	uint32_t downlinktime = t_callback - temp - 750000;
	timestamp = timestamp + downlinktime;
}

