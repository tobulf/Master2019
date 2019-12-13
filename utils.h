/*
 * utils.h
 *
 * Created: 13.12.2019 18:03:15
 *  Author: Tobias
 */ 


#ifndef UTILS_H_
#define UTILS_H_
#include <avr/io.h>
//#include "drivers/Debug.h"
void convert_downlink(uint8_t* buf, uint64_t &timestamp, uint32_t &t_callback);



#endif /* UTILS_H_ */