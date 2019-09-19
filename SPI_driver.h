/*
 * SPI_driver.h
 *
 * Created: 17.09.2019 10:55:00
 *  Author: Tobias
 */ 


#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include <avr/io.h>

/* Handy macros: */
#define set_bit(reg, bit ) (reg |= (1 << bit))
#define clear_bit(reg, bit ) (reg &= ~(1 << bit))
#define test_bit(reg, bit ) (reg & (1 << bit))


class SPI
{
	public:
	SPI();
	void write(uint8_t data);
	uint8_t read(void);
	void chip_select(uint8_t select);
	protected:
	private:
};



#endif /* SPI_DRIVER_H_ */