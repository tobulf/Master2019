/*
 * SPI_driver.cpp
 *
 * Created: 17.09.2019 10:54:48
 *  Author: Tobias
 */ 

#include "SPI_driver.h"


SPI::SPI(){
	/* Set MOSI and SCK output, all others input */
	DDRB = (1<<DDB5)|(1<<DDB7);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR0 = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
};

