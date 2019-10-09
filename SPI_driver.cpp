/*
 * SPI_driver.cpp
 *
 * Created: 17.09.2019 10:54:48
 *  Author: Tobias
 */ 

#include "SPI_driver.h"
extern "C" {
	#include "Debug.h"
};
SPI::SPI(){
	/* Set MOSI and SCK output, all others input */
	DDRB = (1<<DDB4) | (1<<DDB5)| (0<<DDB6) |(1<<DDB7);
	/* Enable SPI, Master, set clock rate fck/4 */
	SPCR0 = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(1<<SPR0);
	SPSR0 = (1<<SPI2X);
};

uint8_t SPI::transmit(uint8_t data){
	/* Start transmission */
	SPDR0 = data;
	/* Wait for transmission complete */
	while(!(SPSR0 & (1<<SPIF)));
	return SPDR0;
}

void SPI::chip_select(uint8_t select){
	if(select){
		set_bit(PORTB, PB4);
	}
	
	else{
		clear_bit(PORTB, PB4);
	}
};
	
	