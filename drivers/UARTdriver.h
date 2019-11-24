/*
 * UARTdriver.h
 *
 * Created: 05.10.2018 18:13:56
 *  Author: tobiasu
 */ 


#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_


#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>

/* Comms to Pheripheral:*/
class PHERIPHERAL_COM{
public:
	PHERIPHERAL_COM();
	unsigned char receive(void);
	void transmit( uint8_t data );
	void putstring(char *string);
protected:

private:
	
};








#endif /* UARTDRIVER_H_ */