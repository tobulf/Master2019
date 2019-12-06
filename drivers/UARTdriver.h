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

/**
 * @brief Communications object for pheripheral UART.
 * 
 */
class PHERIPHERAL_COM{
public:
	/**
	 * @brief Construct a new pheripheral UART.
	 * 
	 */
	PHERIPHERAL_COM();
	/**
	 * @brief Recieve one char.
	 * 
	 * @return unsigned char 
	 */
	unsigned char receive(void);
	/**
	 * @brief transmit one char.
	 * 
	 * @param data 
	 */
	void transmit( uint8_t data );
	/**
	 * @brief Transmit one whole string of type char buffer.
	 * 
	 * @param string 
	 */
	void putstring(char *string);
protected:

private:
	
};








#endif /* UARTDRIVER_H_ */