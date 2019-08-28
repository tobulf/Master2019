/*
 * UARTdriver.cpp
 *
 * Created: 05.10.2018 18:13:16
 *  Author: tobiasu
 */ 

#include "UARTdriver.h"


#define PHERIPHERAL_BAUD 57600UL

#define LF (uint8_t)10
#define CR (uint8_t)13



// Peripheral UART-Communication
unsigned char PHERIPHERAL_COM::receive(void){
	/* Wait for data to be received:*/
	while ( !(UCSR2A & (1<<RXC)) );
	/*Return data from buffer:*/
	return UDR2;
};


void PHERIPHERAL_COM::putstring(char *string){
	for(; *string; ++string){
		transmit(*string);
	}
	/*Terminate using CR-LF*/
	transmit(CR);
	transmit(LF);
};



void PHERIPHERAL_COM::transmit(uint8_t data){
	/* Wait for empty transmit buffer:*/
	while ( !( UCSR2A & (1<<UDRE)) );
	/* Put data into buffer, sends the data:*/
	UDR2 = data;
};


PHERIPHERAL_COM::PHERIPHERAL_COM(){
	/*Calculate ubbr: */
	unsigned int ubrr = (F_CPU/(16*PHERIPHERAL_BAUD))-1U;
	/*Set baud rate */
	UBRR2H = (unsigned char)(ubrr>>8);
	UBRR2L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR2B = (1<<RXEN)|(1<<TXEN);
	/* Set frame format:  2stop bit, 8data*/
	UCSR2C = (1<<USBS)|(3<<UCSZ0);
};

