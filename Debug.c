/*
 * Debug.c
 *
 * Created: 11.10.2018 13:27:34
 *  Author: tobiasu
 */ 


#include "Debug.h"

#ifndef F_CPU
	#define F_CPU 12000000UL
#endif

#ifndef DEBUG_BAUD
	#define DEBUG_BAUD 57600UL
#endif


int USART_TRANSMIT_printf(char var, FILE *stream) {
	// translate \n to \r for br@y++ terminal
	if (var == '\n') USART_transmit('\r');
	USART_transmit(var);
	return 0;
}



/*Enabling prinf:*/
FILE mystdout = FDEV_SETUP_STREAM(USART_TRANSMIT_printf, NULL, _FDEV_SETUP_RW);



unsigned char USART_receive(void){
	/* Wait for data to be received:*/
	while ( !(UCSR1A & (1<<RXC)) );
	/*Return data from buffer:*/
	return UDR1;
}



void  USART_transmit( uint8_t data ){
	/* Wait for empty transmit buffer:*/
	while ( !( UCSR1A & (1<<UDRE)));
	/* Put data into buffer:*/
	UDR1 = data;
}




void  USART_putstring(char *string){
	for(; *string; ++string){
		USART_transmit(*string);
	}
}




void USART_init(unsigned int BR){
	// Calculate ubbr:
	unsigned int ubrr =(F_CPU / (16*DEBUG_BAUD))-1U;
	/*Set baud rate */
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN)|(1<<TXEN);
	/* Set frame format:  2stop bit, 8data*/
	UCSR1C = (1<<USBS)|(3<<UCSZ0);
	stdout = &mystdout;
	//stdin = &mystdout;
}

