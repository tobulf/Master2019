/*
 * Debug.h
 *
 * Created: 11.10.2018 13:27:46
 *  Author: tobiasu
 */ 


#ifndef DEBUG_H_
#define DEBUG_H_



#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>


/*Uart for debug:*/

void USART_init();
int USART_TRANSMIT_printf(char var, FILE *stream);
unsigned char USART_receive(void);
void USART_transmit( uint8_t data );
void USART_putstring(char *string);







#endif /* DEBUG_H_ */