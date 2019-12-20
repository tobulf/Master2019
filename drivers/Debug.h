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
#include <inttypes.h>


/*Uart for debug:*/
/**
 * @brief Init UART for write operations.
 * 
 */
void USART_init();
/**
 * @brief Bridge function to connect UART to stream.
 * 
 * @param var 
 * @param stream 
 * @return int 
 */
int USART_TRANSMIT_printf(char var, FILE *stream);
/**
 * @brief Receive one char.
 * 
 * @return unsigned char 
 */
unsigned char USART_receive(void);
/**
 * @brief Transmit one char.
 * 
 * @param data 
 */
void USART_transmit( uint8_t data );
/**
 * @brief Transmit a whole string of type char buffer.
 * 
 * @param string 
 */
void USART_putstring(char *string);







#endif /* DEBUG_H_ */