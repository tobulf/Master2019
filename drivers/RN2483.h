
/**
 * @file RN2483.h
 * @author Tobias U. Rasmussen (tobulf@gmail.com)
 * @brief A simple driver for the RN2483 chip.
 * @date 2019-03-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */


#ifndef RN2483_H_
#define RN2483_H_

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "WString.h"



/**
 * @brief Communications class using UART. 
 * 
 */
class LoRa_COM{
	public:
	/**
	 * @brief Construct a new LoRa_COM object
	 * 
	 */
	LoRa_COM();
	/**
	 * @brief Returns the corresponding answer to a command. 
	 * 
	 * @return String 
	 */
	String get_answer(void);
	/**
	 * @brief Receives a single byte via UART.
	 * 
	 * @return unsigned char 
	 */
	unsigned char receive(void);
	/**
	 * @brief Transmit a single byte via UART.
	 * 
	 * @param uint8_t data 
	 */
	void transmit(uint8_t data);
	/**
	 * @brief Send a command via UART.
	 * 
	 * @param String command 
	 */
	void send_command(String command);
	/**
	 * @brief Flush UART.
	 * 
	 */
	void UART_flush(void);
	void enable_RX_int(void);
	void disable_RX_int(void);
	private:
};

/**
 * @brief Driver Class for the RN2483, functionality for LoRa class A.
 * 
 */

class RN2483: public LoRa_COM {
	public:
	/**
	 * @brief Construct a new RN2483 object
	 * 
	 */
	RN2483();
	/**
	 * @brief Get the HW-version of the RN2483 chip.
	 * 
	 * @return String 
	 */
	String get_version(void);
	/**
	 * @brief asserts the response from the RN2483 chip, if invalid, returns false.
	 * 
	 * @param response 
	 * @return true 
	 * @return false 
	 */
	bool assert_response(String response);
	/**
	 * @brief Initialize over the air activation according to LoRa Standards.
	 * 
	 * @param app_EUI 
	 * @param app_key 
	 * @return true 
	 * @return false 
	 */
	bool init_OTAA(String app_EUI, String app_key);
	/**
	 * @brief Set Datarate for the Transmission(0-5). SF12-SF7. 
	 * 
	 * @param DR 
	 * @return true 
	 * @return false 
	 */
	bool set_DR(uint8_t DR);
	/**
	 * @brief Set duty cycle for the device according to dcycle=(100/x)-1 (x < 1% per EU regulation).
	 * 
	 * @param channel
	 * @param dcycle
	 * @return true
	 * @return false
	 */
	bool set_duty_cycle(uint8_t channel, uint16_t dcycle);
	/**
	 * @brief Set the size of receive window 1.
	 * 
	 * @param milliseconds 
	 * @return true 
	 * @return false 
	 */

	bool set_RX_window_size(uint16_t milliseconds);
	
	
	/**
	 * @brief Transmit a String of data with a specific port(1-255).
	 * 
	 * @param data 
	 * @param port
	 * @return String 
	 */
	
	String TX_string(String data, uint8_t port);
	

	private:
	/**
	 * @brief Support function, turns a char into a hex representation of the byte, in String format(ex: "A" = "41").
	 * 
	 * @param character 
	 * @return String 
	 */
	String char_to_hex(uint8_t character);

	
};






#endif /* RN2483_H_ */