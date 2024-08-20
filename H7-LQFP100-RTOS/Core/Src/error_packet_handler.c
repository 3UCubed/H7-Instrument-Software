/*
 * error_packet_handler.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "error_packet_handler.h"
#include "usart.h"
#include <stdio.h>				// For uint data types


void handle_error(ERROR_CODE error_code){
	switch (error_code) {
	case EC_vsense:
	case EC_vrefint:
	case EC_TEMP1:
	case EC_TEMP2:
	case EC_TEMP3:
	case EC_TEMP4:
	case EC_busvmon:
	case EC_busimon:
	case EC_2v5:
	case EC_3v3:
	case EC_5v:
	case EC_n3v3:
	case EC_n5v:
	case EC_15v:
	case EC_5vref:
	case EC_n200v:
	case EC_n800v:
	case EC_TMP1:
		// TODO: system reset?
		send_error_packet(error_code);
		break;
	case EC_single_bit_error:
		// TODO: figure out what steps we want to take for SBE
		send_error_packet(error_code);
		break;
	case EC_double_bit_error:
		// TODO: figure out what steps we want to take for DBE
		send_error_packet(error_code);
		break;
	case EC_peripheral_error:
		// TODO: system reset?
		send_error_packet(error_code);
		break;
	case EC_UNKNOWN:
		send_error_packet(error_code);
		break;
	default:
		break;
	}
}

void send_error_packet(ERROR_CODE error_code) {
	uint8_t buffer[ERROR_PACKET_SIZE];

	buffer[0] = ERROR_PACKET_SYNC;
	buffer[1] = ERROR_PACKET_SYNC;
	buffer[2] = error_code;

	HAL_UART_Transmit(&huart1, buffer, ERROR_PACKET_SIZE, 100);
	send_junk_packet();
}

void send_junk_packet() {
	uint8_t buffer[JUNK_PACKET_SIZE];

	for (int i = 0; i < JUNK_PACKET_SIZE; i++) {
		buffer[i] = 0xEE;
	}
	HAL_UART_Transmit(&huart1, buffer, JUNK_PACKET_SIZE, 100);
}







