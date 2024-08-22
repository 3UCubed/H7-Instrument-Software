/*
 * error_packet_handler.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "error_packet_handler.h"
#include "usart.h"
#include <stdio.h>				// For uint data types


void handle_error(ERROR_STRUCT error){
	switch (error.detail) {
	case ED_vsense:
	case ED_vrefint:
	case ED_TEMP1:
	case ED_TEMP2:
	case ED_TEMP3:
	case ED_TEMP4:
	case ED_busvmon:
	case ED_busimon:
	case ED_2v5:
	case ED_3v3:
	case ED_5v:
	case ED_n3v3:
	case ED_n5v:
	case ED_15v:
	case ED_5vref:
	case ED_n200v:
	case ED_n800v:
	case ED_TMP1:
		// TODO: system reset?
		send_error_packet(error);
		break;
	case ED_single_bit_error:
		// TODO: figure out what steps we want to take for SBE
		send_error_packet(error);
		break;
	case ED_double_bit_error:
		// TODO: figure out what steps we want to take for DBE
		send_error_packet(error);
		break;
	case ED_UNDEFINED:
		send_error_packet(error);
		break;
	default:
		break;
	}
}

void send_error_packet(ERROR_STRUCT error) {
	uint8_t buffer[ERROR_PACKET_SIZE];

	buffer[0] = ERROR_PACKET_SYNC;
	buffer[1] = ERROR_PACKET_SYNC;
	buffer[2] = error.detail;

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







