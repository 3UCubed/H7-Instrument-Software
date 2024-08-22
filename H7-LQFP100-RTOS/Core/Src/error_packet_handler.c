/*
 * error_packet_handler.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "error_packet_handler.h"
#include "usart.h"
#include "main.h"
#include "eeprom.h"

uint16_t VirtAddVarTab[NB_OF_VAR] = { 0x5555, 0x6666, 0x7777, 0x8888, 0x9999 };
uint16_t VarDataTab[NB_OF_VAR] = { 0, 0, 0, 0, 0 };

void handle_error(ERROR_STRUCT error) {
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
		increment_error_counter(error.category);
		break;
	case ED_single_bit_error:
		// TODO: figure out what steps we want to take for SBE
		break;
	case ED_double_bit_error:
		// TODO: figure out what steps we want to take for DBE
		break;
	case ED_UNDEFINED:
		// TODO: send error packet?
		break;
	default:
		break;
	}
}

void increment_error_counter(ERROR_CATEGORY category) {
	uint16_t counter_value;
	counter_value = get_eeprom_error_counter(category);
	counter_value++;
	set_eeprom_error_counter(category, counter_value);

}

uint16_t get_eeprom_error_counter(ERROR_CATEGORY category) {
	uint16_t retval = 0;
	HAL_FLASH_Unlock();
	if (EE_Init() != EE_OK) {
		Error_Handler();
	}
	if ((EE_ReadVariable(VirtAddVarTab[category], &retval)) != HAL_OK) {
		Error_Handler();
	}
	return retval;
}

void set_eeprom_error_counter(ERROR_CATEGORY category, uint16_t new_counter_value) {
	if ((EE_WriteVariable(VirtAddVarTab[category], new_counter_value))
			!= HAL_OK) {
		Error_Handler();
	}
}


void reset_eeprom_error_counters() {
	for (int i = 0; i < NB_OF_VAR; i++) {
		if ((EE_WriteVariable(VirtAddVarTab[i], 0))
				!= HAL_OK) {
			Error_Handler();
		}
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

