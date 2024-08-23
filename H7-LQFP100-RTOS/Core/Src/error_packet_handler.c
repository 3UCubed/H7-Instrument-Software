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

	switch (error.category) {
	case EC_power_supply_rail:
		osEventFlagsSet(mode_event_flags, IDLE_FLAG);
		increment_error_counter(error.category);
		send_error_packet(error);
		NVIC_SystemReset();
		break;
	case EC_seu:
		increment_error_counter(error.category);
		send_error_packet(error);
		NVIC_SystemReset();
		break;
	case EC_peripheral:
		increment_error_counter(error.category);
		send_error_packet(error);
		NVIC_SystemReset();
		break;
	case EC_brownout:
		increment_error_counter(error.category);
		send_error_packet(error);
		break;
	case EC_watchdog:
		increment_error_counter(error.category);
		send_error_packet(error);
		break;
	default:
		send_error_packet(error);
		break;
	}
}

void error_counter_init() {
	HAL_FLASH_Unlock();
	if (EE_Init() != EE_OK) {
		Error_Handler();
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
	if ((EE_ReadVariable(VirtAddVarTab[category], &retval)) != HAL_OK) {
		Error_Handler();
	}
	return retval;
}

void set_eeprom_error_counter(ERROR_CATEGORY category,
		uint16_t new_counter_value) {
	VarDataTab[category] = new_counter_value;
	if ((EE_WriteVariable(VirtAddVarTab[category], VarDataTab[category]))
			!= HAL_OK) {
		Error_Handler();
	}
}

void reset_eeprom_error_counters() {
	for (int i = 0; i < NB_OF_VAR; i++) {
		if ((EE_WriteVariable(VirtAddVarTab[i], 0x00)) != HAL_OK) {
			Error_Handler();
		}
	}
}

void send_error_packet(ERROR_STRUCT error) {
	uint8_t buffer[ERROR_PACKET_SIZE];

	buffer[0] = ERROR_PACKET_SYNC;
	buffer[1] = ERROR_PACKET_SYNC;
	buffer[2] = error.category;
	buffer[3] = error.detail;

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

