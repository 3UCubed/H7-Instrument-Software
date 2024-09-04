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

uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5550, 0x5551, 0x5552, 0x5553, 0x5554, 0x5555, 0x5556, 0x5557, 0x5558, 0x5559, 0x555A, 0x555B, 0x555C, 0x555D, 0x555E, 0x555F, 0x6660, 0x6661, 0x6662, 0x6663, 0x6664, 0x6665, 0x6666, 0x6667, 0x6668, 0x6669, 0x666A, 0x666B, 0x666C};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint16_t local_cpy[NUM_ERROR_COUNTERS];

void handle_error(ERROR_STRUCT error) {

	increment_error_counter(error);
	set_previous_error(error);


//	switch (error.category) {
//	case EC_power_supply_rail:
//		osEventFlagsSet(mode_event_flags, IDLE_FLAG);
//		increment_error_counter(error);
////		osEventFlagsSet(mode_event_flags, IDLE_FLAG);
//
//		break;
//	case EC_seu:
//		increment_error_counter(error);
//
//		break;
//	case EC_peripheral:
//		increment_error_counter(error);
//
//		break;
//	case EC_brownout:
//		increment_error_counter(error);
//		break;
//	case EC_watchdog:
//		increment_error_counter(error);
//		break;
//	default:
//		break;
//	}
}

void error_counter_init() {
	// Starting up EEPROM Emulator
	HAL_FLASH_Unlock();
	if (EE_Init() != EE_OK) {
		Error_Handler();
	}

	// Updating our local copy of error counters from EE
	for (int i = 0; i < NUM_ERROR_COUNTERS; i++) {
		if ((EE_ReadVariable(VirtAddVarTab[i], &local_cpy[i])) != HAL_OK) {
			Error_Handler();
		}
	}
}


void increment_error_counter(ERROR_STRUCT error) {
	local_cpy[error.category]++;
	local_cpy[error.detail]++;
	update_error_counter();
}




void update_error_counter(){
	// Writes our local copy of the error counters to EE
	for (int i = 0; i < NUM_ERROR_COUNTERS; i++) {
		if ((EE_WriteVariable(VirtAddVarTab[i], local_cpy[i])) != HAL_OK) {
			Error_Handler();
		}
	}
}


void reset_error_counters() {
	// Resets all error counters to 0
	for (int i = 0; i < NUM_ERROR_COUNTERS; i++) {
		if ((EE_WriteVariable(VirtAddVarTab[i], 0)) != HAL_OK) {
			Error_Handler();
		}
	}
}

void reset_previous_error() {
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_CATEGORY_INDEX], 0xFF)) != HAL_OK) {
		Error_Handler();
	}
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_DETAIL_INDEX], 0xFF)) != HAL_OK) {
		Error_Handler();
	}
}

void set_previous_error(ERROR_STRUCT error) {
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_CATEGORY_INDEX], error.category)) != HAL_OK) {
		Error_Handler();
	}
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_DETAIL_INDEX], error.detail)) != HAL_OK) {
		Error_Handler();
	}
}

ERROR_STRUCT get_previous_error() {
	ERROR_STRUCT prev_error;
	uint16_t category;
	uint16_t detail;

	if ((EE_ReadVariable(VirtAddVarTab[PREV_ERROR_CATEGORY_INDEX], &category)) != HAL_OK) {
		Error_Handler();
	}
	if ((EE_ReadVariable(VirtAddVarTab[PREV_ERROR_DETAIL_INDEX], &detail)) != HAL_OK) {
		Error_Handler();
	}
	prev_error.category = category;
	prev_error.detail = detail;

	return prev_error;
}

void send_error_counter_packet() {
	uint8_t buffer[ERROR_COUNTER_PACKET_SIZE];

	buffer[0] = ERROR_COUNTER_PACKET_SYNC;
	buffer[1] = ERROR_COUNTER_PACKET_SYNC;
	buffer[2] = ((local_cpy[0] & 0xFF00) >> 8);
	buffer[3] = (local_cpy[0] & 0xFF);
	buffer[4] = ((local_cpy[1] & 0xFF00) >> 8);
	buffer[5] = (local_cpy[1] & 0xFF);
	buffer[6] = ((local_cpy[2] & 0xFF00) >> 8);
	buffer[7] = (local_cpy[2] & 0xFF);
	buffer[8] = ((local_cpy[3] & 0xFF00) >> 8);
	buffer[9] = (local_cpy[3] & 0xFF);
	buffer[10] = ((local_cpy[4] & 0xFF00) >> 8);
	buffer[11] = (local_cpy[4] & 0xFF);
	buffer[12] = ((local_cpy[5] & 0xFF00) >> 8);
	buffer[13] = (local_cpy[5] & 0xFF);
	buffer[14] = ((local_cpy[6] & 0xFF00) >> 8);
	buffer[15] = (local_cpy[6] & 0xFF);
	buffer[16] = ((local_cpy[7] & 0xFF00) >> 8);
	buffer[17] = (local_cpy[7] & 0xFF);
	buffer[18] = ((local_cpy[8] & 0xFF00) >> 8);
	buffer[19] = (local_cpy[8] & 0xFF);
	buffer[20] = ((local_cpy[9] & 0xFF00) >> 8);
	buffer[21] = (local_cpy[9] & 0xFF);
	buffer[22] = ((local_cpy[10] & 0xFF00) >> 8);
	buffer[23] = (local_cpy[10] & 0xFF);
	buffer[24] = ((local_cpy[11] & 0xFF00) >> 8);
	buffer[25] = (local_cpy[11] & 0xFF);
	buffer[26] = ((local_cpy[12] & 0xFF00) >> 8);
	buffer[27] = (local_cpy[12] & 0xFF);
	buffer[28] = ((local_cpy[13] & 0xFF00) >> 8);
	buffer[29] = (local_cpy[13] & 0xFF);
	buffer[30] = ((local_cpy[14] & 0xFF00) >> 8);
	buffer[31] = (local_cpy[14] & 0xFF);
	buffer[32] = ((local_cpy[15] & 0xFF00) >> 8);
	buffer[33] = (local_cpy[15] & 0xFF);
	buffer[34] = ((local_cpy[16] & 0xFF00) >> 8);
	buffer[35] = (local_cpy[16] & 0xFF);
	buffer[36] = ((local_cpy[17] & 0xFF00) >> 8);
	buffer[37] = (local_cpy[17] & 0xFF);
	buffer[38] = ((local_cpy[18] & 0xFF00) >> 8);
	buffer[39] = (local_cpy[18] & 0xFF);
	buffer[40] = ((local_cpy[19] & 0xFF00) >> 8);
	buffer[41] = (local_cpy[19] & 0xFF);
	buffer[42] = ((local_cpy[20] & 0xFF00) >> 8);
	buffer[43] = (local_cpy[20] & 0xFF);
	buffer[44] = ((local_cpy[21] & 0xFF00) >> 8);
	buffer[45] = (local_cpy[21] & 0xFF);
	buffer[46] = ((local_cpy[22] & 0xFF00) >> 8);
	buffer[47] = (local_cpy[22] & 0xFF);
	buffer[48] = ((local_cpy[23] & 0xFF00) >> 8);
	buffer[49] = (local_cpy[23] & 0xFF);
	buffer[50] = ((local_cpy[24] & 0xFF00) >> 8);
	buffer[51] = (local_cpy[24] & 0xFF);
	buffer[52] = ((local_cpy[25] & 0xFF00) >> 8);
	buffer[53] = (local_cpy[25] & 0xFF);
	buffer[54] = ((local_cpy[26] & 0xFF00) >> 8);
	buffer[55] = (local_cpy[26] & 0xFF);


	HAL_UART_Transmit(&huart1, buffer, ERROR_COUNTER_PACKET_SIZE, 100);
}

void send_previous_error_packet() {
	ERROR_STRUCT prev_error;
	uint8_t buffer[PREV_ERROR_PACKET_SIZE];

	prev_error = get_previous_error();

	buffer[0] = PREV_ERROR_PACKET_SYNC;
	buffer[1] = PREV_ERROR_PACKET_SYNC;
	buffer[2] = prev_error.category;
	buffer[3] = prev_error.detail;

	HAL_UART_Transmit(&huart1, buffer, PREV_ERROR_PACKET_SIZE, 100);

}

void send_junk_packet() {
	uint8_t buffer[JUNK_PACKET_SIZE];

	for (int i = 0; i < JUNK_PACKET_SIZE; i++) {
		buffer[i] = 0xEE;
	}
	HAL_UART_Transmit(&huart1, buffer, JUNK_PACKET_SIZE, 100);
}

