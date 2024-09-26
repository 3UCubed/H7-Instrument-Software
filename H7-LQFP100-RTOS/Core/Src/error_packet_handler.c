/**
 * @file error_packet_handler.c
 * @brief Implementation of error packet handling
 *
 * Handles all errors in the system. If the built in HAL error handler is called by a peripheral, it redirects here.
 *
 * @author Jared Morrison
 * @date September 4, 2024
 */

#include "error_packet_handler.h"
#include "usart.h"
#include "main.h"
#include "eeprom.h"
#include "tim.h"
#include "dac.h"
void emergency_shutdown();

/**
 * @brief Array storing virtual addresses for EEPROM emulation variables.
 *
 * This array holds the virtual addresses of the variables stored in the emulated EEPROM.
 * The addresses are defined to start from `0x5550` and continue sequentially.
 * The size of the array is determined by the `NB_OF_VAR` constant.
 */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5550, 0x5551, 0x5552, 0x5553, 0x5554, 0x5555, 0x5556, 0x5557, 0x5558, 0x5559, 0x555A, 0x555B, 0x555C, 0x555D, 0x555E, 0x555F, 0x6660, 0x6661, 0x6662, 0x6663, 0x6664, 0x6665, 0x6666, 0x6667, 0x6668, 0x6669, 0x666A, 0x666B, 0x666C, 0x666D, 0x666E};

/**
 * @brief Array used to store the error counters.
 *
 * The reason this array is of size NUM_ERROR_COUNTERS instead of NB_OF_VAR is because
 * there are two extra variables stored in the EE used for keeping track of the last
 * error that occurred. There are a total of 29 variables that we keep track of in the
 * EE. The first 27 are the error counters, and the last two are the error codes for the
 * last error that occurred.
 */
uint16_t local_cpy[NUM_ERROR_COUNTERS];

/**
 * @brief Handles all errors in the system.
 *
 * Built in error handler calls this function instead of entering an infinite while loop.
 * Before calling this function, the caller will create a new variable of type ERROR_STRUCT
 * and populate the error_category and error_detail attributes with the respective codes.
 * Given and error category and detail, this error handler proceeds with the appropriate actions.
 * Regardless of what caused the error, this error handler will always increment the error counter,
 * set the previous error to whatever error we are currently handling, send an error packet,
 * and enter IDLE mode. Additional actions are taken depending on the error category.
 *
 * @param error Error given by the caller.
 */
void handle_error(ERROR_STRUCT error) {
#ifdef ERROR_HANDLING_ENABLED
	vTaskSuspendAll();
	emergency_shutdown();
	while (!IDLING) {};
	increment_error_counter(error);
	set_previous_error(error);
	send_current_error_packet(error);
	send_junk_packet();

	switch (error.category) {
	case EC_power_supply_rail:
		NVIC_SystemReset();
		break;
	case EC_seu:
		NVIC_SystemReset();
		break;
	case EC_peripheral:
		NVIC_SystemReset();
		break;
	default:
		// Should not be possible to get here
		break;
	}
#endif
}

/**
 * @breif Initializes the EE, reads the error counters from the EE, and stores them in local_cpy.
 */
void error_counter_init() {
	HAL_FLASH_Unlock();
	if (EE_Init() != EE_OK) {
		Error_Handler();
	}

	for (int i = 0; i < NUM_ERROR_COUNTERS; i++) {
		if ((EE_ReadVariable(VirtAddVarTab[i], &local_cpy[i])) != HAL_OK) {
			Error_Handler();
		}
	}
}

/**
 * @brief Increments the error counter for a given error, and updates the variable in the EE.
 *
 * I designed the error categories and codes such that they correspond to an index in our
 * local_cpy array. To see what index a particular error is stored in, just check the value
 * each category or detail is assigned in the header file.
 *
 * @param error Error given by the caller.
 */
void increment_error_counter(ERROR_STRUCT error) {
	local_cpy[error.category]++;
	local_cpy[error.detail]++;
	update_error_counter();
}

/**
 * @brief Writes the contents of local_cpy to the EE, excluding the previous error codes.
 */
void update_error_counter(){
	for (int i = 0; i < NUM_ERROR_COUNTERS; i++) {
		if ((EE_WriteVariable(VirtAddVarTab[i], local_cpy[i])) != HAL_OK) {
			Error_Handler();
		}
	}
}

/**
 * @brief Resets all error counters in the EE to 0.
 */
void reset_error_counters() {
	for (int i = 0; i < NUM_ERROR_COUNTERS; i++) {
		if ((EE_WriteVariable(VirtAddVarTab[i], 0)) != HAL_OK) {
			Error_Handler();
		}
	}
}

/**
 * @brief Resets the previous error codes to 0xFF. 0xFF was chose because it doesn't correspond to any error code.
 */
void reset_previous_error() {
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_CATEGORY_INDEX], 0xFF)) != HAL_OK) {
		Error_Handler();
	}
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_DETAIL_INDEX], 0xFF)) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Sets previous error code in the EE.
 *
 * @param error Previous error code in EE is set to this.
 */
void set_previous_error(ERROR_STRUCT error) {
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_CATEGORY_INDEX], error.category)) != HAL_OK) {
		Error_Handler();
	}
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_DETAIL_INDEX], error.detail)) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Reads the previous error codes from EE
 *
 * @return Error populated with retrieved category and detail.
 */
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

/**
 * @brief Creates and sends a packet containing all 27 error counters.
 *
 * There is an error counter for every single category and detail.
 * This type of packet is only sent during sync.
 */
void send_error_counter_packet() {
	uint8_t buffer[ERROR_COUNTER_PACKET_SIZE];

	buffer[0] = ERROR_COUNTER_PACKET_SYNC;
	buffer[1] = ERROR_COUNTER_PACKET_SYNC;
	buffer[2] = ((local_cpy[EC_power_supply_rail] & 0xFF00) >> 8);
	buffer[3] = (local_cpy[EC_power_supply_rail] & 0xFF);
	buffer[4] = ((local_cpy[EC_seu] & 0xFF00) >> 8);
	buffer[5] = (local_cpy[EC_seu] & 0xFF);
	buffer[6] = ((local_cpy[EC_peripheral] & 0xFF00) >> 8);
	buffer[7] = (local_cpy[EC_peripheral] & 0xFF);
	buffer[8] = ((local_cpy[EC_brownout] & 0xFF00) >> 8);
	buffer[9] = (local_cpy[EC_brownout] & 0xFF);
	buffer[10] = ((local_cpy[EC_watchdog] & 0xFF00) >> 8);
	buffer[11] = (local_cpy[EC_watchdog] & 0xFF);
	buffer[12] = ((local_cpy[EC_UNDEFINED] & 0xFF00) >> 8);
	buffer[13] = (local_cpy[EC_UNDEFINED] & 0xFF);
	buffer[14] = ((local_cpy[ED_vsense] & 0xFF00) >> 8);
	buffer[15] = (local_cpy[ED_vsense] & 0xFF);
	buffer[16] = ((local_cpy[ED_vrefint] & 0xFF00) >> 8);
	buffer[17] = (local_cpy[ED_vrefint] & 0xFF);
	buffer[18] = ((local_cpy[ED_TEMP1] & 0xFF00) >> 8);
	buffer[19] = (local_cpy[ED_TEMP1] & 0xFF);
	buffer[20] = ((local_cpy[ED_TEMP2] & 0xFF00) >> 8);
	buffer[21] = (local_cpy[ED_TEMP2] & 0xFF);
	buffer[22] = ((local_cpy[ED_TEMP3] & 0xFF00) >> 8);
	buffer[23] = (local_cpy[ED_TEMP3] & 0xFF);
	buffer[24] = ((local_cpy[ED_TEMP4] & 0xFF00) >> 8);
	buffer[25] = (local_cpy[ED_TEMP4] & 0xFF);
	buffer[26] = ((local_cpy[ED_busvmon] & 0xFF00) >> 8);
	buffer[27] = (local_cpy[ED_busvmon] & 0xFF);
	buffer[28] = ((local_cpy[ED_busimon] & 0xFF00) >> 8);
	buffer[29] = (local_cpy[ED_busimon] & 0xFF);
	buffer[30] = ((local_cpy[ED_2v5] & 0xFF00) >> 8);
	buffer[31] = (local_cpy[ED_2v5] & 0xFF);
	buffer[32] = ((local_cpy[ED_3v3] & 0xFF00) >> 8);
	buffer[33] = (local_cpy[ED_3v3] & 0xFF);
	buffer[34] = ((local_cpy[ED_5v] & 0xFF00) >> 8);
	buffer[35] = (local_cpy[ED_5v] & 0xFF);
	buffer[36] = ((local_cpy[ED_n3v3] & 0xFF00) >> 8);
	buffer[37] = (local_cpy[ED_n3v3] & 0xFF);
	buffer[38] = ((local_cpy[ED_n5v] & 0xFF00) >> 8);
	buffer[39] = (local_cpy[ED_n5v] & 0xFF);
	buffer[40] = ((local_cpy[ED_15v] & 0xFF00) >> 8);
	buffer[41] = (local_cpy[ED_15v] & 0xFF);
	buffer[42] = ((local_cpy[ED_5vref] & 0xFF00) >> 8);
	buffer[43] = (local_cpy[ED_5vref] & 0xFF);
	buffer[44] = ((local_cpy[ED_n200v] & 0xFF00) >> 8);
	buffer[45] = (local_cpy[ED_n200v] & 0xFF);
	buffer[46] = ((local_cpy[ED_n800v] & 0xFF00) >> 8);
	buffer[47] = (local_cpy[ED_n800v] & 0xFF);
	buffer[48] = ((local_cpy[ED_TMP1] & 0xFF00) >> 8);
	buffer[49] = (local_cpy[ED_TMP1] & 0xFF);
	buffer[50] = ((local_cpy[ED_single_bit_error_flash] & 0xFF00) >> 8);
	buffer[51] = (local_cpy[ED_single_bit_error_flash] & 0xFF);
	buffer[52] = ((local_cpy[ED_double_bit_error_flash] & 0xFF00) >> 8);
	buffer[53] = (local_cpy[ED_double_bit_error_flash] & 0xFF);
	buffer[54] = ((local_cpy[ED_single_bit_error_ram] & 0xFF00) >> 8);
	buffer[55] = (local_cpy[ED_single_bit_error_ram] & 0xFF);
	buffer[56] = ((local_cpy[ED_double_bit_error_ram] & 0xFF00) >> 8);
	buffer[57] = (local_cpy[ED_double_bit_error_ram] & 0xFF);
	buffer[58] = ((local_cpy[ED_UNDEFINED] & 0xFF00) >> 8);
	buffer[59] = (local_cpy[ED_UNDEFINED] & 0xFF);

	HAL_UART_Transmit(&huart1, buffer, ERROR_COUNTER_PACKET_SIZE, 100);
}

/**
 * @brief Creates and sends a packet containing the error codes for the previous error.
 *
 * This type of packet is only sent on request.
 */
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

/**
 * @brief Creates and sends a packet containing the error codes for the current error.
 *
 * This type of packet is only sent when handle_error() is called.
 */
void send_current_error_packet(ERROR_STRUCT error) {
	uint8_t buffer[CURRENT_ERROR_PACKET_SIZE];

	// If the error isn't a power supply rail, set the out of bounds values to 0
	if (error.category != EC_power_supply_rail) {
		error.OOB_1 = 0;
		error.OOB_2 = 0;
		error.OOB_3 = 0;
	}

	buffer[0] = CURRENT_ERROR_PACKET_SYNC;
	buffer[1] = CURRENT_ERROR_PACKET_SYNC;
	buffer[2] = error.category;
	buffer[3] = error.detail;
	buffer[4] = ((error.OOB_1 & 0xFF00) >> 8);
	buffer[5] = (error.OOB_1 & 0xFF);
	buffer[6] = ((error.OOB_2 & 0xFF00) >> 8);
	buffer[7] = (error.OOB_2 & 0xFF);
	buffer[8] = ((error.OOB_3 & 0xFF00) >> 8);
	buffer[9] = (error.OOB_3 & 0xFF);

	HAL_UART_Transmit(&huart1, buffer, CURRENT_ERROR_PACKET_SIZE, 100);
}

/**
 * @brief Creates and sends a junk packet containing all 0xCE.
 *
 * Used to clear out the buffer on the OBC.
 */
void send_junk_packet() {	// TODO: Figure out if we still need this.
	uint8_t buffer[JUNK_PACKET_SIZE];

	for (int i = 0; i < JUNK_PACKET_SIZE; i++) {
		buffer[i] = 0xCE;
	}

	HAL_UART_Transmit(&huart1, buffer, JUNK_PACKET_SIZE, 100);
}


void emergency_shutdown() {
	ERPA_ENABLED = 0;
	TIM2->CCR4 = 0;
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);			// PMT packet off
	HK_ENABLED = 0;
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);			// Disable auto sweep


	// Telling rail monitor which voltages are now disabled
	for (int i = RAIL_TMP1; i >= RAIL_busvmon; i--) {
		set_rail_monitor_enable(i, 0);
	}

	// Disabling all voltages
	for (int i = 8; i >= 0; i--) {
		HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
		HAL_Delay(200);
	}
	HAL_Delay(3000);
	IDLING = 1;
}







