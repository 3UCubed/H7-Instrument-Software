/**
 ******************************************************************************
 * @file           : error_packet_handler.c
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Handles all errors in the system. If the built in HAL
 * 					 error handler is called by a peripheral, it redirects here.
 ******************************************************************************
 */

#include "error_packet_handler.h"
#include "usart.h"
#include "main.h"
#include "eeprom.h"
#include "tim.h"
#include "dac.h"

#define ERROR_COUNTER_PACKET_SIZE 58
#define PREV_ERROR_PACKET_SIZE 4
#define CURRENT_ERROR_PACKET_SIZE 10
#define JUNK_PACKET_SIZE 1024

#define ERROR_COUNTER_PACKET_SYNC 0xCC
#define PREV_ERROR_PACKET_SYNC 0xAA
#define CURRENT_ERROR_PACKET_SYNC 0xBB

#define PREV_ERROR_CATEGORY_INDEX 28
#define PREV_ERROR_DETAIL_INDEX 29

void emergency_shutdown();
void flash_mass_erase();
void update_error_counter();
void reset_previous_error();
ERROR_STRUCT get_previous_error();
void send_current_error_packet(ERROR_STRUCT error);
void send_junk_packet();

/**
 * @brief Array storing virtual addresses for EEPROM emulation variables.
 */
uint16_t VirtAddVarTab[NB_OF_VAR] =
{
		0x0001, 0x0002, 0x0003, 0x0004, 0x0005,
		0x0006, 0x0007, 0x0008, 0x0009, 0x0010,
		0x0011, 0x0012, 0x0013, 0x0014, 0x0015,
		0x0016, 0x0017, 0x0018, 0x0019, 0x0020,
		0x0021, 0x0022, 0x0023, 0x0024, 0x0025,
		0x0026, 0x0027, 0x0028, 0x0029, 0x0030
};

/**
 * @brief Array used to store the error counters locally after fetching from flash.
 */
uint16_t local_cpy[NUM_ERROR_COUNTERS];

/**
 * @brief Handles system errors based on the provided error structure.
 *        Initiates an emergency shutdown, manages Flash ECC-related errors, and sends error packets.
 *
 * @param error The error structure containing the error category and details.
 */
void handle_error(ERROR_STRUCT error)
{
#ifdef ERROR_HANDLING_ENABLED
	emergency_shutdown();

	// If error was caused by flash ECC...
	if ((error.detail == ED_single_bit_error_flash) || (error.detail == ED_double_bit_error_flash))
	{
		// Erase user flash, reinit EE, reset error counters, increment error counter, set previous error
		local_cpy[error.category]++;
		local_cpy[error.detail]++;
		flash_mass_erase();
		EE_Init();
		reset_error_counters();
		update_error_counter();
		set_previous_error(error);
	}
	// Otherwise, just increment error counter and set previous error
	else
	{
		increment_error_counter(error);
		set_previous_error(error);
	}

	// Wait until all power supply rails are off, then send current error packet + junk data
	while(!IDLING){};
	send_current_error_packet(error);
	send_junk_packet();

	HAL_TIM_Base_Start_IT(&htim3);
#endif
}

/**
 * @brief Performs a mass erase of Flash memory.
 *        Unlocks Flash and erases all sectors in Bank 2, handling errors if the erase fails.
 */
void flash_mass_erase()
{
	HAL_FLASH_Unlock();

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError = 0;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Banks = FLASH_BANK_2;
	EraseInitStruct.Sector = FLASH_SECTOR_0;
	EraseInitStruct.NbSectors = FLASH_SECTOR_TOTAL;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @breif Initializes the EE, reads the error counters from the EE, and stores them in local_cpy.
 */
void error_counter_init()
{
	HAL_FLASH_Unlock();
	if (EE_Init() != EE_OK)
	{
		Error_Handler();
	}

	for (int i = 0; i < NUM_ERROR_COUNTERS; i++)
	{
		if ((EE_ReadVariable(VirtAddVarTab[i], &local_cpy[i])) != HAL_OK)
		{
			Error_Handler();
		}
	}
}

/**
 * @brief Increments the error counters for the specified error.
 *        Updates both the category and detail counters and saves the updated values.
 *
 * @param error The error structure containing the error category and details.
 */
void increment_error_counter(ERROR_STRUCT error)
{
	// If either counter is about to overflow, go past 65535, reset everything
	if (local_cpy[error.category] >= 0xFFFE || local_cpy[error.detail] >= 0xFFFE)
	{
		reset_error_counters();  // Set all to 0
		update_error_counter();
	}
	local_cpy[error.category]++;
	local_cpy[error.detail]++;
	update_error_counter();
}

/**
 * @brief Writes the contents of local_cpy to the EE, excluding the previous error codes.
 */
void update_error_counter()
{
	for (int i = 0; i < NUM_ERROR_COUNTERS; i++)
	{
		if ((EE_WriteVariable(VirtAddVarTab[i], local_cpy[i])) != HAL_OK)
		{
			Error_Handler();
		}
	}
}

/**
 * @brief Resets all error counters in the EE to 0.
 */
void reset_error_counters()
{
	for (int i = 0; i < NUM_ERROR_COUNTERS; i++)
	{
		local_cpy[i] = 0;  // Reset RAM copy too
		if ((EE_WriteVariable(VirtAddVarTab[i], 0)) != HAL_OK)
		{
			Error_Handler();
		}
	}
}

/**
 * @brief Resets the previous error codes to 0xFF. 0xFF was chose because it doesn't correspond to any error code.
 */
void reset_previous_error()
{
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_CATEGORY_INDEX], EC_UNDEFINED)) != HAL_OK)
	{
		Error_Handler();
	}
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_DETAIL_INDEX], ED_UNDEFINED)) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Sets previous error code in the EE.
 *
 * @param error Previous error code in EE is set to this.
 */
void set_previous_error(ERROR_STRUCT error)
{
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_CATEGORY_INDEX], error.category)) != HAL_OK)
	{
		Error_Handler();
	}
	if ((EE_WriteVariable(VirtAddVarTab[PREV_ERROR_DETAIL_INDEX], error.detail)) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Manually sets a specific error counter to a given value and updates EEPROM.
 *
 * @param category_or_detail Index in local_cpy[] (should match an EC_* or ED_* enum)
 * @param value The value to assign (must be <= 0xFFFF)
 */
void set_error_counter(uint8_t category_or_detail, uint16_t value)
{
	if (category_or_detail >= NUM_ERROR_COUNTERS)
		return; // Out of bounds, ignore

	local_cpy[category_or_detail] = value;

	if (EE_WriteVariable(VirtAddVarTab[category_or_detail], value) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Reads the previous error codes from EE
 *
 * @return Error populated with retrieved category and detail.
 */
ERROR_STRUCT get_previous_error()
{
	ERROR_STRUCT prev_error;
	uint16_t category;
	uint16_t detail;

	if ((EE_ReadVariable(VirtAddVarTab[PREV_ERROR_CATEGORY_INDEX], &category)) != HAL_OK)
	{
		Error_Handler();
	}
	if ((EE_ReadVariable(VirtAddVarTab[PREV_ERROR_DETAIL_INDEX], &detail)) != HAL_OK)
	{
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
void send_error_counter_packet()
{
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
	buffer[32] = ((local_cpy[ED_5v] & 0xFF00) >> 8);
	buffer[33] = (local_cpy[ED_5v] & 0xFF);
	buffer[34] = ((local_cpy[ED_n3v3] & 0xFF00) >> 8);
	buffer[35] = (local_cpy[ED_n3v3] & 0xFF);
	buffer[36] = ((local_cpy[ED_n5v] & 0xFF00) >> 8);
	buffer[37] = (local_cpy[ED_n5v] & 0xFF);
	buffer[38] = ((local_cpy[ED_15v] & 0xFF00) >> 8);
	buffer[39] = (local_cpy[ED_15v] & 0xFF);
	buffer[40] = ((local_cpy[ED_5vref] & 0xFF00) >> 8);
	buffer[41] = (local_cpy[ED_5vref] & 0xFF);
	buffer[42] = ((local_cpy[ED_n200v] & 0xFF00) >> 8);
	buffer[43] = (local_cpy[ED_n200v] & 0xFF);
	buffer[44] = ((local_cpy[ED_n800v] & 0xFF00) >> 8);
	buffer[45] = (local_cpy[ED_n800v] & 0xFF);
	buffer[46] = ((local_cpy[ED_TMP1] & 0xFF00) >> 8);
	buffer[47] = (local_cpy[ED_TMP1] & 0xFF);
	buffer[48] = ((local_cpy[ED_single_bit_error_flash] & 0xFF00) >> 8);
	buffer[49] = (local_cpy[ED_single_bit_error_flash] & 0xFF);
	buffer[50] = ((local_cpy[ED_double_bit_error_flash] & 0xFF00) >> 8);
	buffer[51] = (local_cpy[ED_double_bit_error_flash] & 0xFF);
	buffer[52] = ((local_cpy[ED_single_bit_error_ram] & 0xFF00) >> 8);
	buffer[53] = (local_cpy[ED_single_bit_error_ram] & 0xFF);
	buffer[54] = ((local_cpy[ED_double_bit_error_ram] & 0xFF00) >> 8);
	buffer[55] = (local_cpy[ED_double_bit_error_ram] & 0xFF);
	buffer[56] = ((local_cpy[ED_UNDEFINED] & 0xFF00) >> 8);
	buffer[57] = (local_cpy[ED_UNDEFINED] & 0xFF);

	HAL_UART_Transmit(&huart1, buffer, ERROR_COUNTER_PACKET_SIZE, UART_TIMEOUT_MS);
}

/**
 * @brief Creates and sends a packet containing the error codes for the previous error.
 *
 * This type of packet is only sent on request.
 */
void send_previous_error_packet()
{
	ERROR_STRUCT prev_error;
	uint8_t buffer[PREV_ERROR_PACKET_SIZE];

	prev_error = get_previous_error();

	buffer[0] = PREV_ERROR_PACKET_SYNC;
	buffer[1] = PREV_ERROR_PACKET_SYNC;
	buffer[2] = prev_error.category;
	buffer[3] = prev_error.detail;

	HAL_UART_Transmit(&huart1, buffer, PREV_ERROR_PACKET_SIZE, UART_TIMEOUT_MS);
}

/**
 * @brief Creates and sends a packet containing the error codes for the current error.
 *
 * This type of packet is only sent when handle_error() is called.
 */
void send_current_error_packet(ERROR_STRUCT error)
{
	uint8_t buffer[CURRENT_ERROR_PACKET_SIZE];

	// If the error isn't a power supply rail, set the out of bounds values to 0
	if (error.category != EC_power_supply_rail)
	{
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

	HAL_UART_Transmit(&huart1, buffer, CURRENT_ERROR_PACKET_SIZE, UART_TIMEOUT_MS);
}

/**
 * @brief Creates and sends a junk packet containing all 0xCE.
 *
 * Used to clear out the buffer on the OBC.
 */
void send_junk_packet()
{
	uint8_t buffer[JUNK_PACKET_SIZE];

	for (int i = 0; i < JUNK_PACKET_SIZE; i++) {
		buffer[i] = 0xCE;
	}

	HAL_UART_Transmit(&huart1, buffer, JUNK_PACKET_SIZE, UART_TIMEOUT_MS);
}

/**
 * @brief Initiates an emergency shutdown of the system.
 *        Disables timers, DAC, rail monitoring, and all power supply voltages, setting the system to idle.
 */
void emergency_shutdown()
{
	ERPA_ENABLED = 0;
	TIM2->CCR4 = 0;
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);

	HK_ENABLED = 0;
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	// Telling rail monitor which voltages are now disabled
	for (int i = RAIL_TMP1; i >= RAIL_busvmon; i--)
	{
		set_rail_monitor_enable(i, 0);
	}

	// Disabling all voltages
	for (int i = GPIOS_INDEX_N800V; i >= GPIOS_INDEX_SDN1; i--)
	{
		HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
	}
	IDLING = 1;
}

//void simulate_error_overflow()
//{
//    // Manually set a category and detail to 0xFFFF - 1 to simulate imminent overflow
//    set_error_counter(EC_power_supply_rail, 0xFFFE);
//    set_error_counter(ED_TEMP1, 0xFFFE);
//
//    // Now create an error that triggers the overflow
//    ERROR_STRUCT simulated_error = {
//        .category = EC_power_supply_rail,
//        .detail = ED_TEMP1,
//        .OOB_1 = 0,
//        .OOB_2 = 0,
//        .OOB_3 = 0
//    };
//
//    handle_error(simulated_error); // This will increment, detect overflow, reset, and start again
//}
