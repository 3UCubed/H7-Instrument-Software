/**
 ******************************************************************************
 * @file           : packet_creation.c
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Implementation file for creation of all data packets.
 ******************************************************************************
 */

#include "packet_creation.h"

#define SYNC_DATA_SIZE 63
#define VERSION_DATA_SIZE 5
#define PMT_DATA_SIZE 10
#define ERPA_DATA_SIZE 14
#define HK_DATA_SIZE 48
#define UPTIME_SIZE 4
#define TIMESTAMP_SIZE 6

#define SYNC_SYNCWORD 0x88
#define VERSION_SYNCWORD 0x99
#define PMT_SYNCWORD 0xFF
#define ERPA_SYNCWORD 0xEE
#define HK_SYNCWORD 0xDD

uint16_t pmt_seq = 0;
uint32_t erpa_seq = 0;
uint16_t hk_seq = 0;

/**
 * @brief Creates and transmits a sync packet.
 *
 * This function combines version number, error
 * counters, and reset cause into a single packet.
 */
void create_sync_packet(ERROR_STRUCT reset_cause)
{
	static uint8_t buffer[SYNC_DATA_SIZE];

	buffer[0] = SYNC_SYNCWORD;
	buffer[1] = SYNC_SYNCWORD;
	buffer[2] = V_MAJOR;
	buffer[3] = V_MINOR;
	buffer[4] = V_PATCH;
	buffer[5] = ((local_cpy[EC_power_supply_rail] & 0xFF00) >> 8);
	buffer[6] = (local_cpy[EC_power_supply_rail] & 0xFF);
	buffer[7] = ((local_cpy[EC_seu] & 0xFF00) >> 8);
	buffer[8] = (local_cpy[EC_seu] & 0xFF);
	buffer[9] = ((local_cpy[EC_peripheral] & 0xFF00) >> 8);
	buffer[10] = (local_cpy[EC_peripheral] & 0xFF);
	buffer[11] = ((local_cpy[EC_brownout] & 0xFF00) >> 8);
	buffer[12] = (local_cpy[EC_brownout] & 0xFF);
	buffer[13] = ((local_cpy[EC_watchdog] & 0xFF00) >> 8);
	buffer[14] = (local_cpy[EC_watchdog] & 0xFF);
	buffer[15] = ((local_cpy[EC_UNDEFINED] & 0xFF00) >> 8);
	buffer[16] = (local_cpy[EC_UNDEFINED] & 0xFF);
	buffer[17] = ((local_cpy[ED_vsense] & 0xFF00) >> 8);
	buffer[18] = (local_cpy[ED_vsense] & 0xFF);
	buffer[19] = ((local_cpy[ED_vrefint] & 0xFF00) >> 8);
	buffer[20] = (local_cpy[ED_vrefint] & 0xFF);
	buffer[21] = ((local_cpy[ED_TEMP1] & 0xFF00) >> 8);
	buffer[22] = (local_cpy[ED_TEMP1] & 0xFF);
	buffer[23] = ((local_cpy[ED_TEMP2] & 0xFF00) >> 8);
	buffer[24] = (local_cpy[ED_TEMP2] & 0xFF);
	buffer[25] = ((local_cpy[ED_TEMP3] & 0xFF00) >> 8);
	buffer[26] = (local_cpy[ED_TEMP3] & 0xFF);
	buffer[27] = ((local_cpy[ED_TEMP4] & 0xFF00) >> 8);
	buffer[28] = (local_cpy[ED_TEMP4] & 0xFF);
	buffer[29] = ((local_cpy[ED_busvmon] & 0xFF00) >> 8);
	buffer[30] = (local_cpy[ED_busvmon] & 0xFF);
	buffer[31] = ((local_cpy[ED_busimon] & 0xFF00) >> 8);
	buffer[32] = (local_cpy[ED_busimon] & 0xFF);
	buffer[33] = ((local_cpy[ED_2v5] & 0xFF00) >> 8);
	buffer[34] = (local_cpy[ED_2v5] & 0xFF);
	buffer[35] = ((local_cpy[ED_5v] & 0xFF00) >> 8);
	buffer[36] = (local_cpy[ED_5v] & 0xFF);
	buffer[37] = ((local_cpy[ED_n3v3] & 0xFF00) >> 8);
	buffer[38] = (local_cpy[ED_n3v3] & 0xFF);
	buffer[39] = ((local_cpy[ED_n5v] & 0xFF00) >> 8);
	buffer[40] = (local_cpy[ED_n5v] & 0xFF);
	buffer[41] = ((local_cpy[ED_15v] & 0xFF00) >> 8);
	buffer[42] = (local_cpy[ED_15v] & 0xFF);
	buffer[43] = ((local_cpy[ED_5vref] & 0xFF00) >> 8);
	buffer[44] = (local_cpy[ED_5vref] & 0xFF);
	buffer[45] = ((local_cpy[ED_n200v] & 0xFF00) >> 8);
	buffer[46] = (local_cpy[ED_n200v] & 0xFF);
	buffer[47] = ((local_cpy[ED_n800v] & 0xFF00) >> 8);
	buffer[48] = (local_cpy[ED_n800v] & 0xFF);
	buffer[49] = ((local_cpy[ED_TMP1] & 0xFF00) >> 8);
	buffer[50] = (local_cpy[ED_TMP1] & 0xFF);
	buffer[51] = ((local_cpy[ED_single_bit_error_flash] & 0xFF00) >> 8);
	buffer[52] = (local_cpy[ED_single_bit_error_flash] & 0xFF);
	buffer[53] = ((local_cpy[ED_double_bit_error_flash] & 0xFF00) >> 8);
	buffer[54] = (local_cpy[ED_double_bit_error_flash] & 0xFF);
	buffer[55] = ((local_cpy[ED_single_bit_error_ram] & 0xFF00) >> 8);
	buffer[56] = (local_cpy[ED_single_bit_error_ram] & 0xFF);
	buffer[57] = ((local_cpy[ED_double_bit_error_ram] & 0xFF00) >> 8);
	buffer[58] = (local_cpy[ED_double_bit_error_ram] & 0xFF);
	buffer[59] = ((local_cpy[ED_UNDEFINED] & 0xFF00) >> 8);
	buffer[60] = (local_cpy[ED_UNDEFINED] & 0xFF);
	buffer[61] = reset_cause.category;
	buffer[62] = reset_cause.detail;

	HAL_UART_Transmit(&huart1, buffer, SYNC_DATA_SIZE, UART_TIMEOUT_MS);
}

/**
 * @brief Creates and transmits a version packet.
 *
 * This function sends a packet containing the major,
 * minor, and patch versions of the firmware and transmits
 * it over UART.
 *
 * @note V_MAJOR, V_MINOR, and V_PATCH are defined in main.h.
 */
void create_version_packet()
{
	static uint8_t buffer[VERSION_DATA_SIZE];

	buffer[0] = VERSION_SYNCWORD;
	buffer[1] = VERSION_SYNCWORD;
	buffer[2] = V_MAJOR;
	buffer[3] = V_MINOR;
	buffer[4] = V_PATCH;

	HAL_UART_Transmit(&huart1, buffer, VERSION_DATA_SIZE, UART_TIMEOUT_MS);
}

/**
 * @brief Creates and transmits a PMT packet.
 *
 * This function samples the PMT SPI data, retrieves uptime information,
 * constructs a packet with synchronization bytes and data,
 * and transmits it over UART.
 */
void create_pmt_packet()
{
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {};

	uint8_t buffer[PMT_DATA_SIZE];
	uint8_t pmt_spi[2];
	uint8_t uptime[UPTIME_SIZE];

	get_uptime(uptime);
	sample_pmt_spi(pmt_spi);

	buffer[0] = PMT_SYNCWORD;
	buffer[1] = PMT_SYNCWORD;
	buffer[2] = uptime[0];
	buffer[3] = uptime[1];
	buffer[4] = uptime[2];
	buffer[5] = uptime[3];
	buffer[6] = ((pmt_seq & 0xFF00) >> 8);
	buffer[7] = (pmt_seq & 0xFF);
	buffer[8] = pmt_spi[0];
	buffer[9] = pmt_spi[1];

	HAL_UART_Transmit(&huart1, buffer, PMT_DATA_SIZE, UART_TIMEOUT_MS);

	pmt_seq++;
}

/**
 * @brief Creates and transmits an ERPA packet.
 *
 * This function samples the ERPA SPI and ADC data, retrieves uptime information,
 * constructs a packet with synchronization bytes, sequence number, and data,
 * and transmits it over UART.
 */
void create_erpa_packet()
{
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) {};

	uint8_t buffer[ERPA_DATA_SIZE];
	uint8_t erpa_spi[2];
	uint16_t erpa_adc[1];
	uint8_t uptime[UPTIME_SIZE];
	STEP_VALUES sweep_step = INVALID_STEP;

	get_uptime(uptime);
	sweep_step = get_current_step();

	sample_erpa_spi(erpa_spi);
	sample_erpa_adc(erpa_adc);

	buffer[0] = ERPA_SYNCWORD;
	buffer[1] = ERPA_SYNCWORD;
	buffer[2] = uptime[0];
	buffer[3] = uptime[1];
	buffer[4] = uptime[2];
	buffer[5] = uptime[3];
	buffer[6] = ((erpa_seq >> 16) & 0xFF);
	buffer[7] = ((erpa_seq >> 8) & 0xFF);
	buffer[8] = erpa_seq & 0xFF;
	buffer[9] = sweep_step;
	buffer[10] = ((erpa_adc[0] & 0xFF00) >> 8);	// SWP Monitored MSB
	buffer[11] = (erpa_adc[0] & 0xFF);           // SWP Monitored LSB
	buffer[12] = erpa_spi[0];					// ERPA eADC MSB
	buffer[13] = erpa_spi[1];					// ERPA eADC LSB

	HAL_UART_Transmit(&huart1, buffer, ERPA_DATA_SIZE, UART_TIMEOUT_MS);

	erpa_seq++;
}

/**
 * @brief Creates and transmits a housekeeping (HK) packet.
 *
 * This function retrieves uptime and UNIX time information, samples the voltage rails,
 * constructs a packet with synchronization bytes, sequence number, and voltage readings,
 * and transmits it over UART.
 */
void create_hk_packet()
{
	VOLTAGE_RAIL *rail_monitor_ptr;
	uint8_t buffer[HK_DATA_SIZE];
	uint8_t timestamp[TIMESTAMP_SIZE];
	uint8_t uptime[UPTIME_SIZE];

	get_uptime(uptime);
	get_unix_time(timestamp);
	rail_monitor_ptr = get_rail_monitor();

	buffer[0] = HK_SYNCWORD;                     	// HK SYNC 0xCC MSB
	buffer[1] = HK_SYNCWORD;                     	// HK SYNC 0xCC LSB
	buffer[2] = timestamp[0];
	buffer[3] = timestamp[1];
	buffer[4] = timestamp[2];
	buffer[5] = timestamp[3];
	buffer[6] = timestamp[4];
	buffer[7] = timestamp[5];
	buffer[8] = uptime[0];
	buffer[9] = uptime[1];
	buffer[10] = uptime[2];
	buffer[11] = uptime[3];
	buffer[12] = ((hk_seq & 0xFF00) >> 8);    	// HK SEQ # MSB
	buffer[13] = (hk_seq & 0xFF);             	// HK SEQ # LSB
	buffer[14] = ((rail_monitor_ptr[RAIL_vsense].data & 0xFF00) >> 8);		// HK vsense MSB
	buffer[15] = (rail_monitor_ptr[RAIL_vsense].data & 0xFF);				// HK vsense LSB
	buffer[16] = ((rail_monitor_ptr[RAIL_vrefint].data & 0xFF00) >> 8);		// HK vrefint MSB
	buffer[17] = (rail_monitor_ptr[RAIL_vrefint].data & 0xFF);				// HK vrefint LSB
	buffer[18] = ((rail_monitor_ptr[RAIL_busvmon].data & 0xFF00) >> 8);	// HK BUSvmon MSB
	buffer[19] = (rail_monitor_ptr[RAIL_busvmon].data & 0xFF);				// HK BUSvmon LSB
	buffer[20] = ((rail_monitor_ptr[RAIL_busimon].data & 0xFF00) >> 8);	// HK BUSimon MSB
	buffer[21] = (rail_monitor_ptr[RAIL_busimon].data & 0xFF);				// HK BUSimon LSB
	buffer[22] = ((rail_monitor_ptr[RAIL_2v5].data & 0xFF00) >> 8);		// HK 2v5mon MSB
	buffer[23] = (rail_monitor_ptr[RAIL_2v5].data & 0xFF);					// HK 2v5mon LSB
	buffer[24] = ((rail_monitor_ptr[RAIL_5v].data & 0xFF00) >> 8);			// HK 5vmon MSB
	buffer[25] = (rail_monitor_ptr[RAIL_5v].data & 0xFF);					// HK 5vmon LSB
	buffer[26] = ((rail_monitor_ptr[RAIL_n3v3].data & 0xFF00) >> 8);		// HK n3v3mon MSB
	buffer[27] = (rail_monitor_ptr[RAIL_n3v3].data & 0xFF);				// HK n3v3mon LSB
	buffer[28] = ((rail_monitor_ptr[RAIL_n5v].data & 0xFF00) >> 8);		// HK n5vmon MSB
	buffer[29] = (rail_monitor_ptr[RAIL_n5v].data & 0xFF);					// HK n5vmon LSB
	buffer[30] = ((rail_monitor_ptr[RAIL_15v].data & 0xFF00) >> 8);		// HK 15vmon MSB
	buffer[31] = (rail_monitor_ptr[RAIL_15v].data & 0xFF);					// HK 15vmon LSB
	buffer[32] = ((rail_monitor_ptr[RAIL_5vref].data & 0xFF00) >> 8);		// HK 5vrefmon MSB
	buffer[33] = (rail_monitor_ptr[RAIL_5vref].data & 0xFF);				// HK 5vrefmon LSB
	buffer[34] = ((rail_monitor_ptr[RAIL_n200v].data & 0xFF00) >> 8);		// HK n150vmon MSB
	buffer[35] = (rail_monitor_ptr[RAIL_n200v].data & 0xFF);				// HK n150vmon LSB
	buffer[36] = ((rail_monitor_ptr[RAIL_n800v].data & 0xFF00) >> 8);		// HK n800vmon MSB
	buffer[37] = (rail_monitor_ptr[RAIL_n800v].data & 0xFF);				// HK n800vmon LSB
	buffer[38] = ((rail_monitor_ptr[RAIL_TEMP1].data & 0xFF00) >> 8);	// HK TEMP1 MSB
	buffer[39] = (rail_monitor_ptr[RAIL_TEMP1].data & 0xFF);				// HK TEMP1 LSB
	buffer[40] = ((rail_monitor_ptr[RAIL_TEMP2].data & 0xFF00) >> 8);	// HK TEMP2 MSB
	buffer[41] = (rail_monitor_ptr[RAIL_TEMP2].data & 0xFF);			// HK TEMP2 LSB
	buffer[42] = ((rail_monitor_ptr[RAIL_TEMP3].data & 0xFF00) >> 8);	// HK TEMP3 MSB
	buffer[43] = (rail_monitor_ptr[RAIL_TEMP3].data & 0xFF);			// HK TEMP3 LSB
	buffer[44] = ((rail_monitor_ptr[RAIL_TEMP4].data & 0xFF00) >> 8);	// HK TEMP4 MSB
	buffer[45] = (rail_monitor_ptr[RAIL_TEMP4].data & 0xFF);			// HK TEMP4 LSB
	buffer[46] = ((rail_monitor_ptr[RAIL_TMP1].data & 0xFF00) >> 8);  // TEMPURATURE 1 MSB
	buffer[47] = (rail_monitor_ptr[RAIL_TMP1].data & 0xFF);           // TEMPURATURE 1 LSB

	HAL_UART_Transmit(&huart1, buffer, HK_DATA_SIZE, UART_TIMEOUT_MS);

	hk_seq++;
}

/**
 * @brief Resets the packet sequence numbers for PMT, ERPA, and HK packets.
 *
 * This function sets the sequence counters for the PMT, ERPA, and housekeeping (HK) packets
 * back to zero. It is useful for reinitializing the packet transmission system.
 */
void reset_packet_sequence_numbers()
{
	pmt_seq = 0;
	erpa_seq = 0;
	hk_seq = 0;
}





