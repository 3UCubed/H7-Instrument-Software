/**
 ******************************************************************************
 * @file           : packet_creation.c
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Implementation file for creation of all data packets.
 ******************************************************************************
 */

#include "packet_creation.h"
#include "packet_queue.h"

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
	buffer[35] = ((local_cpy[ED_3v3] & 0xFF00) >> 8);
	buffer[36] = (local_cpy[ED_3v3] & 0xFF);
	buffer[37] = ((local_cpy[ED_5v] & 0xFF00) >> 8);
	buffer[38] = (local_cpy[ED_5v] & 0xFF);
	buffer[39] = ((local_cpy[ED_n3v3] & 0xFF00) >> 8);
	buffer[40] = (local_cpy[ED_n3v3] & 0xFF);
	buffer[41] = ((local_cpy[ED_n5v] & 0xFF00) >> 8);
	buffer[42] = (local_cpy[ED_n5v] & 0xFF);
	buffer[43] = ((local_cpy[ED_15v] & 0xFF00) >> 8);
	buffer[44] = (local_cpy[ED_15v] & 0xFF);
	buffer[45] = ((local_cpy[ED_5vref] & 0xFF00) >> 8);
	buffer[46] = (local_cpy[ED_5vref] & 0xFF);
	buffer[47] = ((local_cpy[ED_n200v] & 0xFF00) >> 8);
	buffer[48] = (local_cpy[ED_n200v] & 0xFF);
	buffer[49] = ((local_cpy[ED_n800v] & 0xFF00) >> 8);
	buffer[50] = (local_cpy[ED_n800v] & 0xFF);
	buffer[51] = ((local_cpy[ED_TMP1] & 0xFF00) >> 8);
	buffer[52] = (local_cpy[ED_TMP1] & 0xFF);
	buffer[53] = ((local_cpy[ED_single_bit_error_flash] & 0xFF00) >> 8);
	buffer[54] = (local_cpy[ED_single_bit_error_flash] & 0xFF);
	buffer[55] = ((local_cpy[ED_double_bit_error_flash] & 0xFF00) >> 8);
	buffer[56] = (local_cpy[ED_double_bit_error_flash] & 0xFF);
	buffer[57] = ((local_cpy[ED_single_bit_error_ram] & 0xFF00) >> 8);
	buffer[58] = (local_cpy[ED_single_bit_error_ram] & 0xFF);
	buffer[59] = ((local_cpy[ED_double_bit_error_ram] & 0xFF00) >> 8);
	buffer[60] = (local_cpy[ED_double_bit_error_ram] & 0xFF);
	buffer[61] = ((local_cpy[ED_UNDEFINED] & 0xFF00) >> 8);
	buffer[62] = (local_cpy[ED_UNDEFINED] & 0xFF);
	buffer[63] = reset_cause.category;
	buffer[64] = reset_cause.detail;

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

	static Packet_t pmt;
	pmt.size = PMT_DATA_SIZE;

	static uint8_t pmt_spi[2];
	static uint8_t uptime[UPTIME_SIZE];

	get_uptime(uptime);
	sample_pmt_spi(pmt_spi);

	pmt.buffer[0] = PMT_SYNCWORD;
	pmt.buffer[1] = PMT_SYNCWORD;
	pmt.buffer[2] = uptime[0];
	pmt.buffer[3] = uptime[1];
	pmt.buffer[4] = uptime[2];
	pmt.buffer[5] = uptime[3];
	pmt.buffer[6] = ((pmt_seq & 0xFF00) >> 8);
	pmt.buffer[7] = (pmt_seq & 0xFF);
	pmt.buffer[8] = pmt_spi[0];
	pmt.buffer[9] = pmt_spi[1];

	enqueue(pmt);

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

	static Packet_t erpa;
	erpa.size = ERPA_DATA_SIZE;

	static uint8_t erpa_spi[2];
	static uint16_t erpa_adc[1];
	static uint8_t uptime[UPTIME_SIZE];
	static STEP_VALUES sweep_step = INVALID_STEP;

	get_uptime(uptime);
	sweep_step = get_current_step();

	sample_erpa_spi(erpa_spi);
	sample_erpa_adc(erpa_adc);

	erpa.buffer[0] = ERPA_SYNCWORD;
	erpa.buffer[1] = ERPA_SYNCWORD;
	erpa.buffer[2] = uptime[0];
	erpa.buffer[3] = uptime[1];
	erpa.buffer[4] = uptime[2];
	erpa.buffer[5] = uptime[3];
	erpa.buffer[6] = ((erpa_seq >> 16) & 0xFF);
	erpa.buffer[7] = ((erpa_seq >> 8) & 0xFF);
	erpa.buffer[8] = erpa_seq & 0xFF;
	erpa.buffer[9] = sweep_step;
	erpa.buffer[10] = ((erpa_adc[0] & 0xFF00) >> 8);	// SWP Monitored MSB
	erpa.buffer[11] = (erpa_adc[0] & 0xFF);           // SWP Monitored LSB
	erpa.buffer[12] = erpa_spi[0];					// ERPA eADC MSB
	erpa.buffer[13] = erpa_spi[1];					// ERPA eADC LSB

	enqueue(erpa);

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
	static Packet_t hk;
	hk.size = HK_DATA_SIZE;

	static VOLTAGE_RAIL *rail_monitor_ptr;
	static uint8_t timestamp[TIMESTAMP_SIZE];
	static uint8_t uptime[UPTIME_SIZE];

	get_uptime(uptime);
	get_unix_time(timestamp);
	rail_monitor_ptr = get_rail_monitor();

	hk.buffer[0] = HK_SYNCWORD;                     	// HK SYNC 0xCC MSB
	hk.buffer[1] = HK_SYNCWORD;                     	// HK SYNC 0xCC LSB
	hk.buffer[2] = timestamp[0];
	hk.buffer[3] = timestamp[1];
	hk.buffer[4] = timestamp[2];
	hk.buffer[5] = timestamp[3];
	hk.buffer[6] = timestamp[4];
	hk.buffer[7] = timestamp[5];
	hk.buffer[8] = uptime[0];
	hk.buffer[9] = uptime[1];
	hk.buffer[10] = uptime[2];
	hk.buffer[11] = uptime[3];
	hk.buffer[12] = ((hk_seq & 0xFF00) >> 8);    	// HK SEQ # MSB
	hk.buffer[13] = (hk_seq & 0xFF);             	// HK SEQ # LSB
	hk.buffer[14] = ((rail_monitor_ptr[RAIL_vsense].data & 0xFF00) >> 8);		// HK vsense MSB
	hk.buffer[15] = (rail_monitor_ptr[RAIL_vsense].data & 0xFF);				// HK vsense LSB
	hk.buffer[16] = ((rail_monitor_ptr[RAIL_vrefint].data & 0xFF00) >> 8);		// HK vrefint MSB
	hk.buffer[17] = (rail_monitor_ptr[RAIL_vrefint].data & 0xFF);				// HK vrefint LSB
	hk.buffer[18] = ((rail_monitor_ptr[RAIL_busvmon].data & 0xFF00) >> 8);	// HK BUSvmon MSB
	hk.buffer[19] = (rail_monitor_ptr[RAIL_busvmon].data & 0xFF);				// HK BUSvmon LSB
	hk.buffer[20] = ((rail_monitor_ptr[RAIL_busimon].data & 0xFF00) >> 8);	// HK BUSimon MSB
	hk.buffer[21] = (rail_monitor_ptr[RAIL_busimon].data & 0xFF);				// HK BUSimon LSB
	hk.buffer[22] = ((rail_monitor_ptr[RAIL_2v5].data & 0xFF00) >> 8);		// HK 2v5mon MSB
	hk.buffer[23] = (rail_monitor_ptr[RAIL_2v5].data & 0xFF);					// HK 2v5mon LSB
	hk.buffer[24] = ((rail_monitor_ptr[RAIL_3v3].data & 0xFF00) >> 8);		// HK 3v3mon MSB
	hk.buffer[25] = (rail_monitor_ptr[RAIL_3v3].data & 0xFF);					// HK 3v3mon LSB
	hk.buffer[26] = ((rail_monitor_ptr[RAIL_5v].data & 0xFF00) >> 8);			// HK 5vmon MSB
	hk.buffer[27] = (rail_monitor_ptr[RAIL_5v].data & 0xFF);					// HK 5vmon LSB
	hk.buffer[28] = ((rail_monitor_ptr[RAIL_n3v3].data & 0xFF00) >> 8);		// HK n3v3mon MSB
	hk.buffer[29] = (rail_monitor_ptr[RAIL_n3v3].data & 0xFF);				// HK n3v3mon LSB
	hk.buffer[30] = ((rail_monitor_ptr[RAIL_n5v].data & 0xFF00) >> 8);		// HK n5vmon MSB
	hk.buffer[31] = (rail_monitor_ptr[RAIL_n5v].data & 0xFF);					// HK n5vmon LSB
	hk.buffer[32] = ((rail_monitor_ptr[RAIL_15v].data & 0xFF00) >> 8);		// HK 15vmon MSB
	hk.buffer[33] = (rail_monitor_ptr[RAIL_15v].data & 0xFF);					// HK 15vmon LSB
	hk.buffer[34] = ((rail_monitor_ptr[RAIL_5vref].data & 0xFF00) >> 8);		// HK 5vrefmon MSB
	hk.buffer[35] = (rail_monitor_ptr[RAIL_5vref].data & 0xFF);				// HK 5vrefmon LSB
	hk.buffer[36] = ((rail_monitor_ptr[RAIL_n200v].data & 0xFF00) >> 8);		// HK n150vmon MSB
	hk.buffer[37] = (rail_monitor_ptr[RAIL_n200v].data & 0xFF);				// HK n150vmon LSB
	hk.buffer[38] = ((rail_monitor_ptr[RAIL_n800v].data & 0xFF00) >> 8);		// HK n800vmon MSB
	hk.buffer[39] = (rail_monitor_ptr[RAIL_n800v].data & 0xFF);				// HK n800vmon LSB
	hk.buffer[40] = ((rail_monitor_ptr[RAIL_TEMP1].data & 0xFF00) >> 8);	// HK TEMP1 MSB
	hk.buffer[41] = (rail_monitor_ptr[RAIL_TEMP1].data & 0xFF);				// HK TEMP1 LSB
	hk.buffer[42] = ((rail_monitor_ptr[RAIL_TEMP2].data & 0xFF00) >> 8);	// HK TEMP2 MSB
	hk.buffer[43] = (rail_monitor_ptr[RAIL_TEMP2].data & 0xFF);			// HK TEMP2 LSB
	hk.buffer[44] = ((rail_monitor_ptr[RAIL_TEMP3].data & 0xFF00) >> 8);	// HK TEMP3 MSB
	hk.buffer[45] = (rail_monitor_ptr[RAIL_TEMP3].data & 0xFF);			// HK TEMP3 LSB
	hk.buffer[46] = ((rail_monitor_ptr[RAIL_TEMP4].data & 0xFF00) >> 8);	// HK TEMP4 MSB
	hk.buffer[47] = (rail_monitor_ptr[RAIL_TEMP4].data & 0xFF);			// HK TEMP4 LSB
	hk.buffer[48] = ((rail_monitor_ptr[RAIL_TMP1].data & 0xFF00) >> 8);  // TEMPURATURE 1 MSB
	hk.buffer[49] = (rail_monitor_ptr[RAIL_TMP1].data & 0xFF);           // TEMPURATURE 1 LSB

	enqueue(hk);

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





