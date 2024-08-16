/*
 * packet_creation.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "packet_creation.h"

uint16_t pmt_seq = 0;
uint32_t erpa_seq = 0;
uint16_t hk_seq = 0;



void create_pmt_packet() {
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
	}
	uint8_t buffer[PMT_DATA_SIZE];
	uint8_t pmt_spi[2];
	uint8_t uptime[UPTIME_SIZE];

	get_uptime(uptime);
	sample_pmt_spi(pmt_spi);

	buffer[0] = PMT_SYNC;
	buffer[1] = PMT_SYNC;
	buffer[2] = ((pmt_seq & 0xFF00) >> 8);
	buffer[3] = (pmt_seq & 0xFF);
	buffer[4] = pmt_spi[0];
	buffer[5] = pmt_spi[1];
	buffer[6] = uptime[0];
	buffer[7] = uptime[1];
	buffer[8] = uptime[2];
	buffer[9] = uptime[3];

	HAL_UART_Transmit(&huart1, buffer, PMT_DATA_SIZE, 100);

	pmt_seq++;
}


void create_erpa_packet() {
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) {
	}

	uint8_t buffer[ERPA_DATA_SIZE];
	uint8_t erpa_spi[2];
	uint16_t erpa_adc[1];
	uint8_t uptime[UPTIME_SIZE];
	uint8_t sweep_step = -1;

	get_uptime(uptime);
	sweep_step = get_current_step();

	sample_erpa_spi(erpa_spi);
	sample_erpa_adc(erpa_adc);

	buffer[0] = ERPA_SYNC;
	buffer[1] = ERPA_SYNC;
	buffer[2] = ((erpa_seq >> 16) & 0xFF);
	buffer[3] = ((erpa_seq >> 8) & 0xFF);
	buffer[4] = erpa_seq & 0xFF;
	buffer[5] = sweep_step;
	buffer[6] = ((erpa_adc[0] & 0xFF00) >> 8);	// SWP Monitored MSB
	buffer[7] = (erpa_adc[0] & 0xFF);           // SWP Monitored LSB
	buffer[8] = erpa_spi[0];					// ERPA eADC MSB
	buffer[9] = erpa_spi[1];					// ERPA eADC LSB
	buffer[10] = uptime[0];
	buffer[11] = uptime[1];
	buffer[12] = uptime[2];
	buffer[13] = uptime[3];

	HAL_UART_Transmit(&huart1, buffer, ERPA_DATA_SIZE, 100);

	erpa_seq++;
}


void create_hk_packet() {
	VOLTAGE_RAIL *rail_monitor_ptr;
	uint8_t buffer[HK_DATA_SIZE];
	uint8_t timestamp[TIMESTAMP_SIZE];
	uint8_t uptime[UPTIME_SIZE];

	get_uptime(uptime);
	get_timestamp(timestamp);
	rail_monitor_ptr = get_rail_monitor();


	buffer[0] = HK_SYNC;                     	// HK SYNC 0xCC MSB
	buffer[1] = HK_SYNC;                     	// HK SYNC 0xCC LSB
	buffer[2] = ((hk_seq & 0xFF00) >> 8);    	// HK SEQ # MSB
	buffer[3] = (hk_seq & 0xFF);             	// HK SEQ # LSB
	buffer[4] = ((rail_monitor_ptr[RAIL_vsense].data & 0xFF00) >> 8);		// HK vsense MSB
	buffer[5] = (rail_monitor_ptr[RAIL_vsense].data & 0xFF);				// HK vsense LSB
	buffer[6] = ((rail_monitor_ptr[RAIL_vrefint].data & 0xFF00) >> 8);		// HK vrefint MSB
	buffer[7] = (rail_monitor_ptr[RAIL_vrefint].data & 0xFF);				// HK vrefint LSB
	buffer[8] = ((rail_monitor_ptr[RAIL_TEMP1].data & 0xFF00) >> 8);	// HK TEMP1 MSB
	buffer[9] = (rail_monitor_ptr[RAIL_TEMP1].data & 0xFF);				// HK TEMP1 LSB
	buffer[10] = ((rail_monitor_ptr[RAIL_TEMP2].data & 0xFF00) >> 8);	// HK TEMP2 MSB
	buffer[11] = (rail_monitor_ptr[RAIL_TEMP2].data & 0xFF);			// HK TEMP2 LSB
	buffer[12] = ((rail_monitor_ptr[RAIL_TEMP3].data & 0xFF00) >> 8);	// HK TEMP3 MSB
	buffer[13] = (rail_monitor_ptr[RAIL_TEMP3].data & 0xFF);			// HK TEMP3 LSB
	buffer[14] = ((rail_monitor_ptr[RAIL_TEMP4].data & 0xFF00) >> 8);	// HK TEMP4 MSB
	buffer[15] = (rail_monitor_ptr[RAIL_TEMP4].data & 0xFF);			// HK TEMP4 LSB
	buffer[16] = ((rail_monitor_ptr[RAIL_busvmon].data & 0xFF00) >> 8);	// HK BUSvmon MSB
	buffer[17] = (rail_monitor_ptr[RAIL_busvmon].data & 0xFF);				// HK BUSvmon LSB
	buffer[18] = ((rail_monitor_ptr[RAIL_busimon].data & 0xFF00) >> 8);	// HK BUSimon MSB
	buffer[19] = (rail_monitor_ptr[RAIL_busimon].data & 0xFF);				// HK BUSimon LSB
	buffer[20] = ((rail_monitor_ptr[RAIL_2v5].data & 0xFF00) >> 8);		// HK 2v5mon MSB
	buffer[21] = (rail_monitor_ptr[RAIL_2v5].data & 0xFF);					// HK 2v5mon LSB
	buffer[22] = ((rail_monitor_ptr[RAIL_3v3].data & 0xFF00) >> 8);		// HK 3v3mon MSB
	buffer[23] = (rail_monitor_ptr[RAIL_3v3].data & 0xFF);					// HK 3v3mon LSB
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
	buffer[38] = ((rail_monitor_ptr[RAIL_TMP1].data & 0xFF00) >> 8);  // TEMPURATURE 1 MSB
	buffer[39] = (rail_monitor_ptr[RAIL_TMP1].data & 0xFF);           // TEMPURATURE 1 LSB
	buffer[40] = timestamp[0];
	buffer[41] = timestamp[1];
	buffer[42] = timestamp[2];
	buffer[43] = timestamp[3];
	buffer[44] = timestamp[4];
	buffer[45] = timestamp[5];
	buffer[46] = timestamp[6];
	buffer[47] = timestamp[7];
	buffer[48] = timestamp[8];
	buffer[49] = timestamp[9];
	buffer[50] = uptime[0];
	buffer[51] = uptime[1];
	buffer[52] = uptime[2];
	buffer[53] = uptime[3];

	HAL_UART_Transmit(&huart1, buffer, HK_DATA_SIZE, 100);

	hk_seq++;
}

void create_error_packet(VOLTAGE_RAIL_NAME rail_name) {
	uint8_t buffer[ERROR_PACKET_SIZE];

	buffer[0] = ERROR_PACKET_SYNC;
	buffer[1] = ERROR_PACKET_SYNC;
	buffer[2] = rail_name;

	HAL_UART_Transmit(&huart1, buffer, ERROR_PACKET_SIZE, 100);
}

void create_junk_packet() {
	uint8_t buffer[JUNK_PACKET_SIZE];

	for (int i = 0; i < JUNK_PACKET_SIZE; i++) {
		buffer[i] = 0x00;
	}

	HAL_UART_Transmit(&huart1, buffer, JUNK_PACKET_SIZE, 100);
}



void reset_packet_sequence_numbers() {
	pmt_seq = 0;
	erpa_seq = 0;
	hk_seq = 0;
}





