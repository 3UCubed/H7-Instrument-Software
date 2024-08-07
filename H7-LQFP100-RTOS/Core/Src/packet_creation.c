/*
 * packet_creation.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "packet_creation.h"

uint16_t pmt_seq = 0;
uint32_t erpa_seq = 0;


packet_t packetize(const uint8_t *data, uint16_t size) {
	packet_t packet;
	packet.array = (uint8_t*) malloc(size * sizeof(uint8_t));
	if (packet.array == NULL) {
		// Packet array is null somehow, should probably do something about this edge case
	}
	memcpy(packet.array, data, size);
	packet.size = size;
	return packet;
}

void create_pmt_packet() {
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
	}
	uint8_t *buffer = (uint8_t*) malloc(PMT_DATA_SIZE * sizeof(uint8_t));
	uint8_t *pmt_spi = (uint8_t*) malloc(2 * sizeof(uint8_t));
	uint8_t *uptime = (uint8_t*) malloc(UPTIME_SIZE * sizeof(uint8_t));

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

	packet_t pmt_packet = packetize(buffer, PMT_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &pmt_packet, 0U, 0U);
	pmt_seq++;
	free(buffer);
	free(pmt_spi);
	free(uptime);
}


void create_erpa_packet() {
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) {
	}

	uint8_t *buffer = (uint8_t*) malloc(ERPA_DATA_SIZE * sizeof(uint8_t)); // Allocate memory for the buffer

	uint8_t *erpa_spi = (uint8_t*) malloc(2 * sizeof(uint8_t));
	uint16_t *erpa_adc = (uint16_t*) malloc(1 * sizeof(uint16_t));
	uint8_t *uptime = (uint8_t*) malloc(UPTIME_SIZE * sizeof(uint8_t));
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

	packet_t erpa_packet = packetize(buffer, ERPA_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &erpa_packet, 0U, 0U);
	erpa_seq++;
	free(buffer);
	free(erpa_spi);
	free(erpa_adc);
	free(uptime);
}
