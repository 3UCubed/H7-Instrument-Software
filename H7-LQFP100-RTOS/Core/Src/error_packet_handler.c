/*
 * error_packet_handler.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "error_packet_handler.h"

osMessageQueueId_t mid_MsgQueue;


// Public Functions
void error_protocol(VOLTAGE_RAIL_NAME failed_rail) {

	packet_t error_packet;
	uint8_t *buffer = (uint8_t*) malloc(
	ERROR_PACKET_DATA_SIZE * sizeof(uint8_t));

	buffer[0] = ERROR_SYNC;
	buffer[1] = ERROR_SYNC;
	buffer[2] = failed_rail;

	error_packet = create_packet(buffer, ERROR_PACKET_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &error_packet, 0U, 0U);

	free(buffer);
	//vTaskSuspendAll();
	//TODO: Shutdown
}
