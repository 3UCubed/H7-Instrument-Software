/*
 * packet_creation.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_PACKET_CREATION_H_
#define INC_PACKET_CREATION_H_

#include "main.h"
#include "time_tagging.h"	// For getting uptime and timestamps during packet creation
#include "sample_data.h"	// For sampling data

packet_t packetize(const uint8_t *data, uint16_t size);
void create_pmt_packet();

#endif /* INC_PACKET_CREATION_H_ */
