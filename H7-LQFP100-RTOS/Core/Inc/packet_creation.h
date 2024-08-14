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
#include "usart.h"

void create_pmt_packet();
void create_erpa_packet();
void create_hk_packet();
void reset_packet_sequence_numbers();

#endif /* INC_PACKET_CREATION_H_ */
