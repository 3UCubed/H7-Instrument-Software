/*
 * packet_creation.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_PACKET_CREATION_H_
#define INC_PACKET_CREATION_H_

#include "time_tagging.h"
#include "usart.h"

#define PMT_DATA_SIZE 10
#define ERPA_DATA_SIZE 14
#define HK_DATA_SIZE 46
#define UPTIME_SIZE 4
#define TIMESTAMP_SIZE 6

#define PMT_SYNC 0xFF
#define ERPA_SYNC 0xEE
#define HK_SYNC 0xDD

void create_pmt_packet();
void create_erpa_packet();
void create_hk_packet();
void reset_packet_sequence_numbers();

#endif /* INC_PACKET_CREATION_H_ */
