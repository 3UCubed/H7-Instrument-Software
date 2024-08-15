/*
 * packet_creation.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_PACKET_CREATION_H_
#define INC_PACKET_CREATION_H_

#include "voltage_monitor.h"
#include "time_tagging.h"
#include "usart.h"

#define ERROR_PACKET_SIZE 3
#define JUNK_PACKET_SIZE 256
#define PMT_DATA_SIZE 10
#define ERPA_DATA_SIZE 14
#define HK_DATA_SIZE 54
#define UPTIME_SIZE 4
#define TIMESTAMP_SIZE 10

#define PMT_SYNC 0xBB
#define ERPA_SYNC 0xAA
#define HK_SYNC 0xCC
#define ERROR_PACKET_SYNC 0xDD

void create_pmt_packet();
void create_erpa_packet();
void create_hk_packet();
void create_error_packet(VOLTAGE_RAIL_NAME rail_name);
void create_junk_packet();
void reset_packet_sequence_numbers();

#endif /* INC_PACKET_CREATION_H_ */
