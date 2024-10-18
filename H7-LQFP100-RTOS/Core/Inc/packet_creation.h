/**
 ******************************************************************************
 * @file           : packet_creation.h
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Header file for packet creation
 ******************************************************************************
 */

#ifndef INC_PACKET_CREATION_H_
#define INC_PACKET_CREATION_H_

#include "time_tagging.h"
#include "usart.h"
#include "error_packet_handler.h"

#define SYNC_DATA_SIZE 65
#define VERSION_DATA_SIZE 5
#define PMT_DATA_SIZE 10
#define ERPA_DATA_SIZE 14
#define HK_DATA_SIZE 50

typedef struct
{
	uint8_t buffer[HK_DATA_SIZE];	// Size of largest packet
	uint8_t size;
}Packet_t;

void create_sync_packet(ERROR_STRUCT reset_cause);
void create_version_packet();
void create_pmt_packet();
void create_erpa_packet();
void create_hk_packet();
void reset_packet_sequence_numbers();

#endif /* INC_PACKET_CREATION_H_ */
