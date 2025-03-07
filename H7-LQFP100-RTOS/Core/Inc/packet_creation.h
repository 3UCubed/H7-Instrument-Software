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

void create_sync_packet(ERROR_STRUCT reset_cause);
void create_version_packet();
void create_pmt_packet();
void create_erpa_packet();
void create_hk_packet();
void reset_packet_sequence_numbers();

extern uint16_t pmt_seq;
extern uint32_t erpa_seq;
extern uint16_t hk_seq;

#endif /* INC_PACKET_CREATION_H_ */
