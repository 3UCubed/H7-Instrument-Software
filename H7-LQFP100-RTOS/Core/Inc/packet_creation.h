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

void create_pmt_packet();
void create_erpa_packet();
void create_hk_packet();
void reset_packet_sequence_numbers();

#endif /* INC_PACKET_CREATION_H_ */
