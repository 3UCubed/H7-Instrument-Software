/*
 * packet_creation.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_PACKET_CREATION_H_
#define INC_PACKET_CREATION_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "voltage_monitor.h"
#include "time_tagging.h"
typedef struct {
	uint8_t *array;  // Pointer to the array data
	uint16_t size;   // Size of the array
} packet_t;

packet_t create_packet(const uint8_t *data, uint16_t size);
uint8_t get_current_step();
void sample_pmt();
void sample_erpa();
void sample_hk();




#endif /* INC_PACKET_CREATION_H_ */
