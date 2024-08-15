/*
 * voltage_monitor.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_VOLTAGE_MONITOR_H_
#define INC_VOLTAGE_MONITOR_H_

#define NUM_VOLTAGE_RAILS 18

#include <stdio.h>				// For uint data types
#include "sample_data.h"
#include "shared_types.h"
// Private Typedefs


typedef struct {
	VOLTAGE_RAIL_NAME name;
	uint8_t error_count;
	uint8_t is_enabled;
	uint16_t data;
	uint16_t max_voltage;
	uint16_t min_voltage;
} VOLTAGE_RAIL;


uint8_t voltage_monitor_init();
uint8_t set_rail_monitor_enable(VOLTAGE_RAIL_NAME rail_name, uint8_t enable_value);
VOLTAGE_RAIL* get_rail_monitor();
uint8_t set_rail_monitor();
uint8_t in_range(uint16_t raw, int min, int max);

#endif /* INC_VOLTAGE_MONITOR_H_ */
