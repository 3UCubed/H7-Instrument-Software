/*
 * voltage_monitor.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_VOLTAGE_MONITOR_H_
#define INC_VOLTAGE_MONITOR_H_

#define NUM_VOLTAGE_RAILS 18

#include "main.h"

// Private Typedefs
typedef enum {
	RAIL_vsense,	// 0
	RAIL_vrefint,	// 1
	RAIL_TEMP1,		// 2
	RAIL_TEMP2,		// 3
	RAIL_TEMP3,		// 4
	RAIL_TEMP4,		// 5
	RAIL_busvmon,	// 6
	RAIL_busimon,	// 7
	RAIL_2v5,		// 8
	RAIL_3v3,		// 9
	RAIL_5v,		// 10
	RAIL_n3v3,		// 11
	RAIL_n5v,		// 12
	RAIL_15v,		// 13
	RAIL_5vref,		// 14
	RAIL_n200v,		// 15
	RAIL_n800v,		// 16
	RAIL_TMP1		// 17
} VOLTAGE_RAIL_NAME;

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
