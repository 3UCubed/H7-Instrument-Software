/*
 * voltage_monitor.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "voltage_monitor.h"

VOLTAGE_RAIL rail_monitor[NUM_VOLTAGE_RAILS];

uint8_t voltage_monitor_init() {
	uint8_t status = 0;

	rail_monitor[RAIL_vsense].name = RAIL_vsense;
	rail_monitor[RAIL_vsense].error_count = 0;
	rail_monitor[RAIL_vsense].is_enabled = 1;
	rail_monitor[RAIL_vsense].data = 0;
	rail_monitor[RAIL_vsense].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_vsense].min_voltage = 0;

	rail_monitor[RAIL_vrefint].name = RAIL_vrefint;
	rail_monitor[RAIL_vrefint].error_count = 0;
	rail_monitor[RAIL_vrefint].is_enabled = 1;
	rail_monitor[RAIL_vrefint].data = 0;
	rail_monitor[RAIL_vrefint].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_vrefint].min_voltage = 0;

	rail_monitor[RAIL_TEMP1].name = RAIL_TEMP1;
	rail_monitor[RAIL_TEMP1].error_count = 0;
	rail_monitor[RAIL_TEMP1].is_enabled = 1;
	rail_monitor[RAIL_TEMP1].data = 0;
	rail_monitor[RAIL_TEMP1].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TEMP1].min_voltage = 0;

	rail_monitor[RAIL_TEMP2].name = RAIL_TEMP2;
	rail_monitor[RAIL_TEMP2].error_count = 0;
	rail_monitor[RAIL_TEMP2].is_enabled = 1;
	rail_monitor[RAIL_TEMP2].data = 0;
	rail_monitor[RAIL_TEMP2].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TEMP2].min_voltage = 0;

	rail_monitor[RAIL_TEMP3].name = RAIL_TEMP3;
	rail_monitor[RAIL_TEMP3].error_count = 0;
	rail_monitor[RAIL_TEMP3].is_enabled = 1;
	rail_monitor[RAIL_TEMP3].data = 0;
	rail_monitor[RAIL_TEMP3].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TEMP3].min_voltage = 0;

	rail_monitor[RAIL_TEMP4].name = RAIL_TEMP4;
	rail_monitor[RAIL_TEMP4].error_count = 0;
	rail_monitor[RAIL_TEMP4].is_enabled = 1;
	rail_monitor[RAIL_TEMP4].data = 0;
	rail_monitor[RAIL_TEMP4].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TEMP4].min_voltage = 0;

	rail_monitor[RAIL_busvmon].name = RAIL_busvmon;
	rail_monitor[RAIL_busvmon].error_count = 0;
	rail_monitor[RAIL_busvmon].is_enabled = 1;
	rail_monitor[RAIL_busvmon].data = 0;
	rail_monitor[RAIL_busvmon].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_busvmon].min_voltage = 0;

	rail_monitor[RAIL_busimon].name = RAIL_busimon;
	rail_monitor[RAIL_busimon].error_count = 0;
	rail_monitor[RAIL_busimon].is_enabled = 1;
	rail_monitor[RAIL_busimon].data = 0;
	rail_monitor[RAIL_busimon].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_busimon].min_voltage = 0;

	rail_monitor[RAIL_2v5].name = RAIL_2v5;
	rail_monitor[RAIL_2v5].error_count = 0;
	rail_monitor[RAIL_2v5].is_enabled = 0;
	rail_monitor[RAIL_2v5].data = 0;
	rail_monitor[RAIL_2v5].max_voltage = 3257;
	rail_monitor[RAIL_2v5].min_voltage = 2947;

	rail_monitor[RAIL_3v3].name = RAIL_3v3;
	rail_monitor[RAIL_3v3].error_count = 0;
	rail_monitor[RAIL_3v3].is_enabled = 0;
	rail_monitor[RAIL_3v3].data = 0;
	rail_monitor[RAIL_3v3].max_voltage = 3909;
	//rail_monitor[RAIL_3v3].min_voltage = 3537;
	rail_monitor[RAIL_3v3].min_voltage = 0;


	rail_monitor[RAIL_5v].name = RAIL_5v;
	rail_monitor[RAIL_5v].error_count = 0;
	rail_monitor[RAIL_5v].is_enabled = 0;
	rail_monitor[RAIL_5v].data = 0;
	rail_monitor[RAIL_5v].max_voltage = 3909;
	rail_monitor[RAIL_5v].min_voltage = 3537;

	rail_monitor[RAIL_n3v3].name = RAIL_n3v3;
	rail_monitor[RAIL_n3v3].error_count = 0;
	rail_monitor[RAIL_n3v3].is_enabled = 0;
	rail_monitor[RAIL_n3v3].data = 0;
	rail_monitor[RAIL_n3v3].max_voltage = 4091;
	rail_monitor[RAIL_n3v3].min_voltage = 3702;

	rail_monitor[RAIL_n5v].name = RAIL_n5v;
	rail_monitor[RAIL_n5v].error_count = 0;
	rail_monitor[RAIL_n5v].is_enabled = 0;
	rail_monitor[RAIL_n5v].data = 0;
	rail_monitor[RAIL_n5v].max_voltage = 4000;
	//rail_monitor[RAIL_n5v].min_voltage = 3619;
	rail_monitor[RAIL_n5v].min_voltage = 0;

	rail_monitor[RAIL_15v].name = RAIL_15v;
	rail_monitor[RAIL_15v].error_count = 0;
	rail_monitor[RAIL_15v].is_enabled = 0;
	rail_monitor[RAIL_15v].data = 0;
	rail_monitor[RAIL_15v].max_voltage = 3896;
	rail_monitor[RAIL_15v].min_voltage = 3525;

	rail_monitor[RAIL_5vref].name = RAIL_5vref;
	rail_monitor[RAIL_5vref].error_count = 0;
	rail_monitor[RAIL_5vref].is_enabled = 0;
	rail_monitor[RAIL_5vref].data = 0;
	rail_monitor[RAIL_5vref].max_voltage = 3909;
	rail_monitor[RAIL_5vref].min_voltage = 3537;

	rail_monitor[RAIL_n200v].name = RAIL_n200v;
	rail_monitor[RAIL_n200v].error_count = 0;
	rail_monitor[RAIL_n200v].is_enabled = 0;
	rail_monitor[RAIL_n200v].data = 0;
	rail_monitor[RAIL_n200v].max_voltage = 4196;
	//rail_monitor[RAIL_n200v].min_voltage = 3796;
	rail_monitor[RAIL_n200v].min_voltage = 0;		// TODO: Currently set to 0, kept triggering because it has been reading ~3351


	rail_monitor[RAIL_n800v].name = RAIL_n800v;
	rail_monitor[RAIL_n800v].error_count = 0;
	rail_monitor[RAIL_n800v].is_enabled = 0;
	rail_monitor[RAIL_n800v].data = 0;
	rail_monitor[RAIL_n800v].max_voltage = 3336;
	rail_monitor[RAIL_n800v].min_voltage = 3018;

	rail_monitor[RAIL_TMP1].name = RAIL_TMP1;
	rail_monitor[RAIL_TMP1].error_count = 0;
	rail_monitor[RAIL_TMP1].is_enabled = 1;
	rail_monitor[RAIL_TMP1].data = 0;
	rail_monitor[RAIL_TMP1].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TMP1].min_voltage = 0;

	status = 1;

	return status;
}
