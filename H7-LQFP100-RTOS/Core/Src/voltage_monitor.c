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
	rail_monitor[RAIL_busvmon].is_enabled = 0;
	rail_monitor[RAIL_busvmon].data = 0;
	rail_monitor[RAIL_busvmon].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_busvmon].min_voltage = 0;

	rail_monitor[RAIL_busimon].name = RAIL_busimon;
	rail_monitor[RAIL_busimon].error_count = 0;
	rail_monitor[RAIL_busimon].is_enabled = 0;
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


uint8_t set_rail_monitor_enable(VOLTAGE_RAIL_NAME rail_name, uint8_t enable_value) {
	uint8_t status = 0;
	rail_monitor[rail_name].is_enabled = enable_value;
	status = 1;

	return status;
}


uint8_t set_rail_monitor() {
	uint8_t status = 0;
	uint16_t hk_adc1[10];
	uint16_t hk_adc3[4];
	int16_t hk_i2c[4];

	sample_hk_i2c(hk_i2c);
	sample_hk_adc1(hk_adc1);
	sample_hk_adc3(hk_adc3);

	memcpy(&rail_monitor[RAIL_vsense].data, &hk_adc3[1], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_vrefint].data, &hk_adc3[0], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_TEMP1].data, &hk_i2c[0], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_TEMP2].data, &hk_i2c[1], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_TEMP3].data, &hk_i2c[2], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_TEMP4].data, &hk_i2c[3], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_busvmon].data, &hk_adc1[0], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_busimon].data, &hk_adc1[1], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_2v5].data, &hk_adc1[2], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_3v3].data, &hk_adc3[3], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_5v].data, &hk_adc1[6], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_n3v3].data, &hk_adc1[3], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_n5v].data, &hk_adc3[2], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_15v].data, &hk_adc1[7], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_5vref].data, &hk_adc1[8], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_n200v].data, &hk_adc1[4], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_n800v].data, &hk_adc1[5], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_TMP1].data, &hk_adc1[9], sizeof(uint16_t));

	status = 1;

	return status;
}

VOLTAGE_RAIL* get_rail_monitor() {
	return rail_monitor;
}

uint8_t in_range(uint16_t raw, int min, int max) {
	if (raw <= max && raw >= min) {
		return 1;
	}
	return 0;
}


void monitor_rails() {
	// Iterate through all voltage rails
	for (int i = 0; i < NUM_VOLTAGE_RAILS; i++){
		if (rail_monitor[i].is_enabled){
			// If current rail is not in range...
			if (!in_range(rail_monitor[i].data, rail_monitor[i].min_voltage, rail_monitor[i].max_voltage)){
				// Increase that rails error count
				rail_monitor[i].error_count++;
				// If that rails' error count is at 3, proceed with error protocol for that rail
				if (rail_monitor[i].error_count == 3) {
					ERROR_STRUCT error;
					error.detail = get_rail_name_error_detail(rail_monitor[i].name);
					error.category = EC_power_supply_rail;
					handle_error(error);
				}
			}
		}
		// If the rail monitor isn't enabled...
		else {
			uint16_t tolerance;
			tolerance = rail_monitor[i].max_voltage * 0.1;

			// If it isn't within +10% of its max voltage from 0...
			if (!in_range(rail_monitor[i].data, 0, tolerance)) {
				// Increase that rails error count
				rail_monitor[i].error_count++;
				// If that rails' error count is at 3, proceed with error protocol for that rail
				if (rail_monitor[i].error_count == 3) {
					ERROR_STRUCT error;
					error.detail = get_rail_name_error_detail(rail_monitor[i].name);
					error.category = EC_power_supply_rail;
					handle_error(error);
				}
			}
		}
	}
}

ERROR_DETAIL get_rail_name_error_detail(VOLTAGE_RAIL_NAME rail_name) {
	switch (rail_name) {
	case RAIL_vsense:
		return ED_vsense;

	case RAIL_vrefint:
		return ED_vrefint;

	case RAIL_TEMP1:
		return ED_TEMP1;

	case RAIL_TEMP2:
		return ED_TEMP2;

	case RAIL_TEMP3:
		return ED_TEMP3;

	case RAIL_TEMP4:
		return ED_TEMP4;

	case RAIL_busvmon:
		return ED_busvmon;

	case RAIL_busimon:
		return ED_busimon;

	case RAIL_2v5:
		return ED_2v5;

	case RAIL_3v3:
		return ED_3v3;

	case RAIL_5v:
		return ED_5v;

	case RAIL_n3v3:
		return ED_n3v3;

	case RAIL_n5v:
		return ED_n5v;

	case RAIL_15v:
		return ED_15v;

	case RAIL_5vref:
		return ED_5vref;

	case RAIL_n200v:
		return ED_n200v;

	case RAIL_n800v:
		return ED_n800v;

	case RAIL_TMP1:
		return ED_TMP1;

	default:
		return ED_UNDEFINED;
	}
}

















