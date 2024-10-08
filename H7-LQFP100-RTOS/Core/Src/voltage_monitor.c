/*
 * voltage_monitor.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "voltage_monitor.h"

VOLTAGE_RAIL rail_monitor[NUM_VOLTAGE_RAILS];

void voltage_monitor_init() {

	rail_monitor[RAIL_vsense].name = RAIL_vsense;
	rail_monitor[RAIL_vsense].error_count = 0;
	rail_monitor[RAIL_vsense].is_enabled = 1;
	rail_monitor[RAIL_vsense].data = 0;
	rail_monitor[RAIL_vsense].max_voltage = 906;		// 85c
	rail_monitor[RAIL_vsense].min_voltage = 596;		// -40c
	rail_monitor[RAIL_vsense].OOB_1 = 0;
	rail_monitor[RAIL_vsense].OOB_2 = 0;
	rail_monitor[RAIL_vsense].OOB_3 = 0;


	rail_monitor[RAIL_vrefint].name = RAIL_vrefint;
	rail_monitor[RAIL_vrefint].error_count = 0;
	rail_monitor[RAIL_vrefint].is_enabled = 1;
	rail_monitor[RAIL_vrefint].data = 0;
	rail_monitor[RAIL_vrefint].max_voltage = 1557;		// 1.255v
	rail_monitor[RAIL_vrefint].min_voltage = 1464;		// 1.180v
	rail_monitor[RAIL_vrefint].OOB_1 = 0;
	rail_monitor[RAIL_vrefint].OOB_2 = 0;
	rail_monitor[RAIL_vrefint].OOB_3 = 0;

	rail_monitor[RAIL_TEMP1].name = RAIL_TEMP1;
	rail_monitor[RAIL_TEMP1].error_count = 0;
	rail_monitor[RAIL_TEMP1].is_enabled = 1;
	rail_monitor[RAIL_TEMP1].data = 0;
	rail_monitor[RAIL_TEMP1].max_voltage = 800;			// 50c
	rail_monitor[RAIL_TEMP1].min_voltage = 7872;		// -20c
	rail_monitor[RAIL_TEMP1].OOB_1 = 0;
	rail_monitor[RAIL_TEMP1].OOB_2 = 0;
	rail_monitor[RAIL_TEMP1].OOB_3 = 0;

	rail_monitor[RAIL_TEMP2].name = RAIL_TEMP2;
	rail_monitor[RAIL_TEMP2].error_count = 0;
	rail_monitor[RAIL_TEMP2].is_enabled = 1;
	rail_monitor[RAIL_TEMP2].data = 0;
	rail_monitor[RAIL_TEMP2].max_voltage = 800;			// 50c
	rail_monitor[RAIL_TEMP2].min_voltage = 7872;		// -20c
	rail_monitor[RAIL_TEMP2].OOB_1 = 0;
	rail_monitor[RAIL_TEMP2].OOB_2 = 0;
	rail_monitor[RAIL_TEMP2].OOB_3 = 0;

	rail_monitor[RAIL_TEMP3].name = RAIL_TEMP3;
	rail_monitor[RAIL_TEMP3].error_count = 0;
	rail_monitor[RAIL_TEMP3].is_enabled = 1;
	rail_monitor[RAIL_TEMP3].data = 0;
	rail_monitor[RAIL_TEMP3].max_voltage = 800;			// 50c
	rail_monitor[RAIL_TEMP3].min_voltage = 7872;		// -20c
	rail_monitor[RAIL_TEMP3].OOB_1 = 0;
	rail_monitor[RAIL_TEMP3].OOB_2 = 0;
	rail_monitor[RAIL_TEMP3].OOB_3 = 0;

	rail_monitor[RAIL_TEMP4].name = RAIL_TEMP4;
	rail_monitor[RAIL_TEMP4].error_count = 0;
	rail_monitor[RAIL_TEMP4].is_enabled = 1;
	rail_monitor[RAIL_TEMP4].data = 0;
	rail_monitor[RAIL_TEMP4].max_voltage = 800;			// 50c
	rail_monitor[RAIL_TEMP4].min_voltage = 7872;		// -20c
	rail_monitor[RAIL_TEMP4].OOB_1 = 0;
	rail_monitor[RAIL_TEMP4].OOB_2 = 0;
	rail_monitor[RAIL_TEMP4].OOB_3 = 0;

	rail_monitor[RAIL_busvmon].name = RAIL_busvmon;
	rail_monitor[RAIL_busvmon].error_count = 0;
	rail_monitor[RAIL_busvmon].is_enabled = 0;
	rail_monitor[RAIL_busvmon].data = 0;
	rail_monitor[RAIL_busvmon].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_busvmon].min_voltage = 0;
	rail_monitor[RAIL_busvmon].OOB_1 = 0;
	rail_monitor[RAIL_busvmon].OOB_2 = 0;
	rail_monitor[RAIL_busvmon].OOB_3 = 0;

	rail_monitor[RAIL_busimon].name = RAIL_busimon;
	rail_monitor[RAIL_busimon].error_count = 0;
	rail_monitor[RAIL_busimon].is_enabled = 0;
	rail_monitor[RAIL_busimon].data = 0;
	rail_monitor[RAIL_busimon].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_busimon].min_voltage = 0;
	rail_monitor[RAIL_busimon].OOB_1 = 0;
	rail_monitor[RAIL_busimon].OOB_2 = 0;
	rail_monitor[RAIL_busimon].OOB_3 = 0;

	rail_monitor[RAIL_2v5].name = RAIL_2v5;
	rail_monitor[RAIL_2v5].error_count = 0;
	rail_monitor[RAIL_2v5].is_enabled = 0;
	rail_monitor[RAIL_2v5].data = 0;
	rail_monitor[RAIL_2v5].max_voltage = 3414;			// 2.75v
	rail_monitor[RAIL_2v5].min_voltage = 2792;			// 2.25v
	rail_monitor[RAIL_2v5].OOB_1 = 0;
	rail_monitor[RAIL_2v5].OOB_2 = 0;
	rail_monitor[RAIL_2v5].OOB_3 = 0;


	rail_monitor[RAIL_3v3].name = RAIL_3v3;
	rail_monitor[RAIL_3v3].error_count = 0;
	rail_monitor[RAIL_3v3].is_enabled = 0;
	rail_monitor[RAIL_3v3].data = 0;
	rail_monitor[RAIL_3v3].max_voltage = 1596;			// 1.29
	rail_monitor[RAIL_3v3].min_voltage = 1306;			// 1.05
	rail_monitor[RAIL_3v3].OOB_1 = 0;
	rail_monitor[RAIL_3v3].OOB_2 = 0;
	rail_monitor[RAIL_3v3].OOB_3 = 0;


	rail_monitor[RAIL_5v].name = RAIL_5v;
	rail_monitor[RAIL_5v].error_count = 0;
	rail_monitor[RAIL_5v].is_enabled = 0;
	rail_monitor[RAIL_5v].data = 0;
	rail_monitor[RAIL_5v].max_voltage = 4095;			// 3.30v
	rail_monitor[RAIL_5v].min_voltage = 3350;			// 2.70v
	rail_monitor[RAIL_5v].OOB_1 = 0;
	rail_monitor[RAIL_5v].OOB_2 = 0;
	rail_monitor[RAIL_5v].OOB_3 = 0;

	rail_monitor[RAIL_n3v3].name = RAIL_n3v3;
	rail_monitor[RAIL_n3v3].error_count = 0;
	rail_monitor[RAIL_n3v3].is_enabled = 0;
	rail_monitor[RAIL_n3v3].data = 0;
	rail_monitor[RAIL_n3v3].max_voltage = 4286;			// 3.45v
	rail_monitor[RAIL_n3v3].min_voltage = 3507;			// 2.83v
	rail_monitor[RAIL_n3v3].OOB_1 = 0;
	rail_monitor[RAIL_n3v3].OOB_2 = 0;
	rail_monitor[RAIL_n3v3].OOB_3 = 0;

	rail_monitor[RAIL_n5v].name = RAIL_n5v;
	rail_monitor[RAIL_n5v].error_count = 0;
	rail_monitor[RAIL_n5v].is_enabled = 0;
	rail_monitor[RAIL_n5v].data = 0;
//	rail_monitor[RAIL_n5v].max_voltage = 4150;			// 3.34v
//	rail_monitor[RAIL_n5v].min_voltage = 3395;			// 2.74v
	rail_monitor[RAIL_n5v].max_voltage = 10000;			// TODO: Erroring out at 0v, wait for Sanj
	rail_monitor[RAIL_n5v].min_voltage = 0;
	rail_monitor[RAIL_n5v].OOB_1 = 0;
	rail_monitor[RAIL_n5v].OOB_2 = 0;
	rail_monitor[RAIL_n5v].OOB_3 = 0;

	rail_monitor[RAIL_15v].name = RAIL_15v;
	rail_monitor[RAIL_15v].error_count = 0;
	rail_monitor[RAIL_15v].is_enabled = 0;
	rail_monitor[RAIL_15v].data = 0;
	rail_monitor[RAIL_15v].max_voltage = 4095;			// 3.30v
	rail_monitor[RAIL_15v].min_voltage = 3350;			// 2.70v
	rail_monitor[RAIL_15v].OOB_1 = 0;
	rail_monitor[RAIL_15v].OOB_2 = 0;
	rail_monitor[RAIL_15v].OOB_3 = 0;

	rail_monitor[RAIL_5vref].name = RAIL_5vref;
	rail_monitor[RAIL_5vref].error_count = 0;
	rail_monitor[RAIL_5vref].is_enabled = 0;
	rail_monitor[RAIL_5vref].data = 0;
	rail_monitor[RAIL_5vref].max_voltage = 4095;		// 3.30v
	rail_monitor[RAIL_5vref].min_voltage = 3350;		// 2.70v
	rail_monitor[RAIL_5vref].OOB_1 = 0;
	rail_monitor[RAIL_5vref].OOB_2 = 0;
	rail_monitor[RAIL_5vref].OOB_3 = 0;

	rail_monitor[RAIL_n200v].name = RAIL_n200v;
	rail_monitor[RAIL_n200v].error_count = 0;
	rail_monitor[RAIL_n200v].is_enabled = 0;
	rail_monitor[RAIL_n200v].data = 0;
	rail_monitor[RAIL_n200v].max_voltage = 3686;		// 2.97v
	rail_monitor[RAIL_n200v].min_voltage = 3015;		// 2.43v
	rail_monitor[RAIL_n200v].OOB_1 = 0;
	rail_monitor[RAIL_n200v].OOB_2 = 0;
	rail_monitor[RAIL_n200v].OOB_3 = 0;


	rail_monitor[RAIL_n800v].name = RAIL_n800v;
	rail_monitor[RAIL_n800v].error_count = 0;
	rail_monitor[RAIL_n800v].is_enabled = 0;
	rail_monitor[RAIL_n800v].data = 0;
	rail_monitor[RAIL_n800v].max_voltage = 3494;		// 2.82v
	rail_monitor[RAIL_n800v].min_voltage = 2859;		// 2.30v
	rail_monitor[RAIL_n800v].OOB_1 = 0;
	rail_monitor[RAIL_n800v].OOB_2 = 0;
	rail_monitor[RAIL_n800v].OOB_3 = 0;

	rail_monitor[RAIL_TMP1].name = RAIL_TMP1;
	rail_monitor[RAIL_TMP1].error_count = 0;
	rail_monitor[RAIL_TMP1].is_enabled = 0;
	rail_monitor[RAIL_TMP1].data = 0;
	rail_monitor[RAIL_TMP1].max_voltage = 2720;			// -40c NOTE: these are swapped because the conversion from int -> temp is inverse
	rail_monitor[RAIL_TMP1].min_voltage = 2023;			// 85c
	rail_monitor[RAIL_TMP1].OOB_1 = 0;
	rail_monitor[RAIL_TMP1].OOB_2 = 0;
	rail_monitor[RAIL_TMP1].OOB_3 = 0;
}


void set_rail_monitor_enable(VOLTAGE_RAIL_NAME rail_name, uint8_t enable_value) {
	rail_monitor[rail_name].is_enabled = enable_value;
}


void set_rail_monitor() {
	uint16_t hk_adc1[10];
	uint16_t hk_adc3[4];
	int16_t hk_i2c[4];

	sample_hk_i2c(hk_i2c);
	sample_hk_adc1(hk_adc1);
	sample_hk_adc3(hk_adc3);


	memcpy(&rail_monitor[RAIL_vsense].data, &hk_adc3[0], sizeof(uint16_t));
	memcpy(&rail_monitor[RAIL_vrefint].data, &hk_adc3[1], sizeof(uint16_t));
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
}

VOLTAGE_RAIL* get_rail_monitor() {
	return rail_monitor;
}

int16_t convert_ADT7410(int16_t raw) {
    float ret = raw;
    if (raw >= 0x1000) {
        ret -= 8192;
    }

    return ret / 16.0;
}

uint8_t check_bounds(VOLTAGE_RAIL_NAME name, uint16_t raw, int min, int max) {
	if (name == RAIL_TEMP1 || name == RAIL_TEMP2 || name == RAIL_TEMP3 || name == RAIL_TEMP4){
		int16_t converted_max = convert_ADT7410(max);
		int16_t converted_min = convert_ADT7410(min);
		int16_t converted_raw = convert_ADT7410(raw);
		if (converted_raw <= converted_max && converted_raw >= converted_min) {
			return 1;
		}
		return 0;
	}

	if (raw <= max && raw >= min) {
		return 1;
	}
	return 0;
}

// Returns 0 if a rail is out of bounds
uint8_t monitor_rails() {
	uint8_t within_bounds = 1;
	uint16_t tolerance;

	// Iterate through all voltage rails
	for (int i = 0; i < NUM_VOLTAGE_RAILS; i++){
		// Range check is different depending on whether the rail is enabled or not
		if (rail_monitor[i].is_enabled){
			within_bounds = check_bounds(rail_monitor[i].name, rail_monitor[i].data, rail_monitor[i].min_voltage, rail_monitor[i].max_voltage);
		}
		else {
			tolerance = rail_monitor[i].max_voltage * 0.1;
			within_bounds = check_bounds(rail_monitor[i].name, rail_monitor[i].data, 0, tolerance);
		}

		// If we aren't within range...
		if (!within_bounds) {

			// Increase that rails error count
			rail_monitor[i].error_count++;

			// Store the voltage each time a rail goes out of bounds
			switch (rail_monitor[i].error_count) {
			case 1:
				rail_monitor[i].OOB_1 = rail_monitor[i].data;
				return 0;
				break;
			case 2:
				rail_monitor[i].OOB_2 = rail_monitor[i].data;
				return 0;
				break;
			case 3:
				rail_monitor[i].OOB_3 = rail_monitor[i].data;
				ERROR_STRUCT error;
				error.detail = get_rail_name_error_detail(rail_monitor[i].name);
				error.category = EC_power_supply_rail;
				error.OOB_1 = rail_monitor[i].OOB_1;
				error.OOB_2 = rail_monitor[i].OOB_2;
				error.OOB_3 = rail_monitor[i].OOB_3;
				handle_error(error);
				break;
			default:
				break;
			}
		}
	}
	// No rails were out of bounds, so voltage_monitor task does not need to enter idle and delay
	return 1;
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

















