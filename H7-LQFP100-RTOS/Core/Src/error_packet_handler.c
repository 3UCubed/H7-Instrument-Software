/*
 * error_packet_handler.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "error_packet_handler.h"


void error_protocol(VOLTAGE_RAIL_NAME rail_name) {

	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);			// ERPA + HK + VM packet off
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);			// PMT packet off

	create_hk_packet();
	create_error_packet(rail_name);
	create_junk_packet();
	NVIC_SystemReset();
}
