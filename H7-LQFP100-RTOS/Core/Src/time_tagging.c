/*
 * time.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "time_tagging.h"

void get_uptime(uint8_t *buffer) {
	uint32_t uptime = 0;
	uint32_t ms = uptime_millis;
	uint32_t st = SysTick->VAL;

	// Did uptime_millis rollover while reading SysTick->VAL?
	if (ms != uptime_millis) {
		ms = uptime_millis;
		st = SysTick->VAL;
	}
	uptime = ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);

	buffer[0] = ((uptime >> 24) & 0xFF);
	buffer[1] = ((uptime >> 16) & 0xFF);
	buffer[2] = ((uptime >> 8) & 0xFF);
	buffer[3] = uptime & 0xFF;
}
