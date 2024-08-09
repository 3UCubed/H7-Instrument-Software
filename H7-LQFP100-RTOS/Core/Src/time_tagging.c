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

void get_timestamp(uint8_t *buffer) {
	RTC_TimeTypeDef current_time;
	RTC_DateTypeDef current_date;

	HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &current_date, RTC_FORMAT_BIN);
	uint32_t milliseconds = 1000000 - (current_time.SubSeconds * 100);

	buffer[0] = current_date.Year;				// 0-99
	buffer[1] = current_date.Month;				// 1-12
	buffer[2] = current_date.Date;				// 1-31
	buffer[3] = current_time.Hours;				// 0-23
	buffer[4] = current_time.Minutes;			// 0-59
	buffer[5] = current_time.Seconds;			// 0-59
	buffer[6] = ((milliseconds >> 24) & 0xFF);
	buffer[7] = ((milliseconds >> 16) & 0xFF);
	buffer[8] = ((milliseconds >> 8) & 0xFF);
	buffer[9] = milliseconds & 0xFF;
}
