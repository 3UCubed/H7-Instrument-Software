/*
 * time_tagging.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_TIME_TAGGING_H_
#define INC_TIME_TAGGING_H_

#include "main.h"
#include "rtc.h"	// For RTC handle

void get_uptime(uint8_t *buffer);
void get_timestamp(uint8_t *buffer);

#endif /* INC_TIME_TAGGING_H_ */
