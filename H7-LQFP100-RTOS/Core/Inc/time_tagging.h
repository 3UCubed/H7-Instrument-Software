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
void calibrateRTC(uint8_t *buffer);
HAL_StatusTypeDef RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
#endif /* INC_TIME_TAGGING_H_ */
