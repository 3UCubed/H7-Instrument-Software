/**
 ******************************************************************************
 * @file           : time_tagging.h
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Header file for time tagging
 ******************************************************************************
 */

#ifndef INC_TIME_TAGGING_H_
#define INC_TIME_TAGGING_H_

#include "rtc.h"
#include <stdio.h>

void get_uptime(uint8_t *buffer);
void get_unix_time(uint8_t *buffer);
void calibrateRTC(uint8_t *buffer);
#endif /* INC_TIME_TAGGING_H_ */
