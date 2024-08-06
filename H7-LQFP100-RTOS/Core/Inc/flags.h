/*
 * flags.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_FLAGS_H_
#define INC_FLAGS_H_

#include "cmsis_os.h"

#define PMT_FLAG_ID 0x0001
#define ERPA_FLAG_ID 0x0002
#define HK_FLAG_ID 0x0004
#define VOLTAGE_MONITOR_FLAG_ID 0x0008
#define STOP_FLAG 0x0016

extern osEventFlagsId_t event_flags;

#endif /* INC_FLAGS_H_ */
