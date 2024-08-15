/*
 * shared_types.h
 *
 *  Created on: Aug 15, 2024
 *      Author: 3ucubed
 */

#ifndef INC_SHARED_TYPES_H_
#define INC_SHARED_TYPES_H_

typedef enum {
	RAIL_vsense,	// 0
	RAIL_vrefint,	// 1
	RAIL_TEMP1,		// 2
	RAIL_TEMP2,		// 3
	RAIL_TEMP3,		// 4
	RAIL_TEMP4,		// 5
	RAIL_busvmon,	// 6
	RAIL_busimon,	// 7
	RAIL_2v5,		// 8
	RAIL_3v3,		// 9
	RAIL_5v,		// 10
	RAIL_n3v3,		// 11
	RAIL_n5v,		// 12
	RAIL_15v,		// 13
	RAIL_5vref,		// 14
	RAIL_n200v,		// 15
	RAIL_n800v,		// 16
	RAIL_TMP1		// 17
} VOLTAGE_RAIL_NAME;

#endif /* INC_SHARED_TYPES_H_ */
