/*
 * error_packet_handler.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_ERROR_PACKET_HANDLER_H_
#define INC_ERROR_PACKET_HANDLER_H_

#include "voltage_monitor.h"
#include "packet_creation.h"

void error_protocol(VOLTAGE_RAIL_NAME failed_rail);

#endif /* INC_ERROR_PACKET_HANDLER_H_ */
