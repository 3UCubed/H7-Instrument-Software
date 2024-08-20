/*
 * error_packet_handler.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_ERROR_PACKET_HANDLER_H_
#define INC_ERROR_PACKET_HANDLER_H_

#define ERROR_PACKET_SIZE 3
#define JUNK_PACKET_SIZE 256
#define ERROR_PACKET_SYNC 0xDD

typedef enum {
	EC_vsense = 0x00,
	EC_vrefint = 0x01,
	EC_TEMP1 = 0x02,
	EC_TEMP2 = 0x03,
	EC_TEMP3 = 0x04,
	EC_TEMP4 = 0x05,
	EC_busvmon = 0x06,
	EC_busimon = 0x07,
	EC_2v5 = 0x08,
	EC_3v3 = 0x09,
	EC_5v = 0x0A,
	EC_n3v3 = 0x0B,
	EC_n5v = 0x0C,
	EC_15v = 0x0D,
	EC_5vref = 0x0E,
	EC_n200v = 0x0F,
	EC_n800v = 0x10,
	EC_TMP1 = 0x11,

	EC_single_bit_error = 0x12,
	EC_double_bit_error = 0x13,
	EC_peripheral_error = 0x14,

	EC_UNKNOWN = 0xFF
}ERROR_CODE;

void handle_error(ERROR_CODE error_code);
void send_error_packet(ERROR_CODE error_code);
void send_junk_packet();

#endif /* INC_ERROR_PACKET_HANDLER_H_ */
