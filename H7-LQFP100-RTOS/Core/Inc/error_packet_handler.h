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
	ED_vsense = 0x00,
	ED_vrefint = 0x01,
	ED_TEMP1 = 0x02,
	ED_TEMP2 = 0x03,
	ED_TEMP3 = 0x04,
	ED_TEMP4 = 0x05,
	ED_busvmon = 0x06,
	ED_busimon = 0x07,
	ED_2v5 = 0x08,
	ED_3v3 = 0x09,
	ED_5v = 0x0A,
	ED_n3v3 = 0x0B,
	ED_n5v = 0x0C,
	ED_15v = 0x0D,
	ED_5vref = 0x0E,
	ED_n200v = 0x0F,
	ED_n800v = 0x10,
	ED_TMP1 = 0x11,

	ED_single_bit_error = 0x12,
	ED_double_bit_error = 0x13,
	ED_peripheral_error = 0x14,

	ED_UNKNOWN = 0xFF
}ERROR_DETAIL;

void handle_error(ERROR_DETAIL error_detail);
void send_error_packet(ERROR_DETAIL error_detail);
void send_junk_packet();

#endif /* INC_ERROR_PACKET_HANDLER_H_ */
