/*
 * error_packet_handler.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_ERROR_PACKET_HANDLER_H_
#define INC_ERROR_PACKET_HANDLER_H_

#include <stdio.h>				// For uint data types


#define ERROR_PACKET_SIZE 4
#define JUNK_PACKET_SIZE 1024
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
	ED_UNDEFINED = 0x14
}ERROR_DETAIL;

typedef enum {
	EC_power_supply_rail = 0x00,
	EC_seu = 0x01,
	EC_peripheral = 0x02,
	EC_brownout = 0x03,
	EC_watchdog = 0x04,
	EC_UNDEFINED = 0x05
}ERROR_CATEGORY;

typedef struct {
	ERROR_CATEGORY category;
	ERROR_DETAIL detail;
}ERROR_STRUCT;

void handle_error(ERROR_STRUCT error);
void send_error_packet(ERROR_STRUCT error);
void send_junk_packet();
void increment_error_counter(ERROR_CATEGORY category);
uint16_t get_eeprom_error_counter(ERROR_CATEGORY category);
void set_eeprom_error_counter(ERROR_CATEGORY category, uint16_t new_counter_value);
void reset_eeprom_error_counters();
void error_counter_init();

#endif /* INC_ERROR_PACKET_HANDLER_H_ */
