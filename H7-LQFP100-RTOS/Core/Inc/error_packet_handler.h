/**
/**
 * @file error_packet_handler.h
 * @brief Contains all function prototypes for error_packet_handler.c
 *
 * ERROR_STRUCT is made up of ERROR_CATEGORY and ERROR_DETAIL. An error category has details associated with it.
 * For instance, if there is an error on a power supply rail, the error category will be EC_power_supply_rail with the
 * detail being whatever specific rail caused the error. However, some categories do not have associated details. For example,
 * EC_watchdog does not have any details associated with it, and the error packet that is sent for a watchdog error will have
 * the detail ED_UNDEFINED.
 *
 * @author Jared Morrison
 * @date September 4, 2024
 */

#ifndef INC_ERROR_PACKET_HANDLER_H_
#define INC_ERROR_PACKET_HANDLER_H_

#include <stdio.h>

#define ERROR_COUNTER_PACKET_SIZE 56
#define PREV_ERROR_PACKET_SIZE 4
#define CURRENT_ERROR_PACKET_SIZE 4
#define JUNK_PACKET_SIZE 1024

#define ERROR_COUNTER_PACKET_SYNC 0xCC
#define PREV_ERROR_PACKET_SYNC 0xAA
#define CURRENT_ERROR_PACKET_SYNC 0xBB

#define NUM_ERROR_COUNTERS 27

#define PREV_ERROR_CATEGORY_INDEX 27
#define PREV_ERROR_DETAIL_INDEX 28

typedef enum {
	EC_power_supply_rail = 0x00,
	EC_seu = 0x01,
	EC_peripheral = 0x02,
	EC_brownout = 0x03,
	EC_watchdog = 0x04,
	EC_UNDEFINED = 0x05
}ERROR_CATEGORY;

typedef enum {
	ED_vsense = 0x06,
	ED_vrefint = 0x07,
	ED_TEMP1 = 0x08,
	ED_TEMP2 = 0x09,
	ED_TEMP3 = 0x0A,
	ED_TEMP4 = 0x0B,
	ED_busvmon = 0x0C,
	ED_busimon = 0x0D,
	ED_2v5 = 0x0E,
	ED_3v3 = 0x0F,
	ED_5v = 0x10,
	ED_n3v3 = 0x11,
	ED_n5v = 0x12,
	ED_15v = 0x13,
	ED_5vref = 0x14,
	ED_n200v = 0x15,
	ED_n800v = 0x16,
	ED_TMP1 = 0x17,
	ED_single_bit_error = 0x18,
	ED_double_bit_error = 0x19,
	ED_UNDEFINED = 0x1A
}ERROR_DETAIL;

typedef struct {
	ERROR_CATEGORY category;
	ERROR_DETAIL detail;
}ERROR_STRUCT;

void error_counter_init();
void increment_error_counter(ERROR_STRUCT error);
void update_error_counter();
void reset_error_counters();
void reset_previous_error();
void set_previous_error(ERROR_STRUCT error);
ERROR_STRUCT get_previous_error();
void handle_error(ERROR_STRUCT error);
void send_error_counter_packet();
void send_previous_error_packet();
void send_current_error_packet(ERROR_STRUCT error);
void send_junk_packet();


#endif /* INC_ERROR_PACKET_HANDLER_H_ */
