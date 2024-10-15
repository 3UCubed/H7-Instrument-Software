/**
 ******************************************************************************
 * @file           : error_packet_handler.h
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Header file for error packet handling
 ******************************************************************************
 */

#ifndef INC_ERROR_PACKET_HANDLER_H_
#define INC_ERROR_PACKET_HANDLER_H_

#include <stdio.h>

typedef enum
{
	EC_power_supply_rail = 0x00,
	EC_seu = 0x01,
	EC_peripheral = 0x02,
	EC_brownout = 0x03,
	EC_watchdog = 0x04,
	EC_UNDEFINED = 0x05
}ERROR_CATEGORY;

typedef enum
{
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
	ED_single_bit_error_flash = 0x18,
	ED_double_bit_error_flash = 0x19,
	ED_single_bit_error_ram = 0x1A,
	ED_double_bit_error_ram = 0x1B,
	ED_UNDEFINED = 0x1C
}ERROR_DETAIL;

typedef struct
{
	ERROR_CATEGORY category;
	ERROR_DETAIL detail;
	uint16_t OOB_1;
	uint16_t OOB_2;
	uint16_t OOB_3;

}ERROR_STRUCT;

void error_counter_init();
void handle_error(ERROR_STRUCT error);
void send_error_counter_packet();
void send_previous_error_packet();
void reset_error_counters();

#endif /* INC_ERROR_PACKET_HANDLER_H_ */
