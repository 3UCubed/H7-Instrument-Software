/*
 * sample_data.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_SAMPLE_DATA_H_
#define INC_SAMPLE_DATA_H_

#include "i2c.h"
#include "spi.h"
#include "adc.h"
#include "dma.h"

#define ADC1_NUM_CHANNELS 11
#define ADC3_NUM_CHANNELS 4

ALIGN_32BYTES(static uint16_t ADC1_raw_data[ADC1_NUM_CHANNELS]);
ALIGN_32BYTES(static uint16_t ADC3_raw_data[ADC3_NUM_CHANNELS]);

static const uint8_t REG_TEMP = 0x00;
static const uint8_t ADT7410_1 = 0x48 << 1;
static const uint8_t ADT7410_2 = 0x4A << 1;
static const uint8_t ADT7410_3 = 0x49 << 1;
static const uint8_t ADT7410_4 = 0x4B << 1;

uint8_t init_adc_dma();
int16_t poll_i2c_sensor(const uint8_t TEMP_ADDR);
void receive_pmt_spi(uint8_t *buffer);
void receive_erpa_spi(uint8_t *buffer);
void receive_erpa_adc(uint16_t *buffer);
void receive_hk_i2c(int16_t *buffer) ;
void receive_hk_adc1(uint16_t *buffer);
void receive_hk_adc3(uint16_t *buffer);

#endif /* INC_SAMPLE_DATA_H_ */
