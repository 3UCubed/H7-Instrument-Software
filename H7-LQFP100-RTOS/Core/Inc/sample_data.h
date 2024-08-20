/*
 * sample_data.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_SAMPLE_DATA_H_
#define INC_SAMPLE_DATA_H_


#include <stdio.h>				// For uint data types
#include <adc.h>
#include "spi.h"
#include "i2c.h"
#define ADC1_NUM_CHANNELS 11
#define ADC3_NUM_CHANNELS 4


uint8_t init_adc_dma();
void sample_pmt_spi(uint8_t *buffer);
void sample_erpa_spi(uint8_t *buffer);
void sample_erpa_adc(uint16_t *buffer);
void sample_hk_i2c(int16_t *buffer);
void sample_hk_adc1(uint16_t *buffer);
void sample_hk_adc3(uint16_t *buffer);
int16_t poll_i2c_sensor(const uint8_t TEMP_ADDR);

#endif /* INC_SAMPLE_DATA_H_ */
