/*
 * sample_data.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_SAMPLE_DATA_H_
#define INC_SAMPLE_DATA_H_

#include "main.h"
#include <stdio.h>			// For uint data types
#include "adc.h"			// For ADC handles
#include "spi.h"			// For SPI handles


#define ADC1_NUM_CHANNELS 11
#define ADC3_NUM_CHANNELS 4


uint8_t init_adc_dma();
void sample_pmt_spi(uint8_t *buffer);

#endif /* INC_SAMPLE_DATA_H_ */
