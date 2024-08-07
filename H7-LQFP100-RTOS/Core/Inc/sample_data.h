/*
 * sample_data.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#ifndef INC_SAMPLE_DATA_H_
#define INC_SAMPLE_DATA_H_

#include "stm32h7xx_hal.h"	// For HAL functions
#include <stdio.h>			// For uint data types
#include "adc.h"			// For ADC handles


#define ADC1_NUM_CHANNELS 11
#define ADC3_NUM_CHANNELS 4


uint8_t init_adc_dma();

#endif /* INC_SAMPLE_DATA_H_ */
