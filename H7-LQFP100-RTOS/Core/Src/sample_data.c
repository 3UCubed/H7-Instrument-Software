/*
 * sample_data.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "sample_data.h"

ALIGN_32BYTES(static uint16_t ADC1_raw_data[ADC1_NUM_CHANNELS]);
ALIGN_32BYTES(static uint16_t ADC3_raw_data[ADC3_NUM_CHANNELS]);

// Public Functions
uint8_t init_adc_dma() {
	uint8_t status = 0;

	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
	ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC1_raw_data,
	ADC1_NUM_CHANNELS) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY,
	ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC3_raw_data,
	ADC3_NUM_CHANNELS) != HAL_OK) {
		Error_Handler();
	}
	status = 1;

	return status;
}
