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

void sample_pmt_spi(uint8_t *buffer) {
	uint8_t spi_raw_data[2];
	uint8_t spi_MSB;
	uint8_t spi_LSB;

	HAL_SPI_Receive(&hspi1, (uint8_t*) spi_raw_data, 1, 1);

	spi_LSB = ((spi_raw_data[0] & 0xFF00) >> 8);
	spi_MSB = (spi_raw_data[1] & 0xFF);

	hspi1.Instance->CR1 |= 1 << 10;

	buffer[0] = spi_MSB;
	buffer[1] = spi_LSB;
}

void sample_erpa_spi(uint8_t *buffer) {
	uint8_t spi_raw_data[2];
	uint8_t spi_MSB;
	uint8_t spi_LSB;

	HAL_SPI_Receive(&hspi2, (uint8_t*) spi_raw_data, 1, 100);

	spi_LSB = ((spi_raw_data[0] & 0xFF00) >> 8);
	spi_MSB = (spi_raw_data[1] & 0xFF);

	hspi2.Instance->CR1 |= 1 << 10;

	buffer[0] = spi_MSB;
	buffer[1] = spi_LSB;
}

void sample_erpa_adc(uint16_t *buffer) {
	uint16_t PC4 = ADC1_raw_data[1];

	buffer[0] = PC4;
}

