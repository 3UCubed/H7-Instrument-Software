/**
 ******************************************************************************
 * @file           : sample_data.c
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Implementation of data sampling
 ******************************************************************************
 */

#include "sample_data.h"

static const uint8_t REG_TEMP = 0x00;
static const uint8_t ADT7410_1 = 0x48 << 1;
static const uint8_t ADT7410_2 = 0x4A << 1;
static const uint8_t ADT7410_3 = 0x49 << 1;
static const uint8_t ADT7410_4 = 0x4B << 1;

ALIGN_32BYTES(static uint16_t ADC1_raw_data[ADC1_NUM_CHANNELS]);
ALIGN_32BYTES(static uint16_t ADC3_raw_data[ADC3_NUM_CHANNELS]);
static uint16_t erpa_spi_raw_data[1];
static uint16_t pmt_spi_raw_data[1];
static uint8_t raw_i2c[2];

/**
 * @brief Initializes ADCs with DMA.
 *
 * This function calibrates and starts DMA for ADC1 and ADC3.
 * It also enables the SPI peripherals.
 */
void init_adc_dma()
{
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC1_raw_data, ADC1_NUM_CHANNELS) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC3_raw_data, ADC3_NUM_CHANNELS) != HAL_OK)
	{
		Error_Handler();
	}

	hspi2.Instance->CR1 |= 1 << 10;
	hspi1.Instance->CR1 |= 1 << 10;
}

/**
 * @brief Samples PMT data via SPI.
 *
 * @param buffer Pointer to store the sampled data.
 */
void sample_pmt_spi(uint8_t *buffer)
{
	uint8_t spi_MSB;
	uint8_t spi_LSB;

	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*) pmt_spi_raw_data, 1);

	spi_LSB = ((pmt_spi_raw_data[0] & 0xFF00) >> 8);
	spi_MSB = (pmt_spi_raw_data[0] & 0xFF);

	buffer[0] = spi_LSB;
	buffer[1] = spi_MSB;
}

/**
 * @brief Samples ERPA data via SPI.
 *
 * @param buffer Pointer to store the sampled data.
 */
void sample_erpa_spi(uint8_t *buffer)
{
	uint8_t spi_MSB;
	uint8_t spi_LSB;
	HAL_SPI_Receive_DMA(&hspi2, (uint8_t*) erpa_spi_raw_data, 1);

	spi_LSB = ((erpa_spi_raw_data[0] & 0xFF00) >> 8);
	spi_MSB = (erpa_spi_raw_data[0] & 0xFF);

	buffer[0] = spi_LSB;
	buffer[1] = spi_MSB;
}

/**
 * @brief Samples ERPA data from ADC.
 *
 * @param buffer Pointer to store the sampled data.
 */
void sample_erpa_adc(uint16_t *buffer)
{
	uint16_t PC4 = ADC1_raw_data[1];

	buffer[0] = PC4;
}

/**
 * @brief Samples HK data from I2C sensors.
 *
 * @param buffer Pointer to store the sampled data from sensors.
 */
void sample_hk_i2c(int16_t *buffer)
{
	int16_t output1 = poll_i2c_sensor(ADT7410_1);
	int16_t output2 = poll_i2c_sensor(ADT7410_2);
	int16_t output3 = poll_i2c_sensor(ADT7410_3);
	int16_t output4 = poll_i2c_sensor(ADT7410_4);

	buffer[0] = output1;
	buffer[1] = output2;
	buffer[2] = output3;
	buffer[3] = output4;
}

/**
 * @brief Samples HK data from ADC1 channels.
 *
 * @param buffer Pointer to store the sampled ADC values.
 */
void sample_hk_adc1(uint16_t *buffer)
{
	uint16_t PA1 = ADC1_raw_data[10];
	uint16_t PA2 = ADC1_raw_data[8];
	uint16_t PC0 = ADC1_raw_data[6];
	uint16_t PA3 = ADC1_raw_data[9];
	uint16_t PB1 = ADC1_raw_data[2];
	uint16_t PA7 = ADC1_raw_data[3];
	uint16_t PC1 = ADC1_raw_data[7];
	uint16_t PC5 = ADC1_raw_data[4];
	uint16_t PA6 = ADC1_raw_data[0];
	uint16_t PB0 = ADC1_raw_data[5];

	buffer[0] = PA1;
	buffer[1] = PA2;
	buffer[2] = PC0;
	buffer[3] = PA3;
	buffer[4] = PB1;
	buffer[5] = PA7;
	buffer[6] = PC1;
	buffer[7] = PC5;
	buffer[8] = PA6;
	buffer[9] = PB0;
}

/**
 * @brief Samples HK data from ADC3 channels.
 *
 * @param buffer Pointer to store the sampled ADC values.
 */
void sample_hk_adc3(uint16_t *buffer)
{
	uint16_t vsense = ADC3_raw_data[0];
	uint16_t vrefint = ADC3_raw_data[1];
	uint16_t PC2 = ADC3_raw_data[2];

	buffer[0] = vsense;
	buffer[1] = vrefint;
	buffer[2] = PC2;
}

/**
 * @brief Polls temperature data from an I2C sensor.
 *
 * @param TEMP_ADDR I2C address of the temperature sensor.
 * @return int16_t The temperature data received from the sensor.
 */
int16_t poll_i2c_sensor(const uint8_t TEMP_ADDR)
{
	int16_t output;
	HAL_StatusTypeDef ret;
	raw_i2c[0] = REG_TEMP;

	ret = HAL_I2C_Master_Transmit_DMA(&hi2c1, TEMP_ADDR, (uint8_t*) raw_i2c, 1);
	if (ret != HAL_OK)
	{
		Error_Handler();
	}
	else
	{
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {};
		ret = HAL_I2C_Master_Receive_DMA(&hi2c1, TEMP_ADDR, (uint8_t*) raw_i2c, 2);
		if (ret != HAL_OK)
		{
			Error_Handler();
		}
		else
		{
			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {};
			output = (int16_t) (raw_i2c[0] << 8);
			output = (output | raw_i2c[1]) >> 3;
		}
	}
	return output;
}

