/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#define SIZE 16
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	GPIO_TypeDef *gpio;
	uint16_t pin;
} gpio_pins;
const int HK_CADENCE = 1; //Should be set at 5
const gpio_pins gpios[] = { { GPIOB, GPIO_PIN_5 }, { GPIOB, GPIO_PIN_6 }, {
		GPIOC, GPIO_PIN_10 }, { GPIOC, GPIO_PIN_13 }, { GPIOC, GPIO_PIN_7 }, {
		GPIOC, GPIO_PIN_8 }, { GPIOC, GPIO_PIN_9 }, { GPIOC, GPIO_PIN_6 }, {
		GPIOB, GPIO_PIN_2 } };
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC1NumChannels 11
#define ADC3NumChannels 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define BUFFER_SIZE 100
unsigned char rx_buf[BUFFER_SIZE];

/* Hexadecimal Addresses for I2C Temperature Sensors */
static const uint8_t ADT7410_1 = 0x48 << 1;
static const uint8_t ADT7410_2 = 0x4A << 1;
static const uint8_t ADT7410_3 = 0x49 << 1;
static const uint8_t ADT7410_4 = 0x4B << 1;

/* Internal ADC DMA variables */
/**
 * CHANGED THIS TO SIZE 15. As of 12/28/23 I have ADC 1 and ADC 3 enabled
 * ADC 1 has 15 channels and ADC 3 has 4. I was trying to isolate ADC 1
 * and just get that working but had no luck. This youtube video might help:
 * https://www.youtube.com/watch?v=AloHXBk6Bfk&t=255s
 */
ALIGN_32BYTES(static uint16_t ADC1Data[ADC1NumChannels]);
ALIGN_32BYTES(static uint16_t ADC3Data[ADC3NumChannels]);

/* DAC Variables for SWP */
uint32_t DAC_OUT[SIZE] = { 0, 0, 620, 1241, 1861, 2482, 3103, 3723, 4095, 4095, 3723, 3103, 2482, 1861, 1241, 620 }; // For 3.3 volts
uint8_t step = 0;
int is_increasing = 1;
int auto_sweep = 0;

/* ERPA PACKET FACRTORING */
int SAMPLING_FACTOR = 1;
int FACTOR_COUNTER = 0;
int SWP_FACTOR_COUNTER = 0;

/* UART Variables */
const uint8_t erpa_sync = 0xAA; // SYNC byte to let packet interpreter / OBC know which packet is which
uint16_t erpa_seq = 0; // SEQ byte which keeps track of what # ERPA packet is being sent (0-65535)
const uint8_t pmt_sync = 0xBB; // SYNC byte to let packet interpreter / OBC know which packet is which
uint16_t pmt_seq = 0; // SEQ byte which keeps track of what # ERPA packet is being sent (0-65535)
const uint8_t hk_sync = 0xCC; // SYNC byte to let packet interpreter / OBC know which packet is which
uint16_t hk_seq = 0; // SEQ byte which keeps track of what # HK packet is being sent (0-65535)

int hk_counter = 0; // counter to know when to send HK packet (sent every 50 ERPA packets)
// we put them in the same routine and send HK when this count == 50
int startupTimer = 0;
uint8_t PMT_ON = 0;
uint8_t ERPA_ON = 0;
uint8_t HK_ON = 0;

static const uint8_t REG_TEMP = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief function to poll individual i2c sensor
 *
 * @param TEMP_ADDR address of individual i2c sensor
 * @return int16_t integer result from sensor
 */
int16_t poll_i2c_sensor(const uint8_t TEMP_ADDR) {
	int16_t output;
	uint8_t buf[2];
	HAL_StatusTypeDef ret;
	buf[0] = REG_TEMP;
	ret = HAL_I2C_Master_Transmit(&hi2c1, TEMP_ADDR, buf, 1, 1000);
	if (ret != HAL_OK) {
		strcpy((char*) buf, "Error Tx\r\n");
	} else {
		/* Read 2 bytes from the temperature register */
		ret = HAL_I2C_Master_Receive(&hi2c1, TEMP_ADDR, buf, 2, 1000);
		if (ret != HAL_OK) {
			strcpy((char*) buf, "Error Rx\r\n");
		} else {
			output = (int16_t) (buf[0] << 8);
			output = (output | buf[1]) >> 3;
		}
	}
	return output;
}

/**
 * @brief called in hk routine to poll each i2c sensor
 *
 * @return int16_t* size 4 buffer to return i2c values
 */
int16_t* i2c() {
	int16_t output1 = poll_i2c_sensor(ADT7410_1);
	int16_t output2 = poll_i2c_sensor(ADT7410_2);
	int16_t output3 = poll_i2c_sensor(ADT7410_3);
	int16_t output4 = poll_i2c_sensor(ADT7410_4);

	int16_t *results = malloc(4 * sizeof(int16_t));
	results[0] = output1;
	results[1] = output2;
	results[2] = output3;
	results[3] = output4;
	return results;
}

/**
 * @brief if auto sweep is enabled, will increase step
 *
 */
void do_auto_sweep() {
	if (step == 9) {
		is_increasing = 0;
		step--;
	} else if (step == 0) {
		is_increasing = 1;
		step++;
	}

	if (is_increasing) {
		step++;
	} else {
		step--;
	}

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_OUT[step]);

}

/**
 * @brief polls respective spi sensor and returns uint16_t value
 *
 * @param spi given spi peripheral
 * @return uint16_t raw value from reading spi peripheral
 */
uint8_t* spi(SPI_HandleTypeDef spi) {
	uint8_t spiRxBuffer[2];
	HAL_SPI_Receive(&spi, (uint8_t*) spiRxBuffer, 1, 1);
	uint8_t SPI_LSB = ((spiRxBuffer[0] & 0xFF00) >> 8);
	uint8_t SPI_MSB = (spiRxBuffer[1] & 0xFF);
	spi.Instance->CR1 |= 1 << 10; // THIS IS NEEDED TO STOP SPI2_SCK FROM GENERATING CLOCK PULSES
	uint8_t *results = malloc(2 * sizeof(uint8_t));
	results[0] = SPI_MSB;
	results[1] = SPI_LSB;
	return results;
}

/**
 * @brief Set the erpa sweep value
 *
 */
void set_erpa_sweep() {
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_OUT[step]);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

}

/**
 * @brief polls erpa adcs
 *
 * @return uint16_t* results of erpa adcs
 */
uint16_t* erpa_adc() {

	uint16_t PC4 = ADC1Data[1]; 			// SWPmon --
	uint16_t PB0 = ADC1Data[5]; 	// TEMP1 -- verified doesn't need to change

	uint16_t *results = malloc(2 * sizeof(uint16_t));
	results[0] = PC4;
	results[1] = PB0;

	return results;

}

uint16_t* hk_adc1() {

	uint16_t PA1 = ADC1Data[10];// BUSVmon -- verified doesn't need to change
	uint16_t PA2 = ADC1Data[8];	// BUSImon -- verified doesn't need to change
	uint16_t PC0 = ADC1Data[6];		// 2v5mon -- verified doesn't need to change
	uint16_t PA3 = ADC1Data[9];				// n3v3mon --
	uint16_t PB1 = ADC1Data[2];		// n200v -- verified doesn't need to change
	uint16_t PA7 = ADC1Data[3];				// n800v --
	uint16_t PC1 = ADC1Data[7];				// 5vmon --
	uint16_t PC5 = ADC1Data[4];		// 15vmon -- verified doesn't need to change
	uint16_t PA6 = ADC1Data[0];				// 5vrefmon --

	uint16_t *results = malloc(9 * sizeof(uint16_t));
	results[0] = PA1;
	results[1] = PA2;
	results[2] = PC0;
	results[3] = PA3;
	results[4] = PB1;
	results[5] = PA7;
	results[6] = PC1;
	results[7] = PC5;
	results[8] = PA6;

	return results;

}

uint16_t* hk_adc3() {

	uint16_t vrefint = ADC3Data[0];
	uint16_t vsense = ADC3Data[1];
	uint16_t PC2 = ADC3Data[2]; 		// n5vmon --
	uint16_t PC3 = ADC3Data[3];			// 3v3mon --

	uint16_t *results = malloc(4 * sizeof(uint16_t));
	results[0] = vrefint;
	results[1] = vsense;
	results[2] = PC2;
	results[3] = PC3;

	return results;
}

/**
 * @brief sends erpa packet via UART
 *
 * @param erpa_spi erpa spi raw value
 * @param erpa_adc_results erpa adc results
 */
void send_erpa_packet(uint8_t *erpa_spi, uint16_t *erpa_adc_results) {

	uint8_t erpa_buf[14];
	erpa_buf[0] = erpa_sync;                  			// ERPA SYNC 0xAA MSB
	erpa_buf[1] = erpa_sync;                  			// ERPA SYNC 0xAA LSB
	erpa_buf[2] = ((erpa_seq & 0xFF00) >> 8); 				// ERPA SEQ # MSB
	erpa_buf[3] = (erpa_seq & 0xFF);          				// ERPA SEQ # MSB
	erpa_buf[4] = ((0 & 0xFF00) >> 8); 	  						// ENDmon MSB
	erpa_buf[5] = (0 & 0xFF);               					// ENDmon LSB
	erpa_buf[6] = ((erpa_adc_results[0] & 0xFF00) >> 8);    // SWP Monitored MSB
	erpa_buf[7] = (erpa_adc_results[0] & 0xFF);             // SWP Monitored LSB
	erpa_buf[8] = ((erpa_adc_results[1] & 0xFF00) >> 8);    // TEMPURATURE 1 MSB
	erpa_buf[9] = (erpa_adc_results[1] & 0xFF);             // TEMPURATURE 1 LSB
	erpa_buf[10] = ((0 & 0xFF00) >> 8);     				// TEMPURATURE 2 MSB
	erpa_buf[11] = (0 & 0xFF);                    			// TEMPURATURE 2 LSB
	erpa_buf[12] = erpa_spi[0];									// ERPA eADC MSB
	erpa_buf[13] = erpa_spi[1];									// ERPA eADC LSB

	HAL_UART_Transmit(&huart1, erpa_buf, sizeof(erpa_buf), 100);
	erpa_seq++;

}

/**
 * @brief sends hk packet via UART
 *
 * @param i2c_values values of all i2c sensors
 * @param hk_adc_results hk adc results
 */
void send_hk_packet(int16_t *i2c_values, uint16_t *hk_adc1_results,
		uint16_t *hk_adc3_results) {

	uint8_t hk_buf[38];									// GUI Structure

	hk_buf[0] = hk_sync;                     			// HK SYNC 0xCC MSB
	hk_buf[1] = hk_sync;                     			// HK SYNC 0xCC LSB
	hk_buf[2] = ((hk_seq & 0xFF00) >> 8);    			// HK SEQ # MSB
	hk_buf[3] = (hk_seq & 0xFF);             			// HK SEQ # LSB
	hk_buf[4] = ((hk_adc3_results[1] & 0xFF00) >> 8);	// HK vsense MSB
	hk_buf[5] = (hk_adc3_results[1] & 0xFF);			// HK vsense LSB
	hk_buf[6] = ((hk_adc3_results[0] & 0xFF00) >> 8);	// HK vrefint MSB
	hk_buf[7] = (hk_adc3_results[0] & 0xFF);			// HK vrefint LSB
	hk_buf[8] = ((i2c_values[0] & 0xFF00) >> 8);		// HK TEMP1 MSB
	hk_buf[9] = (i2c_values[0] & 0xFF);					// HK TEMP1 LSB
	hk_buf[10] = ((i2c_values[1] & 0xFF00) >> 8);		// HK TEMP2 MSB
	hk_buf[11] = (i2c_values[1] & 0xFF);				// HK TEMP2 LSB
	hk_buf[12] = ((i2c_values[2] & 0xFF00) >> 8);		// HK TEMP3 MSB
	hk_buf[13] = (i2c_values[2] & 0xFF);				// HK TEMP3 LSB
	hk_buf[14] = ((i2c_values[3] & 0xFF00) >> 8);		// HK TEMP4 MSB
	hk_buf[15] = (i2c_values[3] & 0xFF);				// HK TEMP4 LSB
	hk_buf[16] = ((hk_adc1_results[0] & 0xFF00) >> 8);	// HK BUSvmon MSB
	hk_buf[17] = (hk_adc1_results[0] & 0xFF);			// HK BUSvmon LSB
	hk_buf[18] = ((hk_adc1_results[1] & 0xFF00) >> 8);	// HK BUSimon MSB
	hk_buf[19] = (hk_adc1_results[1] & 0xFF);			// HK BUSimon LSB
	hk_buf[20] = ((hk_adc1_results[2] & 0xFF00) >> 8);	// HK 2v5mon MSB
	hk_buf[21] = (hk_adc1_results[2] & 0xFF);			// HK 2v5mon LSB
	hk_buf[22] = ((hk_adc3_results[3] & 0xFF00) >> 8);	// HK 3v3mon MSB
	hk_buf[23] = (hk_adc3_results[3] & 0xFF);			// HK 3v3mon LSB
	hk_buf[24] = ((hk_adc1_results[6] & 0xFF00) >> 8);	// HK 5vmon MSB
	hk_buf[25] = (hk_adc1_results[6] & 0xFF);			// HK 5vmon LSB
	hk_buf[26] = ((hk_adc1_results[3] & 0xFF00) >> 8);	// HK n3v3mon MSB
	hk_buf[27] = (hk_adc1_results[3] & 0xFF);			// HK n3v3mon LSB
	hk_buf[28] = ((hk_adc3_results[2] & 0xFF00) >> 8);	// HK n5vmon MSB
	hk_buf[29] = (hk_adc3_results[2] & 0xFF);			// HK n5vmon LSB
	hk_buf[30] = ((hk_adc1_results[7] & 0xFF00) >> 8);	// HK 15vmon MSB
	hk_buf[31] = (hk_adc1_results[7] & 0xFF);			// HK 15vmon LSB
	hk_buf[32] = ((hk_adc1_results[8] & 0xFF00) >> 8);	// HK 5vrefmon MSB
	hk_buf[33] = (hk_adc1_results[8] & 0xFF);			// HK 5vrefmon LSB
	hk_buf[34] = ((hk_adc1_results[4] & 0xFF00) >> 8);	// HK n150vmon MSB
	hk_buf[35] = (hk_adc1_results[4] & 0xFF);			// HK n150vmon LSB
	hk_buf[36] = ((hk_adc1_results[5] & 0xFF00) >> 8);	// HK n800vmon MSB
	hk_buf[37] = (hk_adc1_results[5] & 0xFF);			// HK n800vmon LSB

	HAL_UART_Transmit(&huart1, hk_buf, sizeof(hk_buf), 100);
	hk_seq++;

}

/**
 * @brief sends pmt packet via UART
 *
 * @param pmt_spi raw pmt spi value
 */
void send_pmt_packet(uint8_t *pmt_spi) {

	uint8_t pmt_buf[6];

	pmt_buf[0] = pmt_sync;						// PMT SYNC MSB 0xBB
	pmt_buf[1] = pmt_sync;						// PMT SYNC LSB 0xBB
	pmt_buf[2] = ((pmt_seq & 0xFF00) >> 8);		// PMT SEQ # MSB
	pmt_buf[3] = (pmt_seq & 0xFF);				// PMT SEQ # LSB
	pmt_buf[4] = pmt_spi[0];					// PMT eADC MSB
	pmt_buf[5] = pmt_spi[1];					// PMT eADC LSB

	HAL_UART_Transmit(&huart1, pmt_buf, sizeof(pmt_buf), 100);
	pmt_seq++;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		if (ERPA_ON) {
			SWP_FACTOR_COUNTER++;

			while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) {
			}

			uint8_t *spi2_results = spi(hspi2);
			uint16_t *erpa_adc_results = erpa_adc();

//			if (SWP_FACTOR_COUNTER == (SAMPLING_FACTOR * 2)) {
//				if (auto_sweep) {
//					do_auto_sweep();
//				} else {
//					set_erpa_sweep();
//				}
//				SWP_FACTOR_COUNTER = 0;
//			}

			send_erpa_packet(spi2_results, erpa_adc_results);

			free(spi2_results);
			free(erpa_adc_results);
		}
		if (HK_ON) {
			int16_t *i2c_values = i2c();
			uint16_t *hk_adc1_results = hk_adc1();
			uint16_t *hk_adc3_results = hk_adc3();

			send_hk_packet(i2c_values, hk_adc1_results, hk_adc3_results);

			free(i2c_values);
			free(hk_adc1_results);
			free(hk_adc3_results);

		}

	} else if (htim == &htim1) {
		if (PMT_ON) {

			while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
			}

			uint8_t *spi1_results = spi(hspi1);

			send_pmt_packet(spi1_results);

			free(spi1_results);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart1, rx_buf, 1);
	unsigned char key = rx_buf[0];

	switch (key) {
	case 0x0B: {
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_SET);
		break;
	}
	case 0x0A: {
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x1B: {
		if (step < 9) {
			step++;
		}
		break;
	}
	case 0x1C: {
		if (step > 0) {
			step--;
		}
		break;
	}
	case 0x1D: {
		if (!auto_sweep) {
			auto_sweep = 1;
			step = 0;
		} else {
			auto_sweep = 0;
			step = 0;
		}
		break;
	}
	case 0x24: {
		if (SAMPLING_FACTOR < 32) {
			SAMPLING_FACTOR *= 2;
			FACTOR_COUNTER = 0;
			SWP_FACTOR_COUNTER = 0;
		}
		break;
	}
	case 0x25: {
		if (SAMPLING_FACTOR > 1) {
			SAMPLING_FACTOR /= 2;
			FACTOR_COUNTER = 0;
			SWP_FACTOR_COUNTER = 0;
		}
		break;
	}
	case 0x00: {
		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_SET);
		break;
	}
	case 0x13: {
		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x01: {

		HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_SET);
		break;
	}
	case 0x14: {
		HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x02: {
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_SET);
		break;
	}
	case 0x15: {
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x03: {
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_SET);
		break;
	}
	case 0x16: {
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x04: {
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_SET);
		break;
	}
	case 0x17: {
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x05: {
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_SET);
		break;
	}
	case 0x18: {
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x06: {
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_SET);
		break;
	}
	case 0x19: {
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x07: {
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_SET);
		break;
	}
	case 0x1A: {
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x0C: {
		HAL_SuspendTick();
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		NVIC_SystemReset();
		break;
	}
	case 0x0D: {
		PMT_ON = 1;
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
		break;
	}
	case 0x10: {
		PMT_ON = 0;
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);

		break;
	}
	case 0x0E: {
		ERPA_ON = 1;
		HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
		break;
	}
	case 0x11: {
		ERPA_ON = 0;
		HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);

		break;
	}
	case 0x0F: {
		HK_ON = 1;
		break;
	}
	case 0x12: {
		HK_ON = 0;
		break;
	}
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  TIM2->CCR4 = 312;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_Base_Start(&htim2);

  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, DAC_OUT, SIZE, DAC_ALIGN_12B_R);


  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
  			ADC_SINGLE_ENDED) != HAL_OK) {
  		/* Calibration Error */
  		Error_Handler();
  	}

  	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC1Data,
  	ADC1NumChannels) != HAL_OK) {
  		Error_Handler();
  	}

  	if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY,
  			ADC_SINGLE_ENDED) != HAL_OK) {
  		/* Calibration Error */
  		Error_Handler();
  	}

  	if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC3Data,
  	ADC3NumChannels) != HAL_OK) {
  		Error_Handler();
  	}

  	while (__HAL_UART_GET_FLAG(&huart1, USART_ISR_BUSY) == SET)
  		;
  	while (__HAL_UART_GET_FLAG(&huart1, USART_ISR_REACK) == RESET)
  		;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_UART_Receive_IT(&huart1, rx_buf, 1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 9;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 11;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00506682;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 480-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3125-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC6 PC7 PC8
                           PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
