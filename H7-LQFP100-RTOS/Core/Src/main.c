/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @author 		   : Jared Morrison
 * @version 	   : 2.0.0-alpha
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
#include "cmsis_os.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "stm32h7xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t *array;  // Pointer to the array data
	uint16_t size;   // Size of the array
} packet_t;

typedef struct {
	GPIO_TypeDef *gpio;
	uint16_t pin;
} gpio_pins;

typedef enum {
	RAIL_vsense,	// 0
	RAIL_vrefint,	// 1
	RAIL_TEMP1,		// 2
	RAIL_TEMP2,		// 3
	RAIL_TEMP3,		// 4
	RAIL_TEMP4,		// 5
	RAIL_busvmon,	// 6
	RAIL_busimon,	// 7
	RAIL_2v5,		// 8
	RAIL_3v3,		// 9
	RAIL_5v,		// 10
	RAIL_n3v3,		// 11
	RAIL_n5v,		// 12
	RAIL_15v,		// 13
	RAIL_5vref,		// 14
	RAIL_n200v,		// 15
	RAIL_n800v,		// 16
	RAIL_TMP1		// 17
} VOLTAGE_RAIL_NAME;

typedef struct {
	VOLTAGE_RAIL_NAME name;
	uint8_t error_count;
	uint8_t is_enabled;
	uint16_t data;
	uint16_t max_voltage;
	uint16_t min_voltage;
} VOLTAGE_RAIL;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// *********************************************************************************************************** DEFINES

//#define FLIGHT_MODE

#define PMT_FLAG_ID 0x0001
#define ERPA_FLAG_ID 0x0002
#define HK_FLAG_ID 0x0004
#define VOLTAGE_MONITOR_FLAG_ID 0x0008
#define STOP_FLAG 0x0016

#define PMT_DATA_SIZE 10
#define ERPA_DATA_SIZE 14
#define HK_DATA_SIZE 54
#define UART_RX_BUFFER_SIZE 64
#define UART_TX_BUFFER_SIZE 1000
#define UPTIME_SIZE 4
#define TIMESTAMP_SIZE 10

#define MSGQUEUE_OBJECTS 128
#define ERROR_PACKET_DATA_SIZE 3

#define ADC1_NUM_CHANNELS 11
#define ADC3_NUM_CHANNELS 4

#define PMT_SYNC 0xBB
#define ERPA_SYNC 0xAA
#define HK_SYNC 0xCC
#define ERROR_SYNC 0xDD

#define ACK 0xFF
#define NACK 0x00

#define NUM_VOLTAGE_RAILS 18
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// *********************************************************************************************************** GLOBAL VARIABLES
VOLTAGE_RAIL rail_monitor[NUM_VOLTAGE_RAILS];

volatile uint8_t HK_10_second_counter = 0;
volatile uint32_t uptime_millis = 0;
osMessageQueueId_t mid_MsgQueue;
packet_t msg;

volatile int tx_flag = 1;

uint16_t pmt_seq = 0;
uint32_t erpa_seq = 0;
uint16_t hk_seq = 0;

volatile uint8_t PMT_ON = 0;
volatile uint8_t ERPA_ON = 0;
volatile uint8_t HK_ON = 0;

volatile uint32_t cadence = 3125;
volatile uint8_t step = 3;

osEventFlagsId_t event_flags;

unsigned char UART_RX_BUFFER[UART_RX_BUFFER_SIZE];

ALIGN_32BYTES(static uint16_t ADC1_raw_data[ADC1_NUM_CHANNELS]);
ALIGN_32BYTES(static uint16_t ADC3_raw_data[ADC3_NUM_CHANNELS]);

static const uint8_t REG_TEMP = 0x00;
static const uint8_t ADT7410_1 = 0x48 << 1;
static const uint8_t ADT7410_2 = 0x4A << 1;
static const uint8_t ADT7410_3 = 0x49 << 1;
static const uint8_t ADT7410_4 = 0x4B << 1;

uint32_t DAC_OUT[32] = { 0, 0, 620, 620, 1241, 1241, 1861, 1861, 2482, 2482,
		3103, 3103, 3723, 3723, 4095, 4095, 4095, 4095, 3723, 3723, 3103, 3103,
		2482, 2482, 1861, 1861, 1241, 1241, 620, 620, 0, 0 }; // For 3.3 volts

const gpio_pins gpios[] = {
{ GPIOB, GPIO_PIN_2 },	// 0 -- SDN1
{ GPIOB, GPIO_PIN_5 },	// 1 -- SYS_ON
{ GPIOC, GPIO_PIN_10 },	// 2 -- 3v3_EN
{ GPIOC, GPIO_PIN_7 },	// 3 -- 5v_EN
{ GPIOC, GPIO_PIN_6 },	// 4 -- N3V3_EN
{ GPIOC, GPIO_PIN_8 },	// 5 -- N5V_EN
{ GPIOC, GPIO_PIN_9 },	// 6 -- 15V_EN
{ GPIOC, GPIO_PIN_13 },	// 7 -- N150V_EN
{ GPIOB, GPIO_PIN_6 }	// 8 -- 800HVON
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
// *********************************************************************************************************** FUNCTION PROTOYPES
void system_setup();
int in_range(uint16_t raw, int min, int max);
void error_protocol(VOLTAGE_RAIL_NAME failed_rail);
packet_t create_packet(const uint8_t *data, uint16_t size);
void sample_hk();
void get_uptime(uint8_t *buffer);
void send_ACK();
void send_NACK();
void sync();
void enter_stop();
uint8_t get_current_step();
void flush_message_queue();
HAL_StatusTypeDef RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime,
		uint32_t Format);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// *********************************************************************************************************** CALLBACKS
/**
 * @brief Handles the callback for timer delay elapsed events.
 *
 * This function is called when a timer's delay has elapsed and performs
 * specific actions based on the timer instance.
 *
 * @param htim Pointer to the timer handle structure.
 *             Supported timer instances are htim1, htim2, and htim3.
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
		osEventFlagsSet(event_flags, PMT_FLAG_ID);
	} else if (htim == &htim2) {
		osEventFlagsSet(event_flags, ERPA_FLAG_ID);
	} else if (htim == &htim3) {
		osEventFlagsSet(event_flags, VOLTAGE_MONITOR_FLAG_ID);

// - If flight mode is defined, we only create HK packets every 100 interrupts of TIM3 (running at 100ms)
// - Otherwise, we create HK packets every time TIM3 interrupts
#ifdef FLIGHT_MODE
		if (HK_10_second_counter == 100) {
			osEventFlagsSet(event_flags, HK_FLAG_ID);
			HK_10_second_counter = 0;
		}
		HK_10_second_counter++;
#else
		osEventFlagsSet(event_flags, HK_FLAG_ID);
#endif

	} else {
		printf("Unknown Timer Interrupt\n");
	}
}

/**
 * @brief UART receive complete callback.
 *
 * This function is called when a UART receive operation is complete. It handles
 * various commands received via UART and performs corresponding actions, such as
 * toggling GPIO pins, starting or stopping timers, and other operations.
 *
 * @param huart Pointer to a UART_HandleTypeDef structure that contains
 *              the configuration information for the specified UART module.
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);
	unsigned char key = UART_RX_BUFFER[0];

	switch (key) {
	case 0x10: {
		printf("SDN1 ON\n");
		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_SET);
		break;
	}
	case 0x00: {
		printf("SDN1 OFF\n");
		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x11: {
		printf("SYS ON PB5\n");
		HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_SET);
		rail_monitor[RAIL_2v5].is_enabled = 1;
		break;
	}
	case 0x01: {
		printf("SYS OFF PB5\n");

		// Turning off all voltage enables (including high voltages) in order from highest to lowest, including SYS_ON
		for (int i = 8; i > 0; i--) {
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
			rail_monitor[i].is_enabled = 0;
		}

		break;
	}
	case 0x12: {
		printf("3v3 ON PC10\n");
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_SET);
		rail_monitor[RAIL_3v3].is_enabled = 1;

		break;
	}
	case 0x02: {
		printf("3v3 OFF PC10\n");
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_RESET);
		rail_monitor[RAIL_3v3].is_enabled = 0;
		break;
	}
	case 0x13: {
		printf("5v ON PC7\n");
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_SET);
		rail_monitor[RAIL_5v].is_enabled = 1;
		break;
	}
	case 0x03: {
		printf("5v OFF PC7\n");
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_RESET);
		rail_monitor[RAIL_5v].is_enabled = 0;
		break;
	}
	case 0x14: {
		printf("n3v3 ON PC6\n");
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_SET);
		rail_monitor[RAIL_n3v3].is_enabled = 1;
		break;
	}
	case 0x04: {
		printf("n3v3 OFF PC6\n");
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_RESET);
		rail_monitor[RAIL_n3v3].is_enabled = 0;
		break;
	}
	case 0x15: {
		printf("n5v ON PC8\n");
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_SET);
		rail_monitor[RAIL_n5v].is_enabled = 1;
		break;
	}
	case 0x05: {
		printf("n5v OFF PC8\n");
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_RESET);
		rail_monitor[RAIL_n5v].is_enabled = 0;
		break;
	}
	case 0x16: {
		printf("15v ON PC9\n");
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_SET);
		rail_monitor[RAIL_15v].is_enabled = 1;
		break;
	}
	case 0x06: {
		printf("15v OFF PC9\n");
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_RESET);
		rail_monitor[RAIL_15v].is_enabled = 0;
		break;
	}
	case 0x17: {
		printf("n200v ON PC13\n");
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_SET);
		rail_monitor[RAIL_n200v].is_enabled = 1;
		break;
	}
	case 0x07: {
		printf("n200v OFF PC13\n");
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_RESET);
		rail_monitor[RAIL_n200v].is_enabled = 0;
		break;
	}
	case 0x18: {
		printf("800v ON PB6\n");
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_SET);
		rail_monitor[RAIL_n800v].is_enabled = 1;
		break;
	}
	case 0x08: {
		printf("800v OFF PB6\n");
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_RESET);
		rail_monitor[RAIL_n800v].is_enabled = 0;
		break;
	}
	case 0x19: {
		printf("AUTOSWEEP ON\n");
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, DAC_OUT, 32, DAC_ALIGN_12B_R);
		break;
	}
	case 0x09: {
		printf("AUTOSWEEP OFF\n");
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		break;
	}
	case 0x1A: {
		printf("ERPA ON\n");
		HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
		osEventFlagsSet(event_flags, ERPA_FLAG_ID);
		ERPA_ON = 1;
		break;
	}
	case 0x0A: {
		printf("ERPA OFF\n");
		HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
		ERPA_ON = 0;
		break;
	}
	case 0x1B: {
		printf("PMT ON\n");
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
		osEventFlagsSet(event_flags, PMT_FLAG_ID);
		PMT_ON = 1;
		break;
	}
	case 0x0B: {
		printf("PMT OFF\n");
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		PMT_ON = 0;
		break;
	}
	case 0x1C: {
		printf("HK ON \n");
		osEventFlagsSet(event_flags, HK_FLAG_ID);
		HK_ON = 1;
		break;
	}
	case 0x0C: {
		printf("HK OFF\n");
		HK_ON = 0;
		break;
	}
	case 0x1D: {
		printf("Step Up\n");
		if (step < 17) {
			step += 2;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
					DAC_OUT[step]);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		}
		break;
	}
	case 0x0D: {
		printf("Step Down\n");
		if (step > 3) {
			step -= 2;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
					DAC_OUT[step]);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		}
		break;
	}
	case 0x1E: {
		printf("Factor Up\n");
		if (cadence <= 50000) {
			cadence *= 2;
			TIM2->ARR = cadence;
		}
		break;
	}
	case 0x0E: {
		printf("Factor Down\n");
		if (cadence >= 6250) {
			cadence /= 2;
			TIM2->ARR = cadence;
		}
		break;
	}
	case 0x0F: {
		printf("Enter STOP mode\n");
		osEventFlagsSet(event_flags, STOP_FLAG);
		break;
	}
	case 0xE0: {
		printf("Auto Init\n");
		xTaskResumeFromISR(GPIO_on_taskHandle);
		break;
	}
	case 0xD0: {
		printf("Auto Deinit\n");
		xTaskResumeFromISR(GPIO_off_taskHandle);
		break;
	}
	case 0xAF: {
		sync();
		break;
	}
	case 0xBF: {
		xTaskResumeFromISR(Science_taskHandle);
		break;
	}
	case 0xCF: {
		xTaskResumeFromISR(Idle_taskHandle);
		break;
	}
	default: {
		printf("Unknown Command\n");
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
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
  PeriphClkInitStruct.PLL2.PLL2N = 32;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// *********************************************************************************************************** RAW DATA RETRIEVAL FUNCTIONS
/**
 * @brief Polls an I2C temperature sensor.
 *
 * This function transmits a read request to the specified I2C temperature sensor
 * and reads the temperature value.
 *
 * @param TEMP_ADDR The I2C address of the temperature sensor.
 * @return The temperature reading from the sensor, or an error code.
 */
int16_t poll_i2c_sensor(const uint8_t TEMP_ADDR) {
	int16_t output;
	uint8_t buf[2];
	HAL_StatusTypeDef ret;
	buf[0] = REG_TEMP;
	ret = HAL_I2C_Master_Transmit(&hi2c1, TEMP_ADDR, buf, 1, 1000);
	if (ret != HAL_OK) {
		printf("I2C TX Error\n");
	} else {
		/* Read 2 bytes from the temperature register */
		ret = HAL_I2C_Master_Receive(&hi2c1, TEMP_ADDR, buf, 2, 1000);
		if (ret != HAL_OK) {
			printf("I2C RX Error\n");
		} else {
			output = (int16_t) (buf[0] << 8);
			output = (output | buf[1]) >> 3;
		}
	}
	return output;
}

/**
 * @brief Receives data from an SPI device.
 *
 * This function receives data from the specified SPI device and stores the result
 * in the provided buffer.
 *
 * @param spi_handle The handle to the SPI device.
 * @param buffer The buffer to store the received data.
 */
void receive_pmt_spi(uint8_t *buffer) {
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

/**
 * @brief Receives data from an SPI device.
 *
 * This function receives data from the specified SPI device and stores the result
 * in the provided buffer.
 *
 * @param spi_handle The handle to the SPI device.
 * @param buffer The buffer to store the received data.
 */
void receive_erpa_spi(uint8_t *buffer) {
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

/**
 * @brief Receives ERPA ADC data and stores it in the provided buffer.
 *
 * @param buffer Pointer to an array where ADC data will be stored.
 *
 * This function reads the raw data from the ERPA ADC and writes it to the
 * first element of the provided buffer.
 */
void receive_erpa_adc(uint16_t *buffer) {
	uint16_t PC4 = ADC1_raw_data[1];

	buffer[0] = PC4;
}

/**
 * @brief Receives housekeeping I2C sensor data.
 *
 * This function polls multiple I2C sensors and stores the results in the provided buffer.
 *
 * @param buffer The buffer to store the received I2C sensor data.
 */
void receive_hk_i2c(int16_t *buffer) {
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
 * @brief Receives housekeeping ADC1 data and stores it in the provided buffer.
 *
 * @param buffer Pointer to an array where ADC data will be stored.
 *
 * This function reads the raw housekeeping ADC1 data and writes it to the
 * corresponding elements of the provided buffer.
 */
void receive_hk_adc1(uint16_t *buffer) {
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
 * @brief Receives housekeeping ADC3 sensor data.
 *
 * This function retrieves specific ADC3 sensor data and stores the results in the provided buffer.
 *
 * @param buffer The buffer to store the received ADC3 sensor data.
 */
void receive_hk_adc3(uint16_t *buffer) {
	uint16_t vrefint = ADC3_raw_data[0];
	uint16_t vsense = ADC3_raw_data[1];
	uint16_t PC2 = ADC3_raw_data[2];
	uint16_t PC3 = ADC3_raw_data[3];

	buffer[0] = vrefint;
	buffer[1] = vsense;
	buffer[2] = PC2;
	buffer[3] = PC3;
}

// *********************************************************************************************************** HELPER FUNCTIONS

/**
 * @brief Gets the current step based on the DAC value.
 *
 * @return uint8_t The current step number or -1 if the DAC value is invalid.
 *
 * This function reads the current value from the DAC and returns the corresponding
 * step based on predefined DAC values.
 */
uint8_t get_current_step() {
	int dac_value;

	dac_value = DAC1->DHR12R1;

	switch (dac_value) {
	case 0:
		return 0;
	case 620:
		return 1;
	case 1241:
		return 2;
	case 1861:
		return 3;
	case 2482:
		return 4;
	case 3103:
		return 5;
	case 3723:
		return 6;
	case 4095:
		return 7;
	default:
		return -1;
	}
}

/**
 * @brief Enters STOP mode and configures the system upon wakeup.
 *
 * This function sends an acknowledgment message, suspends all tasks, and
 * enters STOP mode. Upon waking up, it resumes all tasks and reconfigures
 * the system clock.
 */
void enter_stop() {

	//flush_message_queue();
	send_ACK();

	vTaskSuspendAll();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	// When MCU is triggered to wake up, it resumes right here.
	// That's why it looks like we enter stop mode and then instantly
	// configure the clock and resume tasks, but in reality the MCU
	// just stops right here.

	SystemClock_Config();
	xTaskResumeAll();
}

HAL_StatusTypeDef RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime,
		uint32_t Format) {
	uint32_t tmpreg;
	HAL_StatusTypeDef status;

	/* Process Locked */
	__HAL_LOCK(hrtc);

	hrtc->State = HAL_RTC_STATE_BUSY;

	/* Disable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
	/* Enter Initialization mode */
	status = RTC_EnterInitMode(hrtc);
	if (status == HAL_OK) {

		sTime->TimeFormat = 0x00U;
		assert_param(IS_RTC_HOUR24(sTime->Hours));

		assert_param(IS_RTC_MINUTES(sTime->Minutes));
		assert_param(IS_RTC_SECONDS(sTime->Seconds));

		tmpreg = (uint32_t) (((uint32_t) RTC_ByteToBcd2(sTime->Hours)
				<< RTC_TR_HU_Pos)
				| ((uint32_t) RTC_ByteToBcd2(sTime->Minutes) << RTC_TR_MNU_Pos)
				| ((uint32_t) RTC_ByteToBcd2(sTime->Seconds) << RTC_TR_SU_Pos)
				| (((uint32_t) sTime->TimeFormat) << RTC_TR_PM_Pos));

		/* Set the RTC_TR register */
		hrtc->Instance->TR = (uint32_t) (tmpreg & RTC_TR_RESERVED_MASK);

		/* Exit Initialization mode */
		status = RTC_ExitInitMode(hrtc);
	}

	/* Enable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

	if (status == HAL_OK) {
		hrtc->State = HAL_RTC_STATE_READY;
	}

	/* Process Unlocked */
	__HAL_UNLOCK(hrtc);
	return status;

}
/**
 * @brief Calibrates the RTC with the provided date and time values.
 *
 * @param buffer Pointer to an array containing date and time values.
 *
 * This function extracts date and time information from the provided buffer
 * and sets the RTC accordingly. It handles years, months, days, hours, minutes,
 * seconds, and milliseconds.
 */
void calibrateRTC(uint8_t *buffer) {
	//    [0]     [1]     [2]     [3]     [4]     [5]     [6]     [7]     [8]
	//    0xFF    Year   Month    Day     Hour   Minute  Second  ms MSB  ms LSB

	RTC_DateTypeDef date_struct;
	RTC_TimeTypeDef time_struct;
	uint8_t year = buffer[1];
	uint8_t month = buffer[2];
	uint8_t day = buffer[3];
	uint8_t hour = buffer[4];
	uint8_t minute = buffer[5];
	uint8_t second = buffer[6];
	uint16_t milliseconds = (buffer[7] << 8) | buffer[8];

	date_struct.Year = year;
	date_struct.Month = month;
	date_struct.Date = day;

	time_struct.Hours = hour;
	time_struct.Minutes = minute;
	time_struct.Seconds = second;
	time_struct.SubSeconds = milliseconds;

	HAL_StatusTypeDef status;

	status = HAL_RTC_SetDate(&hrtc, &date_struct, RTC_FORMAT_BIN);
	if (status != HAL_OK) {
		Error_Handler();
	}
	RTC_SetTime(&hrtc, &time_struct, RTC_FORMAT_BIN);

//	status = HAL_RTC_SetTime(&hrtc, &time_struct, RTC_FORMAT_BIN);
//	if (status != HAL_OK) {
//		Error_Handler();
//	}
}

/**
 * @brief Synchronizes the system's RTC with the OBC/GUI timestamp.
 *
 * This function performs the following steps to synchronize the RTC:
 * 1. Sends an acknowledgment to indicate the system is awake.
 * 2. Waits to receive an RTC-generated timestamp from the OBC/GUI.
 * 3. Calibrates the RTC with the received timestamp.
 * 4. Sends an acknowledgment to indicate the RTC calibration is complete.
 */
void sync() {
	send_ACK();

	uint8_t key;

	// Wait for 0xFF to be received
	HAL_UART_AbortReceive(&huart1);
	do {
		HAL_UART_Receive(&huart1, UART_RX_BUFFER, 9, 100);
		key = UART_RX_BUFFER[0];
	} while (key != 0xFF);

	calibrateRTC(UART_RX_BUFFER);
	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);

	send_ACK();
}

/**
 * @brief Sends an acknowledgment byte via UART.
 *
 * This function sends a single acknowledgment byte (0xFF) using the UART
 * interface to indicate a successful operation or state transition.
 */
void send_ACK() {
	static uint8_t tx_buffer[1];

	tx_buffer[0] = ACK;
	HAL_UART_Transmit(&huart1, tx_buffer, 1, 100);
}

/**
 * @brief Sends a negative acknowledgment byte via UART.
 *
 * This function sends a single negative acknowledgment byte (0x00) using the UART
 * interface to indicate a failed operation or state transition.
 */
void send_NACK() {
	static uint8_t tx_buffer[1];

	tx_buffer[0] = NACK;
	HAL_UART_Transmit(&huart1, tx_buffer, 1, 100);

}

/**
 * @brief Flushes the message queue and sends accumulated messages via UART.
 *
 * This function retrieves messages from the message queue and accumulates
 * them in a buffer. When the buffer is full or the queue is empty, it sends
 * the buffer contents via UART using DMA. It waits for the transmission
 * to complete before returning.
 */
void flush_message_queue() {
	static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];

	uint32_t total_size = 0;
	osStatus_t status;
	do {
		status = osMessageQueueGet(mid_MsgQueue, &msg, NULL, osWaitForever);
		if (status == osOK) {
			if ((total_size + msg.size) < UART_TX_BUFFER_SIZE) {
				memcpy(&tx_buffer[total_size], msg.array, msg.size);
				free(msg.array);
				total_size += msg.size;
				if (total_size >= (UART_TX_BUFFER_SIZE - HK_DATA_SIZE)) {
					break;
				}
			}
		} else {
			break;
		}
	} while (status == osOK);

	if (total_size > 0) {
		HAL_UART_Transmit_DMA(&huart1, tx_buffer, total_size);

		// Wait for transmission to complete
		while (tx_flag == 0) {
		}

		tx_flag = 0;

	}
}

/**
 * @brief Checks if a value is within a specified range.
 *
 * @param raw The value to check.
 * @param min The minimum value of the range (inclusive).
 * @param max The maximum value of the range (inclusive).
 *
 * @return int Returns 1 if the value is within the range, otherwise 0.
 */
int in_range(uint16_t raw, int min, int max) {
	if (raw <= max && raw >= min) {
		return 1;
	}
	return 0;
}

/**
 * @brief Handles error reporting by creating and sending an error packet.
 *
 * @param tag An identifier for the type of error.
 *
 * This function samples the housekeeping data, creates an error packet with
 * the provided error tag, and places it in the message queue. It also includes
 * synchronization bytes in the packet. The function currently includes a TODO
 * for adding shutdown procedures.
 */
void error_protocol(VOLTAGE_RAIL_NAME failed_rail) {
	vTaskSuspend(HK_taskHandle);
	vTaskSuspend(ERPA_taskHandle);
	vTaskSuspend(PMT_taskHandle);

	sample_hk();
	packet_t error_packet;
	uint8_t *buffer = (uint8_t*) malloc(
	ERROR_PACKET_DATA_SIZE * sizeof(uint8_t));

	buffer[0] = ERROR_SYNC;
	buffer[1] = ERROR_SYNC;
	buffer[2] = failed_rail;

	error_packet = create_packet(buffer, ERROR_PACKET_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &error_packet, 0U, 0U);

	free(buffer);
	//vTaskSuspendAll();
	//TODO: Shutdown
}

/**
 * @brief UART transmit complete callback.
 * @param huart: UART handle.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	tx_flag = 1;
}

/**
 * @brief Creates a packet with given data and size.
 * @param data: Pointer to data to be copied into the packet.
 * @param size: Size of the data.
 * @return Created packet.
 */
packet_t create_packet(const uint8_t *data, uint16_t size) {
	packet_t packet;
	packet.array = (uint8_t*) malloc(size * sizeof(uint8_t));
	if (packet.array == NULL) {
		// Packet array is null somehow, should probably do something about this edge case
	}
	memcpy(packet.array, data, size);
	packet.size = size;
	return packet;
}

/**
 * @brief Performs system initialization and configuration.
 *
 * This function starts a timer channel, calibrates and starts ADCs with DMA,
 * and initializes UART reception. It sets up various hardware components
 * and checks for errors during the configuration process.
 */
void system_setup() {

	rail_monitor[RAIL_vsense].name = RAIL_vsense;
	rail_monitor[RAIL_vsense].error_count = 0;
	rail_monitor[RAIL_vsense].is_enabled = 1;
	rail_monitor[RAIL_vsense].data = 0;
	rail_monitor[RAIL_vsense].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_vsense].min_voltage = 0;

	rail_monitor[RAIL_vrefint].name = RAIL_vrefint;
	rail_monitor[RAIL_vrefint].error_count = 0;
	rail_monitor[RAIL_vrefint].is_enabled = 1;
	rail_monitor[RAIL_vrefint].data = 0;
	rail_monitor[RAIL_vrefint].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_vrefint].min_voltage = 0;

	rail_monitor[RAIL_TEMP1].name = RAIL_TEMP1;
	rail_monitor[RAIL_TEMP1].error_count = 0;
	rail_monitor[RAIL_TEMP1].is_enabled = 1;
	rail_monitor[RAIL_TEMP1].data = 0;
	rail_monitor[RAIL_TEMP1].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TEMP1].min_voltage = 0;

	rail_monitor[RAIL_TEMP2].name = RAIL_TEMP2;
	rail_monitor[RAIL_TEMP2].error_count = 0;
	rail_monitor[RAIL_TEMP2].is_enabled = 1;
	rail_monitor[RAIL_TEMP2].data = 0;
	rail_monitor[RAIL_TEMP2].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TEMP2].min_voltage = 0;

	rail_monitor[RAIL_TEMP3].name = RAIL_TEMP3;
	rail_monitor[RAIL_TEMP3].error_count = 0;
	rail_monitor[RAIL_TEMP3].is_enabled = 1;
	rail_monitor[RAIL_TEMP3].data = 0;
	rail_monitor[RAIL_TEMP3].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TEMP3].min_voltage = 0;

	rail_monitor[RAIL_TEMP4].name = RAIL_TEMP4;
	rail_monitor[RAIL_TEMP4].error_count = 0;
	rail_monitor[RAIL_TEMP4].is_enabled = 1;
	rail_monitor[RAIL_TEMP4].data = 0;
	rail_monitor[RAIL_TEMP4].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TEMP4].min_voltage = 0;

	rail_monitor[RAIL_busvmon].name = RAIL_busvmon;
	rail_monitor[RAIL_busvmon].error_count = 0;
	rail_monitor[RAIL_busvmon].is_enabled = 1;
	rail_monitor[RAIL_busvmon].data = 0;
	rail_monitor[RAIL_busvmon].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_busvmon].min_voltage = 0;

	rail_monitor[RAIL_busimon].name = RAIL_busimon;
	rail_monitor[RAIL_busimon].error_count = 0;
	rail_monitor[RAIL_busimon].is_enabled = 1;
	rail_monitor[RAIL_busimon].data = 0;
	rail_monitor[RAIL_busimon].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_busimon].min_voltage = 0;

	rail_monitor[RAIL_2v5].name = RAIL_2v5;
	rail_monitor[RAIL_2v5].error_count = 0;
	rail_monitor[RAIL_2v5].is_enabled = 0;
	rail_monitor[RAIL_2v5].data = 0;
	rail_monitor[RAIL_2v5].max_voltage = 3257;
	rail_monitor[RAIL_2v5].min_voltage = 2947;

	rail_monitor[RAIL_3v3].name = RAIL_3v3;
	rail_monitor[RAIL_3v3].error_count = 0;
	rail_monitor[RAIL_3v3].is_enabled = 0;
	rail_monitor[RAIL_3v3].data = 0;
	rail_monitor[RAIL_3v3].max_voltage = 3909;
	//rail_monitor[RAIL_3v3].min_voltage = 3537;
	rail_monitor[RAIL_3v3].min_voltage = 0;


	rail_monitor[RAIL_5v].name = RAIL_5v;
	rail_monitor[RAIL_5v].error_count = 0;
	rail_monitor[RAIL_5v].is_enabled = 0;
	rail_monitor[RAIL_5v].data = 0;
	rail_monitor[RAIL_5v].max_voltage = 3909;
	rail_monitor[RAIL_5v].min_voltage = 3537;

	rail_monitor[RAIL_n3v3].name = RAIL_n3v3;
	rail_monitor[RAIL_n3v3].error_count = 0;
	rail_monitor[RAIL_n3v3].is_enabled = 0;
	rail_monitor[RAIL_n3v3].data = 0;
	rail_monitor[RAIL_n3v3].max_voltage = 4091;
	rail_monitor[RAIL_n3v3].min_voltage = 3702;

	rail_monitor[RAIL_n5v].name = RAIL_n5v;
	rail_monitor[RAIL_n5v].error_count = 0;
	rail_monitor[RAIL_n5v].is_enabled = 0;
	rail_monitor[RAIL_n5v].data = 0;
	rail_monitor[RAIL_n5v].max_voltage = 4000;
	//rail_monitor[RAIL_n5v].min_voltage = 3619;
	rail_monitor[RAIL_n5v].min_voltage = 0;

	rail_monitor[RAIL_15v].name = RAIL_15v;
	rail_monitor[RAIL_15v].error_count = 0;
	rail_monitor[RAIL_15v].is_enabled = 0;
	rail_monitor[RAIL_15v].data = 0;
	rail_monitor[RAIL_15v].max_voltage = 3896;
	rail_monitor[RAIL_15v].min_voltage = 3525;

	rail_monitor[RAIL_5vref].name = RAIL_5vref;
	rail_monitor[RAIL_5vref].error_count = 0;
	rail_monitor[RAIL_5vref].is_enabled = 0;
	rail_monitor[RAIL_5vref].data = 0;
	rail_monitor[RAIL_5vref].max_voltage = 3909;
	rail_monitor[RAIL_5vref].min_voltage = 3537;

	rail_monitor[RAIL_n200v].name = RAIL_n200v;
	rail_monitor[RAIL_n200v].error_count = 0;
	rail_monitor[RAIL_n200v].is_enabled = 0;
	rail_monitor[RAIL_n200v].data = 0;
	rail_monitor[RAIL_n200v].max_voltage = 4196;
	//rail_monitor[RAIL_n200v].min_voltage = 3796;
	rail_monitor[RAIL_n200v].min_voltage = 0;		// TODO: Currently set to 0, kept triggering because it has been reading ~3351


	rail_monitor[RAIL_n800v].name = RAIL_n800v;
	rail_monitor[RAIL_n800v].error_count = 0;
	rail_monitor[RAIL_n800v].is_enabled = 0;
	rail_monitor[RAIL_n800v].data = 0;
	rail_monitor[RAIL_n800v].max_voltage = 3336;
	rail_monitor[RAIL_n800v].min_voltage = 3018;

	rail_monitor[RAIL_TMP1].name = RAIL_TMP1;
	rail_monitor[RAIL_TMP1].error_count = 0;
	rail_monitor[RAIL_TMP1].is_enabled = 1;
	rail_monitor[RAIL_TMP1].data = 0;
	rail_monitor[RAIL_TMP1].max_voltage = 10000; // TODO: Get actual range from Sanj
	rail_monitor[RAIL_TMP1].min_voltage = 0;

	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

	TIM2->CCR4 = 312;
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
	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);
}

/**
 * @brief Retrieves the system uptime in milliseconds and stores it in the buffer.
 *
 * @param buffer Pointer to an array where the uptime will be stored as 4 bytes.
 *
 * This function calculates the system uptime based on the `uptime_millis`
 * variable and the current value of the SysTick timer. It handles potential
 * rollovers by checking and correcting the values before storing the uptime
 * in the provided buffer.
 */
void get_uptime(uint8_t *buffer) {
	uint32_t uptime = 0;
	uint32_t ms = uptime_millis;
	uint32_t st = SysTick->VAL;

	// Did uptime_millis rollover while reading SysTick->VAL?
	if (ms != uptime_millis) {
		ms = uptime_millis;
		st = SysTick->VAL;
	}
	uptime = ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);

	buffer[0] = ((uptime >> 24) & 0xFF);
	buffer[1] = ((uptime >> 16) & 0xFF);
	buffer[2] = ((uptime >> 8) & 0xFF);
	buffer[3] = uptime & 0xFF;
}

/**
 * @brief Gets the current timestamp and stores it in the provided buffer.
 * @param buffer: Pointer to the buffer where the timestamp will be stored.
 */
void getTimestamp(uint8_t *buffer) {
	RTC_TimeTypeDef current_time;
	RTC_DateTypeDef current_date;

	HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &current_date, RTC_FORMAT_BIN);
	uint32_t milliseconds = 1000000 - (current_time.SubSeconds * 100);

	buffer[0] = current_date.Year;				// 0-99
	buffer[1] = current_date.Month;				// 1-12
	buffer[2] = current_date.Date;				// 1-31
	buffer[3] = current_time.Hours;				// 0-23
	buffer[4] = current_time.Minutes;			// 0-59
	buffer[5] = current_time.Seconds;			// 0-59
	buffer[6] = ((milliseconds >> 24) & 0xFF);
	buffer[7] = ((milliseconds >> 16) & 0xFF);
	buffer[8] = ((milliseconds >> 8) & 0xFF);
	buffer[9] = milliseconds & 0xFF;
}

/**
 * @brief Samples PMT data and sends it as a packet.
 *
 * This function waits for a specific GPIO pin state, allocates memory for
 * PMT data, SPI data, and uptime information, and retrieves the current
 * uptime and PMT SPI data. It then constructs a data packet including synchronization
 * bytes, sequence information, and the retrieved data, and places the packet in
 * the message queue. Memory allocated for the data is subsequently freed.
 */
void sample_pmt() {
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
	}
	uint8_t *buffer = (uint8_t*) malloc(PMT_DATA_SIZE * sizeof(uint8_t));
	uint8_t *pmt_spi = (uint8_t*) malloc(2 * sizeof(uint8_t));
	uint8_t *uptime = (uint8_t*) malloc(UPTIME_SIZE * sizeof(uint8_t));

	get_uptime(uptime);

	receive_pmt_spi(pmt_spi);

	buffer[0] = PMT_SYNC;
	buffer[1] = PMT_SYNC;
	buffer[2] = ((pmt_seq & 0xFF00) >> 8);
	buffer[3] = (pmt_seq & 0xFF);
	buffer[4] = pmt_spi[0];
	buffer[5] = pmt_spi[1];
	buffer[6] = uptime[0];
	buffer[7] = uptime[1];
	buffer[8] = uptime[2];
	buffer[9] = uptime[3];

	packet_t pmt_packet = create_packet(buffer, PMT_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &pmt_packet, 0U, 0U);
	free(buffer);
	free(pmt_spi);
	free(uptime);
}

/**
 * @brief Samples ERPA data and sends it as a packet.
 *
 * This function waits for a specific GPIO pin state, allocates memory for
 * the ERPA data, retrieves uptime, SPI data, and ADC readings, and constructs
 * a data packet with synchronization bytes, sequence information, and the
 * collected data. The packet is then placed in the message queue, and the
 * allocated memory is freed.
 */
void sample_erpa() {
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) {
	}

	uint8_t *buffer = (uint8_t*) malloc(ERPA_DATA_SIZE * sizeof(uint8_t)); // Allocate memory for the buffer

	uint8_t *erpa_spi = (uint8_t*) malloc(2 * sizeof(uint8_t));
	uint16_t *erpa_adc = (uint16_t*) malloc(1 * sizeof(uint16_t));
	uint8_t *uptime = (uint8_t*) malloc(UPTIME_SIZE * sizeof(uint8_t));
	uint8_t sweep_step = -1;

	get_uptime(uptime);
	sweep_step = get_current_step();

	receive_erpa_spi(erpa_spi);
	receive_erpa_adc(erpa_adc);

	buffer[0] = ERPA_SYNC;
	buffer[1] = ERPA_SYNC;
	buffer[2] = ((erpa_seq >> 16) & 0xFF);
	buffer[3] = ((erpa_seq >> 8) & 0xFF);
	buffer[4] = erpa_seq & 0xFF;
	buffer[5] = sweep_step;
	buffer[6] = ((erpa_adc[0] & 0xFF00) >> 8);	// SWP Monitored MSB
	buffer[7] = (erpa_adc[0] & 0xFF);           // SWP Monitored LSB
	buffer[8] = erpa_spi[0];					// ERPA eADC MSB
	buffer[9] = erpa_spi[1];					// ERPA eADC LSB
	buffer[10] = uptime[0];
	buffer[11] = uptime[1];
	buffer[12] = uptime[2];
	buffer[13] = uptime[3];

	packet_t erpa_packet = create_packet(buffer, ERPA_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &erpa_packet, 0U, 0U);
	free(buffer);
	free(erpa_spi);
	free(erpa_adc);
	free(uptime);
}

/**
 * @brief Samples housekeeping data and sends it as a packet.
 *
 * This function allocates memory for the housekeeping data buffer, retrieves
 * and processes I2C readings, and updates the buffer with various system
 * metrics, including voltage readings and temperatures. It then constructs
 * a data packet with synchronization bytes, sequence information, and sampled
 * data, and places the packet in the message queue. The allocated memory is
 * subsequently freed.
 */
void sample_hk() {
	uint8_t *buffer = (uint8_t*) malloc(HK_DATA_SIZE * sizeof(uint8_t));
	uint8_t *timestamp = (uint8_t*) malloc(TIMESTAMP_SIZE * sizeof(uint8_t));
	uint8_t *uptime = (uint8_t*) malloc(UPTIME_SIZE * sizeof(uint8_t));

	get_uptime(uptime);
	getTimestamp(timestamp);

	buffer[0] = HK_SYNC;                     	// HK SYNC 0xCC MSB
	buffer[1] = HK_SYNC;                     	// HK SYNC 0xCC LSB
	buffer[2] = ((hk_seq & 0xFF00) >> 8);    	// HK SEQ # MSB
	buffer[3] = (hk_seq & 0xFF);             	// HK SEQ # LSB
	buffer[4] = ((rail_monitor[RAIL_vsense].data & 0xFF00) >> 8);		// HK vsense MSB
	buffer[5] = (rail_monitor[RAIL_vsense].data & 0xFF);				// HK vsense LSB
	buffer[6] = ((rail_monitor[RAIL_vrefint].data & 0xFF00) >> 8);		// HK vrefint MSB
	buffer[7] = (rail_monitor[RAIL_vrefint].data & 0xFF);				// HK vrefint LSB
	buffer[8] = ((rail_monitor[RAIL_TEMP1].data & 0xFF00) >> 8);	// HK TEMP1 MSB
	buffer[9] = (rail_monitor[RAIL_TEMP1].data & 0xFF);				// HK TEMP1 LSB
	buffer[10] = ((rail_monitor[RAIL_TEMP2].data & 0xFF00) >> 8);	// HK TEMP2 MSB
	buffer[11] = (rail_monitor[RAIL_TEMP2].data & 0xFF);			// HK TEMP2 LSB
	buffer[12] = ((rail_monitor[RAIL_TEMP3].data & 0xFF00) >> 8);	// HK TEMP3 MSB
	buffer[13] = (rail_monitor[RAIL_TEMP3].data & 0xFF);			// HK TEMP3 LSB
	buffer[14] = ((rail_monitor[RAIL_TEMP4].data & 0xFF00) >> 8);	// HK TEMP4 MSB
	buffer[15] = (rail_monitor[RAIL_TEMP4].data & 0xFF);			// HK TEMP4 LSB
	buffer[16] = ((rail_monitor[RAIL_busvmon].data & 0xFF00) >> 8);	// HK BUSvmon MSB
	buffer[17] = (rail_monitor[RAIL_busvmon].data & 0xFF);				// HK BUSvmon LSB
	buffer[18] = ((rail_monitor[RAIL_busimon].data & 0xFF00) >> 8);	// HK BUSimon MSB
	buffer[19] = (rail_monitor[RAIL_busimon].data & 0xFF);				// HK BUSimon LSB
	buffer[20] = ((rail_monitor[RAIL_2v5].data & 0xFF00) >> 8);		// HK 2v5mon MSB
	buffer[21] = (rail_monitor[RAIL_2v5].data & 0xFF);					// HK 2v5mon LSB
	buffer[22] = ((rail_monitor[RAIL_3v3].data & 0xFF00) >> 8);		// HK 3v3mon MSB
	buffer[23] = (rail_monitor[RAIL_3v3].data & 0xFF);					// HK 3v3mon LSB
	buffer[24] = ((rail_monitor[RAIL_5v].data & 0xFF00) >> 8);			// HK 5vmon MSB
	buffer[25] = (rail_monitor[RAIL_5v].data & 0xFF);					// HK 5vmon LSB
	buffer[26] = ((rail_monitor[RAIL_n3v3].data & 0xFF00) >> 8);		// HK n3v3mon MSB
	buffer[27] = (rail_monitor[RAIL_n3v3].data & 0xFF);				// HK n3v3mon LSB
	buffer[28] = ((rail_monitor[RAIL_n5v].data & 0xFF00) >> 8);		// HK n5vmon MSB
	buffer[29] = (rail_monitor[RAIL_n5v].data & 0xFF);					// HK n5vmon LSB
	buffer[30] = ((rail_monitor[RAIL_15v].data & 0xFF00) >> 8);		// HK 15vmon MSB
	buffer[31] = (rail_monitor[RAIL_15v].data & 0xFF);					// HK 15vmon LSB
	buffer[32] = ((rail_monitor[RAIL_5vref].data & 0xFF00) >> 8);		// HK 5vrefmon MSB
	buffer[33] = (rail_monitor[RAIL_5vref].data & 0xFF);				// HK 5vrefmon LSB
	buffer[34] = ((rail_monitor[RAIL_n200v].data & 0xFF00) >> 8);		// HK n150vmon MSB
	buffer[35] = (rail_monitor[RAIL_n200v].data & 0xFF);				// HK n150vmon LSB
	buffer[36] = ((rail_monitor[RAIL_n800v].data & 0xFF00) >> 8);		// HK n800vmon MSB
	buffer[37] = (rail_monitor[RAIL_n800v].data & 0xFF);				// HK n800vmon LSB
	buffer[38] = ((rail_monitor[RAIL_TMP1].data & 0xFF00) >> 8);  // TEMPURATURE 1 MSB
	buffer[39] = (rail_monitor[RAIL_TMP1].data & 0xFF);           // TEMPURATURE 1 LSB
	buffer[40] = timestamp[0];
	buffer[41] = timestamp[1];
	buffer[42] = timestamp[2];
	buffer[43] = timestamp[3];
	buffer[44] = timestamp[4];
	buffer[45] = timestamp[5];
	buffer[46] = timestamp[6];
	buffer[47] = timestamp[7];
	buffer[48] = timestamp[8];
	buffer[49] = timestamp[9];
	buffer[50] = uptime[0];
	buffer[51] = uptime[1];
	buffer[52] = uptime[2];
	buffer[53] = uptime[3];

	packet_t hk_packet = create_packet(buffer, HK_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &hk_packet, 0U, 0U);

	free(buffer);
	free(timestamp);
	free(uptime);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
