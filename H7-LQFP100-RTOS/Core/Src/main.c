/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @author 		   : Jared Morrison
 * @version 	   : 1.0.0
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
#include "packet_creation.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "ramecc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief Enumeration of predefined DAC values.
 *        Represents specific output values for the DAC.
 */
typedef enum
{
    DAC_VALUE_0 = 0,
    DAC_VALUE_620 = 620,
    DAC_VALUE_1241 = 1241,
    DAC_VALUE_1861 = 1861,
    DAC_VALUE_2482 = 2482,
    DAC_VALUE_3103 = 3103,
    DAC_VALUE_3723 = 3723,
    DAC_VALUE_4095 = 4095
} DAC_VALUES;

/**
 * @brief Enumeration of accepted command codes.
 *        Defines a set of command values for controlling system power, modes, and error handling.
 */
typedef enum
{
	CMD_SDN1_ON = 0x10,
	CMD_SDN1_OFF = 0x00,
	CMD_SYS_ON = 0x11,
	CMD_SYS_OFF = 0x01,
	CMD_5V_ON = 0x12,
	CMD_5V_OFF = 0x02,
	CMD_N3V3_ON = 0x13,
	CMD_N3V3_OFF = 0x03,
	CMD_N5V_ON = 0x14,
	CMD_N5V_OFF = 0x04,
	CMD_15V_ON = 0x15,
	CMD_15V_OFF = 0x05,
	CMD_N200V_ON = 0x16,
	CMD_N200V_OFF = 0x06,
	CMD_N800V_ON = 0x17,
	CMD_N800V_OFF = 0x07,
	CMD_AUTOSWEEP_ON = 0x18,
	CMD_AUTOSWEEP_OFF = 0x08,
	CMD_ERPA_ON = 0x19,
	CMD_ERPA_OFF = 0x09,
	CMD_PMT_ON = 0x1A,
	CMD_PMT_OFF = 0x0A,
	CMD_HK_ON = 0x1B,
	CMD_HK_OFF = 0x0B,
	CMD_STEP_UP = 0x1C,
	CMD_STEP_DOWN = 0x0C,
	CMD_FACTOR_UP = 0x1D,
	CMD_FACTOR_DOWN = 0x0D,
	CMD_AUTO_INIT = 0x1E,
	CMD_AUTO_DEINIT = 0x0E,
	CMD_SYNC_MODE = 0xA0,
	CMD_SCIENCE_MODE = 0xA1,
	CMD_IDLE_MODE = 0xA2,
	CMD_ENTER_STOP = 0xA3,
	CMD_RESET_ERROR_COUNTERS = 0xA4,
	CMD_SEND_PREVIOUS_ERROR = 0xA5,
	CMD_SEND_VERSION_PACKET = 0xA6
}ACCEPTED_COMMANDS;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HK_100MS_COUNTER_MAX 32

#define ACK 0xFF
#define NACK 0x00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/**
 * @brief Array of GPIO pin configurations.
 *        Represents the mapping of specific GPIO ports and pins to system components.
 */
const gpio_pins gpios[] =
{
		{ GPIOB, GPIO_PIN_2 	},	// 0 -- SDN1
		{ GPIOB, GPIO_PIN_5 	},	// 1 -- SYS_ON
		{ GPIOC, GPIO_PIN_7 	},	// 2 -- 5v_EN
		{ GPIOC, GPIO_PIN_6 	},	// 3 -- N3V3_EN
		{ GPIOC, GPIO_PIN_8 	},	// 4 -- N5V_EN
		{ GPIOC, GPIO_PIN_9 	},	// 5 -- 15V_EN
		{ GPIOC, GPIO_PIN_13 	},	// 6 -- N150V_EN
		{ GPIOB, GPIO_PIN_6 	}	// 7 -- 800HVON
};

/**
 * @brief Array of DAC output values representing sweeping up and down voltages.
 *
 * DAC_OUT alternates between specified DAC values to control the output voltage.
 * The first half sweeps up, reaching a maximum at DAC_VALUE_4095, then sweeps down.
 */
uint32_t DAC_OUT[DAC_OUT_ARRAY_SIZE] =
{
		DAC_VALUE_0, DAC_VALUE_0,		// |	Sweeping Up
		DAC_VALUE_620, DAC_VALUE_620,	// |
		DAC_VALUE_1241, DAC_VALUE_1241,	// |
		DAC_VALUE_1861, DAC_VALUE_1861,	// |
		DAC_VALUE_2482, DAC_VALUE_2482,	// |
		DAC_VALUE_3103, DAC_VALUE_3103,	// |
		DAC_VALUE_3723, DAC_VALUE_3723,	// V

		DAC_VALUE_4095, DAC_VALUE_4095,	// --	Max Sweep
		DAC_VALUE_4095, DAC_VALUE_4095, // --

		DAC_VALUE_3723, DAC_VALUE_3723,	// |	Sweeping Down
		DAC_VALUE_3103, DAC_VALUE_3103,	// |
		DAC_VALUE_2482, DAC_VALUE_2482,	// |
		DAC_VALUE_1861, DAC_VALUE_1861,	// |
		DAC_VALUE_1241, DAC_VALUE_1241,	// |
		DAC_VALUE_620, DAC_VALUE_620,	// |
		DAC_VALUE_0, DAC_VALUE_0		// V
}; // For 3.3 volts

osEventFlagsId_t packet_event_flags;
osEventFlagsId_t utility_event_flags;
osEventFlagsId_t mode_event_flags;

unsigned char UART_RX_BUFFER[UART_RX_BUFFER_SIZE];
volatile uint8_t HK_ENABLED = DISABLED;
volatile uint8_t ERPA_ENABLED = DISABLED;
volatile uint8_t step = 0;
volatile uint32_t cadence = 3125;
volatile uint32_t uptime_millis = 0;
volatile uint8_t HK_100_ms_counter = 0;
volatile uint8_t IDLING = 1;
volatile uint8_t startup_pmt_sent = 0;
volatile uint8_t startup_erpa_sent = 0;
volatile uint8_t startup_hk_sent = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void system_setup();
void init_flash_ecc();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Callback function for handling ECC correction in flash memory.
 *        Detects and handles single-bit flash ECC errors.
 */
void HAL_FLASHEx_EccCorrectionCallback()
{
	ERROR_STRUCT error;
	error.category = EC_seu;
	error.detail = ED_single_bit_error_flash;
	handle_error(error);
}

/**
 * @brief Callback function for handling ECC detection in flash memory.
 *        Detects and handles double-bit flash ECC errors.
 */
void HAL_FLASHEx_EccDetectionCallback()
{
	ERROR_STRUCT error;
	error.category = EC_seu;
	error.detail = ED_double_bit_error_flash;
	handle_error(error);
}

/**
 * @brief Output compare callback for handling timer events.
 *        Sets event flags based on the triggered timer and manages housekeeping tasks.
 *
 * @param htim Pointer to the timer handle triggering the callback.
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		if (startup_pmt_sent == 0)
		{
			pmt_seq = 0;
			startup_pmt_sent = 1;
		}
		osEventFlagsSet(packet_event_flags, PMT_FLAG);
	}
	else if (htim == &htim2)
	{
		if (startup_erpa_sent == 0)
		{
			erpa_seq = 0;
			startup_erpa_sent = 1;
		}
		if (ERPA_ENABLED)
		{
			osEventFlagsSet(packet_event_flags, ERPA_FLAG);
		}
		if (HK_100_ms_counter == HK_100MS_COUNTER_MAX)
		{
			osEventFlagsSet(utility_event_flags, VOLTAGE_MONITOR_FLAG);
			if (startup_hk_sent == 0)
			{
				hk_seq = 0;
				startup_hk_sent = 1;
			}
			if (HK_ENABLED)
			{
				osEventFlagsSet(packet_event_flags, HK_FLAG);
			}
			HK_100_ms_counter = 0;
		}
		HK_100_ms_counter++;
	}
	else
	{
		// Unknown timer interrupt
	}
}

/**
 * @brief UART receive complete callback.
 *        Processes received commands to control GPIO pins and manage power rails.
 *
 * @param huart Pointer to the UART handle triggering the callback.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);
	uint8_t key_index = 0;
	unsigned char key = UART_RX_BUFFER[key_index];

	switch (key)
	{
	case CMD_SDN1_ON:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_SDN1].gpio, gpios[GPIOS_INDEX_SDN1].pin, GPIO_PIN_SET);
		break;
	}

	case CMD_SDN1_OFF:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_SDN1].gpio, gpios[GPIOS_INDEX_SDN1].pin, GPIO_PIN_RESET);
		break;
	}

	case CMD_SYS_ON:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_SYS].gpio, gpios[GPIOS_INDEX_SYS].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_2v5, ENABLED);
		break;
	}

	case CMD_SYS_OFF:
	{
		for (int i = GPIOS_INDEX_N800V; i > GPIOS_INDEX_SDN1; i--)
		{
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
		}

		for (int i = RAIL_n800v; i >= RAIL_2v5; i--)
		{
			set_rail_monitor_enable(i, DISABLED);
		}
		break;
	}

	case CMD_5V_ON:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_5V].gpio, gpios[GPIOS_INDEX_5V].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_5v, ENABLED);
		break;
	}

	case CMD_5V_OFF:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_5V].gpio, gpios[GPIOS_INDEX_5V].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_5v, DISABLED);
		break;
	}

	case CMD_N3V3_ON:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_N3V3].gpio, gpios[GPIOS_INDEX_N3V3].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_n3v3, ENABLED);
		break;
	}

	case CMD_N3V3_OFF:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_N3V3].gpio, gpios[GPIOS_INDEX_N3V3].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_n3v3, DISABLED);
		break;
	}

	case CMD_N5V_ON:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_N5V].gpio, gpios[GPIOS_INDEX_N5V].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_n5v, ENABLED);
		break;
	}

	case CMD_N5V_OFF:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_N5V].gpio, gpios[GPIOS_INDEX_N5V].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_n5v, DISABLED);
		break;
	}

	case CMD_15V_ON:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_15V].gpio, gpios[GPIOS_INDEX_15V].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_15v, ENABLED);
		break;
	}

	case CMD_15V_OFF:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_15V].gpio, gpios[GPIOS_INDEX_15V].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_15v, DISABLED);
		break;
	}

	case CMD_N200V_ON:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_N200V].gpio, gpios[GPIOS_INDEX_N200V].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_n200v, ENABLED);
		break;
	}

	case CMD_N200V_OFF:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_N200V].gpio, gpios[GPIOS_INDEX_N200V].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_n200v, DISABLED);
		break;
	}

	case CMD_N800V_ON:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_N800V].gpio, gpios[GPIOS_INDEX_N800V].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_n800v, ENABLED);
		break;
	}

	case CMD_N800V_OFF:
	{
		HAL_GPIO_WritePin(gpios[GPIOS_INDEX_N800V].gpio, gpios[GPIOS_INDEX_N800V].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_n800v, DISABLED);
		break;
	}

	case CMD_AUTOSWEEP_ON:
	{
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, DAC_OUT, DAC_OUT_ARRAY_SIZE, DAC_ALIGN_12B_R);
		break;
	}

	case CMD_AUTOSWEEP_OFF:
	{
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		break;
	}

	case CMD_ERPA_ON:
	{
		osEventFlagsSet(packet_event_flags, ERPA_FLAG);
		TIM2->CCR4 = ERPA_PWM_FREQ;
		ERPA_ENABLED = ENABLED;
		break;
	}

	case CMD_ERPA_OFF:
	{
		ERPA_ENABLED = DISABLED;
		TIM2->CCR4 = 0;
		break;
	}

	case CMD_PMT_ON:
	{
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
		osEventFlagsSet(packet_event_flags, PMT_FLAG);
		break;
	}

	case CMD_PMT_OFF:
	{
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		break;
	}

	case CMD_HK_ON:
	{
		osEventFlagsSet(packet_event_flags, HK_FLAG);
		HK_ENABLED = ENABLED;
		break;
	}

	case CMD_HK_OFF:
	{
		HK_ENABLED = DISABLED;
		break;
	}

	case CMD_STEP_UP:
	{
		if (step < 14)
		{
			step += 2;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
					DAC_OUT[step]);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		}
		break;
	}

	case CMD_STEP_DOWN:
	{
		if (step > 1)
		{
			step -= 2;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
					DAC_OUT[step]);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		}
		break;
	}

	case CMD_FACTOR_UP:
	{
		if (cadence <= 50000)
		{
			cadence *= 2;
			TIM2->ARR = cadence;
		}
		break;
	}

	case CMD_FACTOR_DOWN:
	{
		if (cadence >= 6250)
		{
			cadence /= 2;
			TIM2->ARR = cadence;
		}
		break;
	}

	case CMD_ENTER_STOP:
	{
		osEventFlagsSet(utility_event_flags, STOP_FLAG);
		break;
	}

	case CMD_AUTO_INIT:
	{
		osEventFlagsSet(utility_event_flags, AUTOINIT_FLAG);
		break;
	}

	case CMD_AUTO_DEINIT:
	{
		osEventFlagsSet(utility_event_flags, AUTODEINIT_FLAG);
		break;
	}

	case CMD_SYNC_MODE:
	{
		osEventFlagsSet(mode_event_flags, SYNC_FLAG);
		break;
	}

	case CMD_SCIENCE_MODE:
	{
		osEventFlagsSet(mode_event_flags, SCIENCE_FLAG);
		break;
	}

	case CMD_IDLE_MODE:
	{
		osEventFlagsSet(mode_event_flags, IDLE_FLAG);
		break;
	}

	case CMD_RESET_ERROR_COUNTERS:
	{
		reset_error_counters();
		break;
	}

	case CMD_SEND_PREVIOUS_ERROR:
	{
#ifdef ERROR_HANDLING_ENABLED
		send_previous_error_packet();
#endif
		break;
	}

	case CMD_SEND_VERSION_PACKET:
	{
		create_version_packet();
		break;
	}

	default:
	{
		// Unknown command
		break;
	}
	}
}

/**
 * @brief Retrieves and handles the cause of a system reset.
 *        Checks for watchdog and brownout reset conditions and reports errors.
 */
ERROR_STRUCT get_reset_cause()
{
	ERROR_STRUCT error;
	error.category = EC_UNDEFINED;
	error.detail = ED_UNDEFINED;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST))
    {
        error.category = EC_watchdog;
        error.detail = ED_UNDEFINED;
        __HAL_RCC_CLEAR_RESET_FLAGS();
        increment_error_counter(error);
        set_previous_error(error);
    }

    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
    {
        error.category = EC_brownout;
        error.detail = ED_UNDEFINED;
        __HAL_RCC_CLEAR_RESET_FLAGS();
        increment_error_counter(error);
        set_previous_error(error);
    }

	return error;
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
  MX_IWDG1_Init();
  MX_RAMECC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  system_setup();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
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
  PeriphClkInitStruct.PLL2.PLL2N = 24;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Initializes the system and its components.
 *        Sets up error handling, event flags, timer, voltage monitoring, ADC, and UART reception.
 */
void system_setup()
{
#ifdef ERROR_HANDLING_ENABLED
	error_counter_init();
	init_flash_ecc();
#endif

	packet_event_flags = osEventFlagsNew(NULL);
    if (packet_event_flags == NULL)
    {
        Error_Handler();
    }

    utility_event_flags = osEventFlagsNew(NULL);
    if (utility_event_flags == NULL)
    {
        Error_Handler();
    }

    mode_event_flags = osEventFlagsNew(NULL);
    if (mode_event_flags == NULL)
    {
        Error_Handler();
    }

    TIM2->CCR4 = 0;
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);

	voltage_monitor_init();

	init_adc_dma();

	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);
}

/**
 * @brief Sends an acknowledgment (ACK) over UART.
 *        Transmits a single byte indicating a successful operation.
 */
void send_ACK()
{
	static uint8_t tx_buffer[1];

	tx_buffer[0] = ACK;
	HAL_UART_Transmit(&huart1, tx_buffer, 1, UART_TIMEOUT_MS);
}

/**
 * @brief Sends a negative acknowledgment (NACK) over UART.
 *        Transmits a single byte indicating an unsuccessful operation.
 */
void send_NACK()
{
	static uint8_t tx_buffer[1];

	tx_buffer[0] = NACK;
	HAL_UART_Transmit(&huart1, tx_buffer, 1, UART_TIMEOUT_MS);
}

/**
 * @brief Retrieves the current step value based on the DAC output.
 *
 * @return The corresponding step value based on the DAC1 output, or -1 if out of range.
 */
STEP_VALUES get_current_step()
{
	int dac_value;

	dac_value = DAC1->DHR12R1;

	switch (dac_value)
	{
	case DAC_VALUE_0:
		return STEP_0;

	case DAC_VALUE_620:
		return STEP_1;

	case DAC_VALUE_1241:
		return STEP_2;

	case DAC_VALUE_1861:
		return STEP_3;

	case DAC_VALUE_2482:
		return STEP_4;

	case DAC_VALUE_3103:
		return STEP_5;

	case DAC_VALUE_3723:
		return STEP_6;

	case DAC_VALUE_4095:
		return STEP_7;

	default:
		return INVALID_STEP;
	}
}

/**
 * @brief Enters low-power stop mode after sending an acknowledgment.
 *        Suspends all FreeRTOS tasks, enters stop mode, and resumes configuration upon wake-up.
 */
void enter_stop()
{
	  send_ACK();

	  vTaskSuspendAll();
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	  // MCU resumes here after waking from stop mode.

	  NVIC_SystemReset();
	  SystemClock_Config();
	  reset_packet_sequence_numbers();
	  xTaskResumeAll();
}

/**
 * @brief Initializes Flash ECC (Error Correction Code) settings.
 *        Unlocks Flash memory, sets IRQ priority, and enables ECC correction and detection interrupts.
 */
void init_flash_ecc()
{
	HAL_FLASH_Unlock();

	HAL_NVIC_SetPriority(FLASH_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(FLASH_IRQn);
	HAL_FLASHEx_EnableEccCorrectionInterrupt();
	HAL_FLASHEx_EnableEccDetectionInterrupt();
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
	HAL_IWDG_Refresh(&hiwdg1);

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim == &htim3)
  {
	NVIC_SystemReset();
  }
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
	ERROR_STRUCT error;
	error.category = EC_peripheral;
	error.detail = ED_UNDEFINED;
	handle_error(error);
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
