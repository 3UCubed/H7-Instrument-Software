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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIMULATE

#define PMT_FLAG_ID 0x0001
#define ERPA_FLAG_ID 0x0002
#define HK_FLAG_ID 0x0004

#define PMT_DATA_SIZE 6
#define ERPA_DATA_SIZE 14
#define HK_DATA_SIZE 38
#define UART_RX_BUFFER_SIZE 100

#define ADC1_NUM_CHANNELS 11
#define ADC3_NUM_CHANNELS 4

#define PMT_SYNC 0xBB
#define ERPA_SYNC 0xAA
#define HK_SYNC 0xCC

static const uint8_t REG_TEMP = 0x00;
static const uint8_t ADT7410_1 = 0x48 << 1;
static const uint8_t ADT7410_2 = 0x4A << 1;
static const uint8_t ADT7410_3 = 0x49 << 1;
static const uint8_t ADT7410_4 = 0x4B << 1;
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

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for PMT_task */
osThreadId_t PMT_taskHandle;
const osThreadAttr_t PMT_task_attributes = {
  .name = "PMT_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ERPA_task */
osThreadId_t ERPA_taskHandle;
const osThreadAttr_t ERPA_task_attributes = {
  .name = "ERPA_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for HK_task */
osThreadId_t HK_taskHandle;
const osThreadAttr_t HK_task_attributes = {
  .name = "HK_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_RX_task */
osThreadId_t UART_RX_taskHandle;
const osThreadAttr_t UART_RX_task_attributes = {
  .name = "UART_RX_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t pmt_seq = 0;
uint8_t erpa_seq = 0;
uint8_t hk_seq = 0;

osEventFlagsId_t event_flags;

unsigned char UART_RX_BUFFER[UART_RX_BUFFER_SIZE];

ALIGN_32BYTES(static uint16_t ADC1_raw_data[ADC1_NUM_CHANNELS]);
ALIGN_32BYTES(static uint16_t ADC3_raw_data[ADC3_NUM_CHANNELS]);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
void PMT_init(void *argument);
void ERPA_init(void *argument);
void HK_init(void *argument);
void UART_RX_init(void *argument);

/* USER CODE BEGIN PFP */
void system_setup();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Handles the callback for timer delay elapsed events.
 *
 * This function is called when a timer's delay has elapsed and performs
 * specific actions based on the timer instance.
 *
 * @param htim Pointer to the timer handle structure.
 *             Supported timer instances are htim1, htim2, and htim3.
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
	  osEventFlagsSet(event_flags, PMT_FLAG_ID); // Set the event flag for Task1

  }
  else if (htim == &htim2)
  {
	  osEventFlagsSet(event_flags, ERPA_FLAG_ID);
  }
  else if (htim == &htim3)
  {
	  osEventFlagsSet(event_flags, HK_FLAG_ID);
  }
  else
  {
	  printf("Unknown Timer Interrupt\n");
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);
	unsigned char key = UART_RX_BUFFER[0];

	switch (key) {
	case 0x0B: {
		printf("SDN1 ON\n");
//		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_SET);
		break;
	}
	case 0x0A: {
		printf("SDN1 OFF\n");
//		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x1B: {
		printf("Step Up\n");
//		if (step < 17) {
//			step+=2;
//		}
		break;
	}
	case 0x1C: {
		printf("Step Down\n");
//		if (step > 3) {
//			step-=2;
//		}
		break;
	}
	case 0x1D: {
		printf("Toggle AutoSweep\n");
//		if (!auto_sweep) {
//			auto_sweep = 1;
//			HAL_TIM_Base_Start(&htim2);
//
//			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, DAC_OUT, SIZE, DAC_ALIGN_12B_R);
//
//		} else {
//			auto_sweep = 0;
//			HAL_TIM_Base_Stop(&htim2);
//
//			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
//		}
		break;
	}
	case 0x24: {
		printf("Factor Up\n");
//		if (cadence <= 50000){
//			cadence *= 2;
//			TIM2->ARR = cadence;
//		}
		break;
	}
	case 0x25: {
		printf("Factor Down\n");
//		if (cadence >= 6250){
//			cadence /= 2;
//			TIM2->ARR = cadence;
//		}
		break;
	}
	case 0x00: {
		printf("SYS ON PB5\n");
//		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_SET);
		break;
	}
	case 0x13: {
		printf("SYS OFF PB5\n");
//		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x01: {

		printf("800v ON PB6\n");
//		HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_SET);
		break;
	}
	case 0x14: {
		printf("800v OFF PB6\n");
//		HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x02: {
		printf("5v ON PC2\n");
//		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_SET);
		break;
	}
	case 0x15: {
		printf("5v OFF PC2\n");
//		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x03: {
		printf("n200v ON PC13\n");
//		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_SET);
		break;
	}
	case 0x16: {
		printf("n200v OFF PC13\n");
//		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x04: {
		printf("3v3 ON PC7\n");
//		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_SET);
		break;
	}
	case 0x17: {
		printf("3v3 OFF PC7\n");
//		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x05: {
		printf("n5v ON PC8\n");
//		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_SET);
		break;
	}
	case 0x18: {
		printf("n5v OFF PC8\n");
//		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x06: {
		printf("15v ON PC9\n");
//		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_SET);
		break;
	}
	case 0x19: {
		printf("15v OFF PC9\n");
//		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x07: {
		printf("n3v3 ON PC6\n");
//		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_SET);
		break;
	}
	case 0x1A: {
		printf("n3v3 OFF PC6\n");
//		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x0C: {
		printf("Enter STOP mode\n");
//		HAL_SuspendTick();
//		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//		NVIC_SystemReset();
		break;
	}
	case 0x0D: {
		printf("PMT ON\n");
//		PMT_ON = 1;
//		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
		break;
	}
	case 0x10: {
		printf("PMT OFF\n");
//		PMT_ON = 0;
//		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		break;
	}
	case 0x0E: {
		printf("ERPA ON\n");
		//ERPA_ON = 1;
		//HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
		break;
	}
	case 0x11: {
		printf("ERPA OFF\n");
		//ERPA_ON = 0;
		//HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
		break;
	}
	case 0x0F: {
		printf("HK ON \n");
		//HK_ON = 1;
		//HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
		break;
	}
	case 0x12: {
		printf("HK OFF\n");
		//HK_ON = 0;
		//HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of PMT_task */
  PMT_taskHandle = osThreadNew(PMT_init, NULL, &PMT_task_attributes);

  /* creation of ERPA_task */
  ERPA_taskHandle = osThreadNew(ERPA_init, NULL, &ERPA_task_attributes);

  /* creation of HK_task */
  HK_taskHandle = osThreadNew(HK_init, NULL, &HK_task_attributes);

  /* creation of UART_RX_task */
  UART_RX_taskHandle = osThreadNew(UART_RX_init, NULL, &UART_RX_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  event_flags = osEventFlagsNew(NULL); // Create an event flags group
  system_setup();
  printf("Starting kernal...\n");
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
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
  hi2c1.Init.Timing = 0x00707CBB;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 128-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 62500-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3125-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 460800;
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
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void system_setup()
{
	  TIM2->CCR4 = 312;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);




	  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
	  			ADC_SINGLE_ENDED) != HAL_OK) {
	  		/* Calibration Error */
	  		Error_Handler();
	  	}

	  	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC1_raw_data,
	  	ADC1_NUM_CHANNELS) != HAL_OK) {
	  		Error_Handler();
	  	}

	  	if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY,
	  			ADC_SINGLE_ENDED) != HAL_OK) {
	  		/* Calibration Error */
	  		Error_Handler();
	  	}

	  	if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC3_raw_data,
	  	ADC3_NUM_CHANNELS) != HAL_OK) {
	  		Error_Handler();
	  	}
}

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

void receive_spi(SPI_HandleTypeDef spi_handle, uint8_t *buffer)
{
	uint8_t spi_raw_data[2];
	uint8_t spi_MSB;
	uint8_t spi_LSB;

	HAL_SPI_Receive(&spi_handle, (uint8_t*) spi_raw_data, 1, 1);

	spi_LSB = ((spi_raw_data[0] & 0xFF00) >> 8);
	spi_MSB = (spi_raw_data[1] & 0xFF);

	spi_handle.Instance->CR1 |= 1 << 10;

	buffer[0] = spi_MSB;
	buffer[1] = spi_LSB;
}

void receive_erpa_adc(uint16_t *buffer)
{
	uint16_t PC4 = ADC1_raw_data[1]; 			// SWPmon --
	uint16_t PB0 = ADC1_raw_data[5]; 	// TEMP1 -- verified doesn't need to change

	buffer[0] = PC4;
	buffer[1] = PB0;
}

void receive_hk_i2c(int16_t *buffer)
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

void receive_hk_adc1(uint16_t *buffer)
{
	uint16_t PA1 = ADC1_raw_data[10];// BUSVmon -- verified doesn't need to change
	uint16_t PA2 = ADC1_raw_data[8];	// BUSImon -- verified doesn't need to change
	uint16_t PC0 = ADC1_raw_data[6];		// 2v5mon -- verified doesn't need to change
	uint16_t PA3 = ADC1_raw_data[9];				// n3v3mon --
	uint16_t PB1 = ADC1_raw_data[2];		// n200v -- verified doesn't need to change
	uint16_t PA7 = ADC1_raw_data[3];				// n800v --
	uint16_t PC1 = ADC1_raw_data[7];				// 5vmon --
	uint16_t PC5 = ADC1_raw_data[4];		// 15vmon -- verified doesn't need to change
	uint16_t PA6 = ADC1_raw_data[0];				// 5vrefmon --

	buffer[0] = PA1;
	buffer[1] = PA2;
	buffer[2] = PC0;
	buffer[3] = PA3;
	buffer[4] = PB1;
	buffer[5] = PA7;
	buffer[6] = PC1;
	buffer[7] = PC5;
	buffer[8] = PA6;
}

void receive_hk_adc3(uint16_t *buffer)
{
	uint16_t vrefint = ADC3_raw_data[0];
	uint16_t vsense = ADC3_raw_data[1];
	uint16_t PC2 = ADC3_raw_data[2]; 		// n5vmon --
	uint16_t PC3 = ADC3_raw_data[3];			// 3v3mon --

	buffer[0] = vrefint;
	buffer[1] = vsense;
	buffer[2] = PC2;
	buffer[3] = PC3;
}

void sample_pmt(uint8_t *buffer)
{
	uint8_t pmt_spi[2];


#ifdef SIMULATE
	pmt_spi[0] = 0xE;
	pmt_spi[1] = 0xD;
#else
	receive_spi(hspi1, pmt_spi);
#endif

	buffer[0] = PMT_SYNC;
	buffer[1] = PMT_SYNC;
	buffer[2] = ((pmt_seq & 0xFF00) >> 8);
	buffer[3] = (pmt_seq & 0xFF);
	buffer[4] = pmt_spi[0];
	buffer[5] = pmt_spi[1];

}


void sample_erpa(uint8_t *buffer)
{
	uint8_t erpa_spi[2];
	uint16_t erpa_adc[2];

#ifdef SIMULATE
	erpa_spi[0] = 0xE;
	erpa_spi[1] = 0xD;

	erpa_adc[0] = 0xEE;
	erpa_adc[1] = 0xDD;
#else
	receive_spi(hspi2);
	reveice_erpa_adc(erpa_adc);
#endif

	buffer[0] = ERPA_SYNC;
	buffer[1] = ERPA_SYNC;
	buffer[2] = ((erpa_seq & 0xFF00) >> 8);
	buffer[3] = (erpa_seq & 0xFF);
	buffer[4] = ((0 & 0xFF00) >> 8); 	  						// ENDmon MSB
	buffer[5] = (0 & 0xFF);               					// ENDmon LSB
	buffer[6] = ((erpa_adc[0] & 0xFF00) >> 8);    // SWP Monitored MSB
	buffer[7] = (erpa_adc[0] & 0xFF);             // SWP Monitored LSB
	buffer[8] = ((erpa_adc[1] & 0xFF00) >> 8);    // TEMPURATURE 1 MSB
	buffer[9] = (erpa_adc[1] & 0xFF);             // TEMPURATURE 1 LSB
	buffer[10] = ((0 & 0xFF00) >> 8);     				// TEMPURATURE 2 MSB
	buffer[11] = (0 & 0xFF);                    			// TEMPURATURE 2 LSB
	buffer[12] = erpa_spi[0];									// ERPA eADC MSB
	buffer[13] = erpa_spi[1];									// ERPA eADC LSB
}


void sample_hk(uint8_t *buffer)
{
	int16_t hk_i2c[4];
	uint16_t hk_adc1[9];
	uint16_t hk_adc3[4];

#ifdef SIMULATE
	hk_i2c[0] = 0x11;
	hk_i2c[1] = 0x12;
	hk_i2c[2] = 0x13;
	hk_i2c[3] = 0x14;

	hk_adc1[0] = 0xA0;
	hk_adc1[1] = 0xA1;
	hk_adc1[2] = 0xA2;
	hk_adc1[3] = 0xA3;
	hk_adc1[4] = 0xA4;
	hk_adc1[5] = 0xA5;
	hk_adc1[6] = 0xA6;
	hk_adc1[7] = 0xA7;
	hk_adc1[8] = 0xA8;

	hk_adc3[0] = 0xB0;
	hk_adc3[1] = 0xB1;
	hk_adc3[2] = 0xB2;
	hk_adc3[3] = 0xB3;
#else
	receive_hk_i2c(hk_i2c);
	receive_hk_adc1(hk_adc1);
	receive_hk_adc3(hk_adc3);
#endif

	buffer[0] = HK_SYNC;                     			// HK SYNC 0xCC MSB
	buffer[1] = HK_SYNC;                     			// HK SYNC 0xCC LSB
	buffer[2] = ((hk_seq & 0xFF00) >> 8);    			// HK SEQ # MSB
	buffer[3] = (hk_seq & 0xFF);             			// HK SEQ # LSB
	buffer[4] = ((hk_adc3[1] & 0xFF00) >> 8);	// HK vsense MSB
	buffer[5] = (hk_adc3[1] & 0xFF);			// HK vsense LSB
	buffer[6] = ((hk_adc3[0] & 0xFF00) >> 8);	// HK vrefint MSB
	buffer[7] = (hk_adc3[0] & 0xFF);			// HK vrefint LSB
	buffer[8] = ((hk_i2c[0] & 0xFF00) >> 8);		// HK TEMP1 MSB
	buffer[9] = (hk_i2c[0] & 0xFF);					// HK TEMP1 LSB
	buffer[10] = ((hk_i2c[1] & 0xFF00) >> 8);		// HK TEMP2 MSB
	buffer[11] = (hk_i2c[1] & 0xFF);				// HK TEMP2 LSB
	buffer[12] = ((hk_i2c[2] & 0xFF00) >> 8);		// HK TEMP3 MSB
	buffer[13] = (hk_i2c[2] & 0xFF);				// HK TEMP3 LSB
	buffer[14] = ((hk_i2c[3] & 0xFF00) >> 8);		// HK TEMP4 MSB
	buffer[15] = (hk_i2c[3] & 0xFF);				// HK TEMP4 LSB
	buffer[16] = ((hk_adc1[0] & 0xFF00) >> 8);	// HK BUSvmon MSB
	buffer[17] = (hk_adc1[0] & 0xFF);			// HK BUSvmon LSB
	buffer[18] = ((hk_adc1[1] & 0xFF00) >> 8);	// HK BUSimon MSB
	buffer[19] = (hk_adc1[1] & 0xFF);			// HK BUSimon LSB
	buffer[20] = ((hk_adc1[2] & 0xFF00) >> 8);	// HK 2v5mon MSB
	buffer[21] = (hk_adc1[2] & 0xFF);			// HK 2v5mon LSB
	buffer[22] = ((hk_adc1[3] & 0xFF00) >> 8);	// HK 3v3mon MSB
	buffer[23] = (hk_adc1[3] & 0xFF);			// HK 3v3mon LSB
	buffer[24] = ((hk_adc1[6] & 0xFF00) >> 8);	// HK 5vmon MSB
	buffer[25] = (hk_adc1[6] & 0xFF);			// HK 5vmon LSB
	buffer[26] = ((hk_adc1[3] & 0xFF00) >> 8);	// HK n3v3mon MSB
	buffer[27] = (hk_adc1[3] & 0xFF);			// HK n3v3mon LSB
	buffer[28] = ((hk_adc1[2] & 0xFF00) >> 8);	// HK n5vmon MSB
	buffer[29] = (hk_adc1[2] & 0xFF);			// HK n5vmon LSB
	buffer[30] = ((hk_adc1[7] & 0xFF00) >> 8);	// HK 15vmon MSB
	buffer[31] = (hk_adc1[7] & 0xFF);			// HK 15vmon LSB
	buffer[32] = ((hk_adc1[8] & 0xFF00) >> 8);	// HK 5vrefmon MSB
	buffer[33] = (hk_adc1[8] & 0xFF);			// HK 5vrefmon LSB
	buffer[34] = ((hk_adc1[4] & 0xFF00) >> 8);	// HK n150vmon MSB
	buffer[35] = (hk_adc1[4] & 0xFF);			// HK n150vmon LSB
	buffer[36] = ((hk_adc1[5] & 0xFF00) >> 8);	// HK n800vmon MSB
	buffer[37] = (hk_adc1[5] & 0xFF);			// HK n800vmon LSB

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_PMT_init */
/**
  * @brief  Function implementing the PMT_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_PMT_init */
void PMT_init(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t pmt_data[PMT_DATA_SIZE];
	/* Infinite loop */
	for (;;) {
	    osEventFlagsWait(event_flags, PMT_FLAG_ID, osFlagsWaitAny, osWaitForever);
	    sample_pmt(pmt_data);
		pmt_seq++;
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ERPA_init */
/**
* @brief Function implementing the ERPA_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ERPA_init */
void ERPA_init(void *argument)
{
  /* USER CODE BEGIN ERPA_init */
	uint8_t erpa_data[ERPA_DATA_SIZE];
  /* Infinite loop */
  for(;;)
  {
	    osEventFlagsWait(event_flags, ERPA_FLAG_ID, osFlagsWaitAny, osWaitForever);
	    sample_erpa(erpa_data);
		erpa_seq++;
  }
  /* USER CODE END ERPA_init */
}

/* USER CODE BEGIN Header_HK_init */
/**
* @brief Function implementing the HK_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HK_init */
void HK_init(void *argument)
{
  /* USER CODE BEGIN HK_init */
	uint8_t hk_data[HK_DATA_SIZE];
  /* Infinite loop */
  for(;;)
  {
	    osEventFlagsWait(event_flags, HK_FLAG_ID, osFlagsWaitAny, osWaitForever);
	    sample_hk(hk_data);
		hk_seq++;
  }
  /* USER CODE END HK_init */
}

/* USER CODE BEGIN Header_UART_RX_init */
/**
* @brief Function implementing the UART_RX_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_RX_init */
void UART_RX_init(void *argument)
{
  /* USER CODE BEGIN UART_RX_init */
  /* Infinite loop */
  for(;;)
  {
		HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);
		osDelay(1);
  }
  /* USER CODE END UART_RX_init */
}

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