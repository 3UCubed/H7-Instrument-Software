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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
	RAIL_BUSVMON,	// 0
	RAIL_BUSIMON,	// 1
	RAIL_2v5,		// 2
	RAIL_3v3,		// 3
	RAIL_5v,		// 4
	RAIL_n3v3,		// 5
	RAIL_n5v,		// 6
	RAIL_15v,		// 7
	RAIL_5vref,		// 8
	RAIL_n200v,		// 9
	RAIL_n800v,		// 10
} ERROR_TAGS;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// *********************************************************************************************************** DEFINES
//#define SIMULATE
//#define ERPA_CAP 700
//#define PMT_CAP 80
//#define HK_CAP 10000
#define PMT_FLAG_ID 0x0001
#define ERPA_FLAG_ID 0x0002
#define HK_FLAG_ID 0x0004
#define VOLTAGE_MONITOR_FLAG_ID 0x0008
#define STOP_FLAG 0x0016

#define PMT_DATA_SIZE 10
#define ERPA_DATA_SIZE 15
#define HK_DATA_SIZE 48
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

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

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
/* Definitions for GPIO_on_task */
osThreadId_t GPIO_on_taskHandle;
uint32_t GPIO_on_taskBuffer[ 128 ];
osStaticThreadDef_t GPIO_on_taskControlBlock;
const osThreadAttr_t GPIO_on_task_attributes = {
  .name = "GPIO_on_task",
  .cb_mem = &GPIO_on_taskControlBlock,
  .cb_size = sizeof(GPIO_on_taskControlBlock),
  .stack_mem = &GPIO_on_taskBuffer[0],
  .stack_size = sizeof(GPIO_on_taskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GPIO_off_task */
osThreadId_t GPIO_off_taskHandle;
uint32_t GPIO_off_taskBuffer[ 128 ];
osStaticThreadDef_t GPIO_off_taskControlBlock;
const osThreadAttr_t GPIO_off_task_attributes = {
  .name = "GPIO_off_task",
  .cb_mem = &GPIO_off_taskControlBlock,
  .cb_size = sizeof(GPIO_off_taskControlBlock),
  .stack_mem = &GPIO_off_taskBuffer[0],
  .stack_size = sizeof(GPIO_off_taskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_TX_task */
osThreadId_t UART_TX_taskHandle;
const osThreadAttr_t UART_TX_task_attributes = {
  .name = "UART_TX_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Voltage_Monitor */
osThreadId_t Voltage_MonitorHandle;
const osThreadAttr_t Voltage_Monitor_attributes = {
  .name = "Voltage_Monitor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for FLAG_task */
osThreadId_t FLAG_taskHandle;
const osThreadAttr_t FLAG_task_attributes = {
  .name = "FLAG_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
// *********************************************************************************************************** GLOBAL VARIABLES
uint16_t _vsense;
uint16_t _vrefint;
uint16_t _busvmon;
uint16_t _busimon;
uint16_t _2v5;
uint16_t _3v3;
uint16_t _5v;
uint16_t _n3v3;
uint16_t _n5v;
uint16_t _15v;
uint16_t _5vref;
uint16_t _n200v;
uint16_t _n800v;

volatile uint32_t UptimeMillis = 0;
osMessageQueueId_t mid_MsgQueue;
packet_t msg;

osStatus_t status;
volatile int tx_flag = 1;
volatile int available_msgs = 0;
volatile int TEMPERATURE_COUNTER = 1000; // Starts at 1000 so that temperature sensors are sampled on first hk packet

uint16_t pmt_seq = 0;
uint16_t erpa_seq = 0;
uint16_t hk_seq = 0;

uint8_t PMT_ON = 0;
uint8_t ERPA_ON = 0;
uint8_t HK_ON = 0;

volatile uint32_t cadence = 3125;
uint8_t step = 3;

osEventFlagsId_t event_flags;

unsigned char UART_RX_BUFFER[UART_RX_BUFFER_SIZE];

ALIGN_32BYTES(static uint16_t ADC1_raw_data[ADC1_NUM_CHANNELS]);
ALIGN_32BYTES(static uint16_t ADC3_raw_data[ADC3_NUM_CHANNELS]);

static const uint8_t REG_TEMP = 0x00;
static const uint8_t ADT7410_1 = 0x48 << 1;
static const uint8_t ADT7410_2 = 0x4A << 1;
static const uint8_t ADT7410_3 = 0x49 << 1;
static const uint8_t ADT7410_4 = 0x4B << 1;

uint32_t DAC_OUT[32] = { 0, 0, 0, 0, 620, 620, 1241, 1241, 1861, 1861, 2482,
		2482, 3103, 3103, 3723, 3723, 4095, 4095, 4095, 4095, 3723, 3723, 3103,
		3103, 2482, 2482, 1861, 1861, 1241, 1241, 620, 620 }; // For 3.3 volts

const gpio_pins gpios[] = { { GPIOB, GPIO_PIN_5 }, { GPIOB, GPIO_PIN_6 }, {
GPIOC, GPIO_PIN_7 }, { GPIOC, GPIO_PIN_13 }, { GPIOC, GPIO_PIN_10 }, {
GPIOC, GPIO_PIN_8 }, { GPIOC, GPIO_PIN_9 }, { GPIOC, GPIO_PIN_6 }, {
GPIOB, GPIO_PIN_2 } };
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
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
void PMT_init(void *argument);
void ERPA_init(void *argument);
void HK_init(void *argument);
void GPIO_on_init(void *argument);
void GPIO_off_init(void *argument);
void UART_TX_init(void *argument);
void Voltage_Monitor_init(void *argument);
void FLAG_init(void *argument);

/* USER CODE BEGIN PFP */
// *********************************************************************************************************** FUNCTION PROTOYPES
void system_setup();
int inRange(uint16_t raw, int min, int max);
void error_protocol(ERROR_TAGS tag);
packet_t create_packet(const uint8_t *data, uint16_t size);
void sample_hk();
void getUptime(uint8_t *buffer);
void sendACK();
void sendNACK();
void sync();
void enterStop();
uint8_t getCurrentStep();
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
		osEventFlagsSet(event_flags, HK_FLAG_ID);
	} else if (htim == &htim4) {
		osEventFlagsSet(event_flags, VOLTAGE_MONITOR_FLAG_ID);
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
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_SET);
		break;
	}
	case 0x00: {
		printf("SDN1 OFF\n");
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x11: {
		printf("SYS ON PB5\n");
		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_SET);

		break;
	}
	case 0x01: {
		printf("SYS OFF PB5\n");
		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_RESET); // turning off PB5 & ensuring all other enables are off

		HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_RESET);

		break;
	}
	case 0x12: {
		printf("3v3 ON PC10\n");
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_SET);
		break;
	}
	case 0x02: {
		printf("3v3 OFF PC10\n");
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x13: {
		printf("5v ON PC7\n");
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_SET);
		break;
	}
	case 0x03: {
		printf("5v OFF PC7\n");
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x14: {
		printf("n3v3 ON PC6\n");
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_SET);
		break;
	}
	case 0x04: {
		printf("n3v3 OFF PC6\n");
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x15: {
		printf("n5v ON PC8\n");
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_SET);
		break;
	}
	case 0x05: {
		printf("n5v OFF PC8\n");
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x16: {
		printf("15v ON PC9\n");
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_SET);
		break;
	}
	case 0x06: {
		printf("15v OFF PC9\n");
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_RESET);
		break;
	}
	case 0x17: {
		printf("n200v ON PC13\n");
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_SET);
		break;
	}
	case 0x07: {
		printf("n200v OFF PC13\n");
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_RESET);

		break;
	}
	case 0x18: {
		printf("800v ON PB6\n");
		HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_SET);
		break;
	}
	case 0x08: {
		printf("800v OFF PB6\n");
		HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_RESET);
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
		ERPA_ON = 1;
		erpa_seq = 0;
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
		PMT_ON = 1;
		pmt_seq = 0;
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
		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
		HK_ON = 1;
		hk_seq = 0;
		break;
	}
	case 0x0C: {
		printf("HK OFF\n");
		HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
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
	case 0x1F: {
		printf("Exit STOP mode\n");
		// TODO: Exit stop mode
		break;
	}
	case 0x0F: {
		printf("Enter STOP mode\n");
		osEventFlagsSet(event_flags, STOP_FLAG);
		break;
	}
	case 0xAF: {
		sync();
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

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
	mid_MsgQueue = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(packet_t), NULL);
	if (mid_MsgQueue == NULL) {
		; // Message Queue object not created, handle failure
	}

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of PMT_task */
  PMT_taskHandle = osThreadNew(PMT_init, NULL, &PMT_task_attributes);

  /* creation of ERPA_task */
  ERPA_taskHandle = osThreadNew(ERPA_init, NULL, &ERPA_task_attributes);

  /* creation of HK_task */
  HK_taskHandle = osThreadNew(HK_init, NULL, &HK_task_attributes);

  /* creation of GPIO_on_task */
  GPIO_on_taskHandle = osThreadNew(GPIO_on_init, NULL, &GPIO_on_task_attributes);

  /* creation of GPIO_off_task */
  GPIO_off_taskHandle = osThreadNew(GPIO_off_init, NULL, &GPIO_off_task_attributes);

  /* creation of UART_TX_task */
  UART_TX_taskHandle = osThreadNew(UART_TX_init, NULL, &UART_TX_task_attributes);

  /* creation of Voltage_Monitor */
  Voltage_MonitorHandle = osThreadNew(Voltage_Monitor_init, NULL, &Voltage_Monitor_attributes);

  /* creation of FLAG_task */
  FLAG_taskHandle = osThreadNew(FLAG_init, NULL, &FLAG_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	event_flags = osEventFlagsNew(NULL);
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
  hi2c1.Init.Timing = 0x0020081F;
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

  /** I2C Enable Fast Mode Plus
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 100-1;
  hrtc.Init.SynchPrediv = 10000-1;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 50-1;
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
  sConfigOC.Pulse = 0;
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
  htim2.Init.Prescaler = 50-1;
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
  htim3.Init.Prescaler = 10000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000-1;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
	/* Set the RXFIFO threshold */
	HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_4);

	/* Enable the FIFO mode */
	HAL_UARTEx_EnableFifoMode(&huart1);

	/* Enable MCU wakeup by UART */
	HAL_UARTEx_EnableStopMode(&huart1);

	/* Enable the UART RX FIFO threshold interrupt */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXFT);

	/* Enable the UART wakeup from stop mode interrupt */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_WUF);

	/* Put UART peripheral in reception process */
//	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);
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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
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
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
 * @brief Receives ADC data for ERPA.
 *
 * This function retrieves data from specific ADC channels and stores the values
 * in the provided buffer.
 *
 * @param buffer The buffer to store the received ADC data.
 */
void receive_erpa_adc(uint16_t *buffer) {
	uint16_t PC4 = ADC1_raw_data[1];
	uint16_t PB0 = ADC1_raw_data[5];

	buffer[0] = PC4;
	buffer[1] = PB0;
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
 * @brief Receives housekeeping ADC1 sensor data.
 *
 * This function retrieves multiple ADC1 sensor data and stores the results in the provided buffer.
 *
 * @param buffer The buffer to store the received ADC1 sensor data.
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

uint8_t getCurrentStep(){
	int dacValue;

	dacValue = DAC1->DHR12R1;

	switch (dacValue) {
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

void enterStop(){

	//flushMessageQueue();
	sendACK();

	vTaskSuspendAll();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	// When MCU is triggered to wake up, it resumes right here.
	// That's why it looks like we enter stop mode and then instantly
	// configure the clock and resume tasks, but in reality the MCU
	// just stops right here.

	xTaskResumeAll();
	SystemClock_Config();
}

void calibrateRTC(uint8_t *buffer) {
	//    [0]     [1]     [2]     [3]     [4]     [5]     [6]     [7]     [8]
	//    0xFF    Year   Month    Day     Hour   Minute  Second  ms MSB  ms LSB

	RTC_DateTypeDef dateStruct;
	RTC_TimeTypeDef timeStruct;
	uint8_t year = buffer[1];
	uint8_t month = buffer[2];
	uint8_t day = buffer[3];
	uint8_t hour = buffer[4];
	uint8_t minute = buffer[5];
	uint8_t second = buffer[6];
	uint16_t milliseconds = (buffer[7] << 8) | buffer[8];

	dateStruct.Year = year;
	dateStruct.Month = month;
	dateStruct.Date = day;

	timeStruct.Hours = hour;
	timeStruct.Minutes = minute;
	timeStruct.Seconds = second;
	timeStruct.SubSeconds = milliseconds;

	HAL_StatusTypeDef status;

	status = HAL_RTC_SetDate(&hrtc, &dateStruct, RTC_FORMAT_BIN);
	if (status != HAL_OK) {
		Error_Handler();
	}

	status = HAL_RTC_SetTime(&hrtc, &timeStruct, RTC_FORMAT_BIN);
	if (status != HAL_OK) {
		Error_Handler();
	}
}

void sync() {
	// 1. Send acknowledgement (0xFF) (This tells OBC/GUI that we have woken up)
	// 2. Wait to receive RTC generated timestamp from OBC/GUI
	// 3. Calibrate our RTC from received timestamp
	// 4. Send acknowledgement (0xFF) (This tells OBC/GUI that we have calibrated our RTC, and are now in run mode)
	sendACK();

	uint8_t key;

	// Wait for 0xFF to be received
	HAL_UART_AbortReceive(&huart1);
	do {
		HAL_UART_Receive(&huart1, UART_RX_BUFFER, 9, 100);
		key = UART_RX_BUFFER[0];
	} while (key != 0xFF);

	//calibrateRTC(UART_RX_BUFFER);
	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);

	sendACK();
}


void sendACK() {
	static uint8_t tx_buffer[1];

	tx_buffer[0] = ACK;
	HAL_UART_Transmit(&huart1, tx_buffer, 1, 100);
}

void sendNACK() {
	static uint8_t tx_buffer[1];

	tx_buffer[0] = NACK;
	HAL_UART_Transmit(&huart1, tx_buffer, 1, 100);

}

void flushMessageQueue() {
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
		}
		else{
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

int inRange(uint16_t raw, int min, int max) {
	if (raw <= max && raw >= min) {
		return 1;
	}
	return 0;
}

void error_protocol(ERROR_TAGS tag) {
	sample_hk();
	packet_t error_packet;
	uint8_t *buffer = (uint8_t*) malloc(
	ERROR_PACKET_DATA_SIZE * sizeof(uint8_t));

	buffer[0] = ERROR_SYNC;
	buffer[1] = ERROR_SYNC;
	buffer[2] = tag;

	error_packet = create_packet(buffer, ERROR_PACKET_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &error_packet, 0U, 0U);
	available_msgs++;

	free(buffer);

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
 * @brief Performs system setup tasks.
 *
 * This function initializes various system components including timers, ADC calibration, and DMA for ADC data acquisition.
 * It starts PWM for TIM2, performs ADC calibration for ADC1 and ADC3, and starts DMA for ADC data acquisition.
 * Any errors encountered during these initialization steps are handled by the Error_Handler function.
 */
void system_setup() {
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);

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

void getUptime(uint8_t *buffer) {
	uint32_t uptime = 0;
	uint32_t ms = UptimeMillis;
	uint32_t st = SysTick->VAL;

	// Did UptimeMillis rollover while reading SysTick->VAL?
	if (ms != UptimeMillis) {
		// Rollover occurred so read both again.
		// Must read both because we don't know whether the
		// rollover occurred before or after reading SysTick->VAL.
		// No need to check for another rollover because there is
		// no chance of another rollover occurring so quickly.
		ms = UptimeMillis;
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
	RTC_TimeTypeDef currentTime;
	RTC_DateTypeDef currentDate;

	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
	uint32_t milliseconds = 1000000 - (currentTime.SubSeconds * 100);

	buffer[0] = currentDate.Year;				// 0-99
	buffer[1] = currentDate.Month;				// 1-12
	buffer[2] = currentDate.Date;				// 1-31
	buffer[3] = currentTime.Hours;				// 0-23
	buffer[4] = currentTime.Minutes;			// 0-59
	buffer[5] = currentTime.Seconds;			// 0-59
	buffer[6] = ((milliseconds >> 24) & 0xFF);
	buffer[7] = ((milliseconds >> 16) & 0xFF);
	buffer[8] = ((milliseconds >> 8) & 0xFF);
	buffer[9] = milliseconds & 0xFF;
}

/**
 * @brief Samples data from the PMT.
 *
 * This function samples data from the PMT. If the SIMULATE precompiler directive is set,
 * simulated data is used. Otherwise, SPI communication is used to receive actual data.
 * The sampled data is stored in the provided buffer.
 *
 * @param buffer Pointer to the buffer where sampled data will be stored.
 */
void sample_pmt() {
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
	}
	uint8_t *buffer = (uint8_t*) malloc(PMT_DATA_SIZE * sizeof(uint8_t));
	uint8_t *pmt_spi = (uint8_t*) malloc(2 * sizeof(uint8_t));
	uint8_t *uptime = (uint8_t*) malloc(UPTIME_SIZE * sizeof(uint8_t));

	getUptime(uptime);

#ifdef SIMULATE
	pmt_spi[0] = 0xE;
	pmt_spi[1] = 0xD;
#else
	receive_pmt_spi(pmt_spi);
#endif

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
	available_msgs++;
	free(buffer);
	free(pmt_spi);
	free(uptime);
}

/**
 * @brief Samples data from the ERPA.
 *
 * This function samples data from the ERPA. If the SIMULATE precompiler directive is set,
 * simulated data is used. Otherwise, SPI communication and ADC readings are used to obtain actual data.
 * The sampled data is stored in the provided buffer.
 *
 * @param buffer Pointer to the buffer where sampled data will be stored.
 */
void sample_erpa() {
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) {
	}

	uint8_t *buffer = (uint8_t*) malloc(ERPA_DATA_SIZE * sizeof(uint8_t)); // Allocate memory for the buffer

	uint8_t *erpa_spi = (uint8_t*) malloc(2 * sizeof(uint8_t));
	uint16_t *erpa_adc = (uint16_t*) malloc(2 * sizeof(uint16_t));
	uint8_t *uptime = (uint8_t*) malloc(UPTIME_SIZE * sizeof(uint8_t));
	uint8_t sweep_step = -1;

	getUptime(uptime);
	sweep_step = getCurrentStep();
#ifdef SIMULATE
	erpa_spi[0] = 0xE;
	erpa_spi[1] = 0xD;

	erpa_adc[0] = 0xEE;
	erpa_adc[1] = 0xDD;
#else
	receive_erpa_spi(erpa_spi);
	receive_erpa_adc(erpa_adc);
#endif

	buffer[0] = ERPA_SYNC;
	buffer[1] = ERPA_SYNC;
	buffer[2] = ((erpa_seq & 0xFF00) >> 8);
	buffer[3] = (erpa_seq & 0xFF);
	buffer[4] = sweep_step;
	buffer[5] = ((erpa_adc[0] & 0xFF00) >> 8);	// SWP Monitored MSB
	buffer[6] = (erpa_adc[0] & 0xFF);           // SWP Monitored LSB
	buffer[7] = ((erpa_adc[1] & 0xFF00) >> 8);  // TEMPURATURE 1 MSB
	buffer[8] = (erpa_adc[1] & 0xFF);           // TEMPURATURE 1 LSB
	buffer[9] = erpa_spi[0];					// ERPA eADC MSB
	buffer[10] = erpa_spi[1];					// ERPA eADC LSB
	buffer[11] = uptime[0];
	buffer[12] = uptime[1];
	buffer[13] = uptime[2];
	buffer[14] = uptime[3];

	packet_t erpa_packet = create_packet(buffer, ERPA_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &erpa_packet, 0U, 0U);
	available_msgs++;
	free(buffer);
	free(erpa_spi);
	free(erpa_adc);
	free(uptime);
}

/**
 * @brief Samples data from the HK system.
 *
 * This function samples data from the HK system. If the SIMULATE precompiler directive is set,
 * simulated data is used. Otherwise, actual data is obtained through I2C communication and ADC readings.
 * The sampled data is stored in the provided buffer.
 *
 * @param buffer Pointer to the buffer where sampled data will be stored.
 */
void sample_hk() {
	uint8_t *buffer = (uint8_t*) malloc(HK_DATA_SIZE * sizeof(uint8_t));

	static uint16_t last_i2c_reading[4];
	int16_t *hk_i2c = (int16_t*) malloc(4 * sizeof(int16_t));
	uint8_t *timestamp = (uint8_t*) malloc(TIMESTAMP_SIZE * sizeof(uint8_t));

	getTimestamp(timestamp);

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
	if (TEMPERATURE_COUNTER > 999) {
		receive_hk_i2c(hk_i2c);
		last_i2c_reading[0] = hk_i2c[0];
		last_i2c_reading[1] = hk_i2c[1];
		last_i2c_reading[2] = hk_i2c[2];
		last_i2c_reading[3] = hk_i2c[3];
		TEMPERATURE_COUNTER = 0;
	} else {
		TEMPERATURE_COUNTER++;
		hk_i2c[0] = last_i2c_reading[0];
		hk_i2c[1] = last_i2c_reading[1];
		hk_i2c[2] = last_i2c_reading[2];
		hk_i2c[3] = last_i2c_reading[3];
	}

#endif

	buffer[0] = HK_SYNC;                     	// HK SYNC 0xCC MSB
	buffer[1] = HK_SYNC;                     	// HK SYNC 0xCC LSB
	buffer[2] = ((hk_seq & 0xFF00) >> 8);    	// HK SEQ # MSB
	buffer[3] = (hk_seq & 0xFF);             	// HK SEQ # LSB
	buffer[4] = ((_vsense & 0xFF00) >> 8);		// HK vsense MSB
	buffer[5] = (_vsense & 0xFF);				// HK vsense LSB
	buffer[6] = ((_vrefint & 0xFF00) >> 8);		// HK vrefint MSB
	buffer[7] = (_vrefint & 0xFF);				// HK vrefint LSB
	buffer[8] = ((hk_i2c[0] & 0xFF00) >> 8);	// HK TEMP1 MSB
	buffer[9] = (hk_i2c[0] & 0xFF);				// HK TEMP1 LSB
	buffer[10] = ((hk_i2c[1] & 0xFF00) >> 8);	// HK TEMP2 MSB
	buffer[11] = (hk_i2c[1] & 0xFF);			// HK TEMP2 LSB
	buffer[12] = ((hk_i2c[2] & 0xFF00) >> 8);	// HK TEMP3 MSB
	buffer[13] = (hk_i2c[2] & 0xFF);			// HK TEMP3 LSB
	buffer[14] = ((hk_i2c[3] & 0xFF00) >> 8);	// HK TEMP4 MSB
	buffer[15] = (hk_i2c[3] & 0xFF);			// HK TEMP4 LSB
	buffer[16] = ((_busvmon & 0xFF00) >> 8);	// HK BUSvmon MSB
	buffer[17] = (_busvmon & 0xFF);				// HK BUSvmon LSB
	buffer[18] = ((_busimon & 0xFF00) >> 8);	// HK BUSimon MSB
	buffer[19] = (_busimon & 0xFF);				// HK BUSimon LSB
	buffer[20] = ((_2v5 & 0xFF00) >> 8);		// HK 2v5mon MSB
	buffer[21] = (_2v5 & 0xFF);					// HK 2v5mon LSB
	buffer[22] = ((_3v3 & 0xFF00) >> 8);		// HK 3v3mon MSB
	buffer[23] = (_3v3 & 0xFF);					// HK 3v3mon LSB
	buffer[24] = ((_5v & 0xFF00) >> 8);			// HK 5vmon MSB
	buffer[25] = (_5v & 0xFF);					// HK 5vmon LSB
	buffer[26] = ((_n3v3 & 0xFF00) >> 8);		// HK n3v3mon MSB
	buffer[27] = (_n3v3 & 0xFF);				// HK n3v3mon LSB
	buffer[28] = ((_n5v & 0xFF00) >> 8);		// HK n5vmon MSB
	buffer[29] = (_n5v & 0xFF);					// HK n5vmon LSB
	buffer[30] = ((_15v & 0xFF00) >> 8);		// HK 15vmon MSB
	buffer[31] = (_15v & 0xFF);					// HK 15vmon LSB
	buffer[32] = ((_5vref & 0xFF00) >> 8);		// HK 5vrefmon MSB
	buffer[33] = (_5vref & 0xFF);				// HK 5vrefmon LSB
	buffer[34] = ((_n200v & 0xFF00) >> 8);		// HK n150vmon MSB
	buffer[35] = (_n200v & 0xFF);				// HK n150vmon LSB
	buffer[36] = ((_n800v & 0xFF00) >> 8);		// HK n800vmon MSB
	buffer[37] = (_n800v & 0xFF);				// HK n800vmon LSB
	buffer[38] = timestamp[0];
	buffer[39] = timestamp[1];
	buffer[40] = timestamp[2];
	buffer[41] = timestamp[3];
	buffer[42] = timestamp[4];
	buffer[43] = timestamp[5];
	buffer[44] = timestamp[6];
	buffer[45] = timestamp[7];
	buffer[46] = timestamp[8];
	buffer[47] = timestamp[9];

	packet_t hk_packet = create_packet(buffer, HK_DATA_SIZE);
	osMessageQueuePut(mid_MsgQueue, &hk_packet, 0U, 0U);
	available_msgs++;

	free(buffer);
	free(hk_i2c);
	free(timestamp);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_PMT_init */
// *********************************************************************************************************** RTOS TASK FUNCTIONS
/**
 * @brief  Function implementing the PMT_task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_PMT_init */
void PMT_init(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {

		osEventFlagsWait(event_flags, PMT_FLAG_ID, osFlagsWaitAny,
		osWaitForever);
		if (PMT_ON) {
			sample_pmt();
			pmt_seq++;
#ifdef PMT_CAP
	if (pmt_seq >= PMT_CAP){
		osThreadSuspend(PMT_taskHandle);
	}
#endif

		}
		osThreadYield();
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

	/* Infinite loop */
	for (;;) {
		osEventFlagsWait(event_flags, ERPA_FLAG_ID, osFlagsWaitAny,
		osWaitForever);
		if (ERPA_ON) {
			sample_erpa();
			erpa_seq++;
#ifdef ERPA_CAP
	if (erpa_seq >= ERPA_CAP){
		osThreadSuspend(ERPA_taskHandle);
	}
#endif

		}
		osThreadYield();
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

	/* Infinite loop */
	for (;;) {
		osEventFlagsWait(event_flags, HK_FLAG_ID, osFlagsWaitAny,
		osWaitForever);
		if (HK_ON) {
			sample_hk();
			hk_seq++;
#ifdef HK_CAP
	if (hk_seq >= HK_CAP){
		osThreadSuspend(HK_taskHandle);
	}
#endif
		}
		osThreadYield();
	}
  /* USER CODE END HK_init */
}

/* USER CODE BEGIN Header_GPIO_on_init */
/**
 * @brief Function implementing the GPIO_on_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GPIO_on_init */
void GPIO_on_init(void *argument)
{
  /* USER CODE BEGIN GPIO_on_init */
	osThreadSuspend(GPIO_on_taskHandle);
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_SET); // sdn1
		osDelay(100);
		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_SET); // sys on pb5
		osDelay(100);
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_SET); // 3v3 on pc1
		osDelay(100);
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_SET); // 5v on pc7
		osDelay(100);
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_SET); // n3v3 on pc6
		osDelay(100);
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_SET); // n5v on pc8
		osDelay(100);
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_SET); // 15v on pc9
		osThreadSuspend(GPIO_on_taskHandle);
	}
  /* USER CODE END GPIO_on_init */
}

/* USER CODE BEGIN Header_GPIO_off_init */
/**
 * @brief Function implementing the GPIO_off_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GPIO_off_init */
void GPIO_off_init(void *argument)
{
  /* USER CODE BEGIN GPIO_off_init */
	osThreadSuspend(GPIO_off_taskHandle);
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_RESET); // 15v on pc9
		osDelay(100);
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_RESET); // n5v on pc8
		osDelay(100);
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_RESET); // n3v3 on pc6
		osDelay(100);
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_RESET); // 5v on pc7
		osDelay(100);
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_RESET); // 3v3 on pc1
		osDelay(100);
		HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_RESET); // sys on pb5
		osDelay(100);
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_RESET); // sdn1
		osThreadSuspend(GPIO_off_taskHandle);
	}
  /* USER CODE END GPIO_off_init */
}

/* USER CODE BEGIN Header_UART_TX_init */
/**
 * @brief Function implementing the UART_TX_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UART_TX_init */
void UART_TX_init(void *argument)
{
  /* USER CODE BEGIN UART_TX_init */
	static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];

	uint32_t total_size = 0;
	osStatus_t status;

	while (1) {
		total_size = 0;
		// Retrieve all messages from the queue and store them in tx_buffer
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
			}
		} while (status == osOK);

		if (total_size > 0) {
			HAL_UART_Transmit_DMA(&huart1, tx_buffer, total_size);

			// Wait for transmission to complete
			while (tx_flag == 0) {
				osThreadYield();
			}

			// Reset the flag
			tx_flag = 0;
		}

		// Yield thread control
		osThreadYield();
	}
  /* USER CODE END UART_TX_init */
}

/* USER CODE BEGIN Header_Voltage_Monitor_init */
/**
 * @brief Function implementing the Voltage_Monitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Voltage_Monitor_init */
void Voltage_Monitor_init(void *argument)
{
  /* USER CODE BEGIN Voltage_Monitor_init */
	/* Infinite loop */


	for (;;) {
		osEventFlagsWait(event_flags, VOLTAGE_MONITOR_FLAG_ID, osFlagsWaitAny,
		osWaitForever);

		uint16_t *hk_adc1 = (uint16_t*) malloc(9 * sizeof(uint16_t));
		uint16_t *hk_adc3 = (uint16_t*) malloc(4 * sizeof(uint16_t));

		receive_hk_adc1(hk_adc1);
		receive_hk_adc3(hk_adc3);

		_vsense = hk_adc3[1];
		_vrefint = hk_adc3[0];
		_busvmon = hk_adc1[0];
		_busimon = hk_adc1[1];
		_2v5 = hk_adc1[2];
		_3v3 = hk_adc3[3];
		_5v = hk_adc1[6];
		_n3v3 = hk_adc1[3];
		_n5v = hk_adc3[2];
		_15v = hk_adc1[7];
		_5vref = hk_adc1[8];
		_n200v = hk_adc1[4];
		_n800v = hk_adc1[5];

//		if (!inRange(_busvmon, 1574, 1739)) {
//			error_protocol(RAIL_BUSVMON);
//		}
//
//		if (!inRange(_busimon, 35, 39)) {
//			error_protocol(RAIL_BUSIMON);
//		}
//
//		if (!inRange(_2v5, 2947, 3257)) {
//			error_protocol(RAIL_2v5);
//		}
//
//		if (!inRange(_3v3, 3537, 3909)) {
//			error_protocol(RAIL_3v3);
//		}
//
//		if (!inRange(_5v, 3537, 3909)) {
//			error_protocol(RAIL_5v);
//		}
//
//		if (!inRange(_n3v3, 3702, 4091)) {
//			error_protocol(RAIL_n3v3);
//		}
//
//		if (!inRange(_n5v, 3619, 4000)) {
//			error_protocol(RAIL_n5v);
//		}
//
//		if (!inRange(_15v, 3525, 3896)) {
//			error_protocol(RAIL_15v);
//		}
//
//		if (!inRange(_5vref, 3537, 3909)) {
//			error_protocol(RAIL_5vref);
//		}
//
//		if (!inRange(_n200v, 3796, 4196)) {
//			error_protocol(RAIL_n200v);
//		}
//
//		if (!inRange(_n800v, 3018, 3336)) {
//			error_protocol(RAIL_n800v);
//		}

		free(hk_adc1);
		free(hk_adc3);

		osThreadYield();
	}
  /* USER CODE END Voltage_Monitor_init */
}

/* USER CODE BEGIN Header_FLAG_init */
/**
* @brief Function implementing the FLAG_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FLAG_init */
void FLAG_init(void *argument)
{
  /* USER CODE BEGIN FLAG_init */
  /* Infinite loop */
  for(;;)
  {
		int current_flag = osEventFlagsGet(event_flags);

		if ((current_flag & STOP_FLAG) != 0) {
			osEventFlagsClear(event_flags, STOP_FLAG);
			enterStop();
		}
    osDelay(1);
  }
  /* USER CODE END FLAG_init */
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
