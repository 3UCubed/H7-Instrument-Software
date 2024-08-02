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
#include "stm32h7xx_hal.h"

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
/* Definitions for Science_task */
osThreadId_t Science_taskHandle;
const osThreadAttr_t Science_task_attributes = {
  .name = "Science_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Idle_task */
osThreadId_t Idle_taskHandle;
const osThreadAttr_t Idle_task_attributes = {
  .name = "Idle_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
// *********************************************************************************************************** GLOBAL VARIABLES
uint8_t _2v5_enabled = 0;
uint8_t _3v3_enabled = 0;
uint8_t _5v_enabled = 0;
uint8_t _n3v3_enabled = 0;
uint8_t _n5v_enabled = 0;
uint8_t _15v_enabled = 0;
uint8_t _5vref_enabled = 0;
uint8_t _n200v_enabled = 0;
uint8_t _n800v_enabled = 0;

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
uint16_t _tmp1;

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
void PMT_init(void *argument);
void ERPA_init(void *argument);
void HK_init(void *argument);
void GPIO_on_init(void *argument);
void GPIO_off_init(void *argument);
void UART_TX_init(void *argument);
void Voltage_Monitor_init(void *argument);
void FLAG_init(void *argument);
void Science_init(void *argument);
void Idle_init(void *argument);

/* USER CODE BEGIN PFP */
// *********************************************************************************************************** FUNCTION PROTOYPES
void system_setup();
int in_range(uint16_t raw, int min, int max);
void error_protocol(ERROR_TAGS tag);
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
		_2v5_enabled = 1;
		break;
	}
	case 0x01: {
		printf("SYS OFF PB5\n");

		// Turning off all voltage enables (including high voltages) in order from highest to lowest, including SYS_ON
		for (int i = 8; i > 0; i--) {
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
		}

		_2v5_enabled = 0;
		_3v3_enabled = 0;
		_5v_enabled = 0;
		_n3v3_enabled = 0;
		_n5v_enabled = 0;
		_15v_enabled = 0;
		_5vref_enabled = 0;
		_n200v_enabled = 0;
		_n800v_enabled = 0;

		break;
	}
	case 0x12: {
		printf("3v3 ON PC10\n");
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_SET);
		_3v3_enabled = 1;
		break;
	}
	case 0x02: {
		printf("3v3 OFF PC10\n");
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_RESET);
		_3v3_enabled = 0;
		break;
	}
	case 0x13: {
		printf("5v ON PC7\n");
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_SET);
		_5v_enabled = 1;
		break;
	}
	case 0x03: {
		printf("5v OFF PC7\n");
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_RESET);
		_5v_enabled = 0;
		break;
	}
	case 0x14: {
		printf("n3v3 ON PC6\n");
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_SET);
		_n3v3_enabled = 1;
		break;
	}
	case 0x04: {
		printf("n3v3 OFF PC6\n");
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_RESET);
		_n3v3_enabled = 0;
		break;
	}
	case 0x15: {
		printf("n5v ON PC8\n");
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_SET);
		_n5v_enabled = 1;
		break;
	}
	case 0x05: {
		printf("n5v OFF PC8\n");
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_RESET);
		_n5v_enabled = 0;
		break;
	}
	case 0x16: {
		printf("15v ON PC9\n");
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_SET);
		_15v_enabled = 1;
		break;
	}
	case 0x06: {
		printf("15v OFF PC9\n");
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_RESET);
		_15v_enabled = 0;
		break;
	}
	case 0x17: {
		printf("n200v ON PC13\n");
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_SET);
		_n200v_enabled = 1;
		break;
	}
	case 0x07: {
		printf("n200v OFF PC13\n");
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_RESET);
		_n200v_enabled = 0;

		break;
	}
	case 0x18: {
		printf("800v ON PB6\n");
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_SET);
		_n800v_enabled = 1;
		break;
	}
	case 0x08: {
		printf("800v OFF PB6\n");
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_RESET);
		_n800v_enabled = 0;
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

  /* creation of Science_task */
  Science_taskHandle = osThreadNew(Science_init, NULL, &Science_task_attributes);

  /* creation of Idle_task */
  Idle_taskHandle = osThreadNew(Idle_init, NULL, &Idle_task_attributes);

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
  htim3.Init.Prescaler = 100-1;
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
 * @brief Performs system initialization and configuration.
 *
 * This function starts a timer channel, calibrates and starts ADCs with DMA,
 * and initializes UART reception. It sets up various hardware components
 * and checks for errors during the configuration process.
 */
void system_setup() {
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
	int16_t *hk_i2c = (int16_t*) malloc(4 * sizeof(int16_t));
	uint8_t *timestamp = (uint8_t*) malloc(TIMESTAMP_SIZE * sizeof(uint8_t));
	uint8_t *uptime = (uint8_t*) malloc(UPTIME_SIZE * sizeof(uint8_t));

	get_uptime(uptime);
	getTimestamp(timestamp);
	receive_hk_i2c(hk_i2c);


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
	buffer[38] = ((_tmp1 & 0xFF00) >> 8);  // TEMPURATURE 1 MSB
	buffer[39] = (_tmp1 & 0xFF);           // TEMPURATURE 1 LSB
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
	free(hk_i2c);
	free(timestamp);
	free(uptime);
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

		// Enabling all voltages from SDN1 to 15V (inclusive)
		for (int i = 0; i < 7; i++) {
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_SET);
			osDelay(100);
		}

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

		// Disabling all voltages from 15V to SDN1 (inclusive)
		for (int i = 6; i >= 0; i--) {
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
			osDelay(100);
		}

		osThreadSuspend(GPIO_off_taskHandle);
	}
  /* USER CODE END GPIO_off_init */
}

/* USER CODE BEGIN Header_UART_TX_init */
/**
 * @brief UART transmission initialization and processing task.
 *
 * This function continuously retrieves messages from the message queue,
 * stores them in a buffer, and sends the buffer data via UART using DMA.
 * It handles transmission completion by waiting for a flag and yields thread
 * control periodically. The function operates in an infinite loop, processing
 * and transmitting data as long as the task is running.
 *
 * @param argument Pointer to the argument passed to the thread (not used).
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

	// TODO: Figure out how we should monitor busvmon, busimon, vsense, and vrefint
	for (;;) {
		osEventFlagsWait(event_flags, VOLTAGE_MONITOR_FLAG_ID, osFlagsWaitAny,
		osWaitForever);

		uint16_t *hk_adc1 = (uint16_t*) malloc(10 * sizeof(uint16_t));
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
		_tmp1 = hk_adc1[9];

#ifdef FLIGHT_MODE
		if (_2v5_enabled){
			if (!in_range(_2v5, 2947, 3257)) {
				error_protocol(RAIL_2v5);
			}
		}

		if (_3v3_enabled){
			if (!in_range(_3v3, 3537, 3909)) {
				error_protocol(RAIL_3v3);
			}
		}

		if (_5v_enabled){
			if (!in_range(_5v, 3537, 3909)) {
				error_protocol(RAIL_5v);
			}
		}

		if (_n3v3_enabled){
			if (!in_range(_n3v3, 3702, 4091)) {
				error_protocol(RAIL_n3v3);
			}
		}

		if (_n5v_enabled) {
			if (!in_range(_n5v, 3619, 4000)) {
				error_protocol(RAIL_n5v);
			}
		}

		if (_15v_enabled) {
			if (!in_range(_15v, 3525, 3896)) {
				error_protocol(RAIL_15v);
			}
		}

		if (_5vref_enabled) {
			if (!in_range(_5vref, 3537, 3909)) {
				error_protocol(RAIL_5vref);
			}
		}

		if (_n200v_enabled) {
			if (!in_range(_n200v, 3796, 4196)) {
				error_protocol(RAIL_n200v);
			}
		}

		if (_n800v_enabled) {
			if (!in_range(_n800v, 3018, 3336)) {
				error_protocol(RAIL_n800v);
			}
		}
#endif

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
	for (;;) {
		int current_flag = osEventFlagsGet(event_flags);

		if ((current_flag & STOP_FLAG) != 0) {
			osEventFlagsClear(event_flags, STOP_FLAG);
			enter_stop();
		}
		osDelay(1);
	}
  /* USER CODE END FLAG_init */
}

/* USER CODE BEGIN Header_Science_init */
/**
* @brief Function implementing the Science_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Science_init */
void Science_init(void *argument)
{
  /* USER CODE BEGIN Science_init */
	osThreadSuspend(Science_taskHandle);

  /* Infinite loop */
  for(;;)
  {
		// Enabling all voltages
		for (int i = 0; i < 9; i++) {
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_SET);
			osDelay(200);
		}

		__disable_irq();

		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, DAC_OUT, 32, DAC_ALIGN_12B_R);	// Enable auto sweep (doesn't start until ERPA timer is started)
		HK_ON = 1;
		HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);			// ERPA packet on
		ERPA_ON = 1;
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);				// PMT packet on
		PMT_ON = 1;

		osEventFlagsSet(event_flags, HK_FLAG_ID);
		osEventFlagsSet(event_flags, ERPA_FLAG_ID);
		osEventFlagsSet(event_flags, PMT_FLAG_ID);

		__enable_irq();

		osThreadSuspend(Science_taskHandle);
  }
  /* USER CODE END Science_init */
}

/* USER CODE BEGIN Header_Idle_init */
/**
* @brief Function implementing the Idle_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Idle_init */
void Idle_init(void *argument)
{
  /* USER CODE BEGIN Idle_init */
	osThreadSuspend(Idle_taskHandle);

  /* Infinite loop */
  for(;;)
  {
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		PMT_ON = 0;
		HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
		ERPA_ON = 0;
		HK_ON = 0;
		osDelay(100);

		// Disabling all voltages
		for (int i = 8; i >= 0; i--) {
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
			osDelay(200);
		}

		osThreadSuspend(Idle_taskHandle);
  }
  /* USER CODE END Idle_init */
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
