/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : freertos.c
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Implementation of freertos tasks.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "packet_queue.h"
#include "usart.h"
#include "voltage_monitor.h"
#include "packet_creation.h"
#include "dac.h"
#include "tim.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RETRY_DELAY 1
#define PACKET_GAP 1
#define PS_RAIL_DELAY 100
#define IDLE_TO_SCIENCE_DELAY 1000
#define IDLE_TO_VOLTAGE_MONITOR_DELAY 3500	// TODO: Reduce to 1000 for assembled instrument
#define SYNC_DELAY 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for PMT_task */
osThreadId_t PMT_taskHandle;
uint32_t PMT_taskBuffer[ 128 ];
osStaticThreadDef_t PMT_taskControlBlock;
const osThreadAttr_t PMT_task_attributes = {
  .name = "PMT_task",
  .cb_mem = &PMT_taskControlBlock,
  .cb_size = sizeof(PMT_taskControlBlock),
  .stack_mem = &PMT_taskBuffer[0],
  .stack_size = sizeof(PMT_taskBuffer),
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for ERPA_task */
osThreadId_t ERPA_taskHandle;
uint32_t ERPA_taskBuffer[ 128 ];
osStaticThreadDef_t ERPA_taskControlBlock;
const osThreadAttr_t ERPA_task_attributes = {
  .name = "ERPA_task",
  .cb_mem = &ERPA_taskControlBlock,
  .cb_size = sizeof(ERPA_taskControlBlock),
  .stack_mem = &ERPA_taskBuffer[0],
  .stack_size = sizeof(ERPA_taskBuffer),
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for HK_task */
osThreadId_t HK_taskHandle;
uint32_t HK_taskBuffer[ 128 ];
osStaticThreadDef_t HK_taskControlBlock;
const osThreadAttr_t HK_task_attributes = {
  .name = "HK_task",
  .cb_mem = &HK_taskControlBlock,
  .cb_size = sizeof(HK_taskControlBlock),
  .stack_mem = &HK_taskBuffer[0],
  .stack_size = sizeof(HK_taskBuffer),
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for AUTOINIT_task */
osThreadId_t AUTOINIT_taskHandle;
uint32_t AUTOINIT_taskBuffer[ 128 ];
osStaticThreadDef_t AUTOINIT_taskControlBlock;
const osThreadAttr_t AUTOINIT_task_attributes = {
  .name = "AUTOINIT_task",
  .cb_mem = &AUTOINIT_taskControlBlock,
  .cb_size = sizeof(AUTOINIT_taskControlBlock),
  .stack_mem = &AUTOINIT_taskBuffer[0],
  .stack_size = sizeof(AUTOINIT_taskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AUTODEINIT_task */
osThreadId_t AUTODEINIT_taskHandle;
uint32_t AUTODEINIT_taskBuffer[ 128 ];
osStaticThreadDef_t AUTODEINIT_taskControlBlock;
const osThreadAttr_t AUTODEINIT_task_attributes = {
  .name = "AUTODEINIT_task",
  .cb_mem = &AUTODEINIT_taskControlBlock,
  .cb_size = sizeof(AUTODEINIT_taskControlBlock),
  .stack_mem = &AUTODEINIT_taskBuffer[0],
  .stack_size = sizeof(AUTODEINIT_taskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Voltage_Monitor */
osThreadId_t Voltage_MonitorHandle;
uint32_t Voltage_MonitorBuffer[ 128 ];
osStaticThreadDef_t Voltage_MonitorControlBlock;
const osThreadAttr_t Voltage_Monitor_attributes = {
  .name = "Voltage_Monitor",
  .cb_mem = &Voltage_MonitorControlBlock,
  .cb_size = sizeof(Voltage_MonitorControlBlock),
  .stack_mem = &Voltage_MonitorBuffer[0],
  .stack_size = sizeof(Voltage_MonitorBuffer),
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for STOP_task */
osThreadId_t STOP_taskHandle;
uint32_t STOP_taskBuffer[ 128 ];
osStaticThreadDef_t STOP_taskControlBlock;
const osThreadAttr_t STOP_task_attributes = {
  .name = "STOP_task",
  .cb_mem = &STOP_taskControlBlock,
  .cb_size = sizeof(STOP_taskControlBlock),
  .stack_mem = &STOP_taskBuffer[0],
  .stack_size = sizeof(STOP_taskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Science_task */
osThreadId_t Science_taskHandle;
uint32_t Science_taskBuffer[ 128 ];
osStaticThreadDef_t Science_taskControlBlock;
const osThreadAttr_t Science_task_attributes = {
  .name = "Science_task",
  .cb_mem = &Science_taskControlBlock,
  .cb_size = sizeof(Science_taskControlBlock),
  .stack_mem = &Science_taskBuffer[0],
  .stack_size = sizeof(Science_taskBuffer),
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for Idle_task */
osThreadId_t Idle_taskHandle;
uint32_t Idle_taskBuffer[ 128 ];
osStaticThreadDef_t Idle_taskControlBlock;
const osThreadAttr_t Idle_task_attributes = {
  .name = "Idle_task",
  .cb_mem = &Idle_taskControlBlock,
  .cb_size = sizeof(Idle_taskControlBlock),
  .stack_mem = &Idle_taskBuffer[0],
  .stack_size = sizeof(Idle_taskBuffer),
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for Sync_task */
osThreadId_t Sync_taskHandle;
uint32_t Sync_taskBuffer[ 128 ];
osStaticThreadDef_t Sync_taskControlBlock;
const osThreadAttr_t Sync_task_attributes = {
  .name = "Sync_task",
  .cb_mem = &Sync_taskControlBlock,
  .cb_size = sizeof(Sync_taskControlBlock),
  .stack_mem = &Sync_taskBuffer[0],
  .stack_size = sizeof(Sync_taskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void PMT_init(void *argument);
void ERPA_init(void *argument);
void HK_init(void *argument);
void AUTOINIT_init(void *argument);
void AUTODEINIT_init(void *argument);
void Voltage_Monitor_init(void *argument);
void STOP_init(void *argument);
void Science_init(void *argument);
void Idle_init(void *argument);
void Sync_init(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 3 */
/**
 * @brief Tick hook function called by each tick interrupt.
 *		  Increments uptime_millis, used in time tagging.
 */
void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
	uptime_millis++;
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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

  /* creation of AUTOINIT_task */
  AUTOINIT_taskHandle = osThreadNew(AUTOINIT_init, NULL, &AUTOINIT_task_attributes);

  /* creation of AUTODEINIT_task */
  AUTODEINIT_taskHandle = osThreadNew(AUTODEINIT_init, NULL, &AUTODEINIT_task_attributes);

  /* creation of Voltage_Monitor */
  Voltage_MonitorHandle = osThreadNew(Voltage_Monitor_init, NULL, &Voltage_Monitor_attributes);

  /* creation of STOP_task */
  STOP_taskHandle = osThreadNew(STOP_init, NULL, &STOP_task_attributes);

  /* creation of Science_task */
  Science_taskHandle = osThreadNew(Science_init, NULL, &Science_task_attributes);

  /* creation of Idle_task */
  Idle_taskHandle = osThreadNew(Idle_init, NULL, &Idle_task_attributes);

  /* creation of Sync_task */
  Sync_taskHandle = osThreadNew(Sync_init, NULL, &Sync_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_PMT_init */
/**
 * @brief Initializes the PMT thread.
 *
 * This function waits for PMT event flags, creates a PMT packet,
 * and yields control to other threads in an infinite loop.
 */
/* USER CODE END Header_PMT_init */
void PMT_init(void *argument)
{
  /* USER CODE BEGIN PMT_init */
  /* Infinite loop */
	for (;;)
	{
		osEventFlagsWait(packet_event_flags, PMT_FLAG, osFlagsWaitAny, osWaitForever);

		create_pmt_packet();

		osThreadYield();
	}
  /* USER CODE END PMT_init */
}

/* USER CODE BEGIN Header_ERPA_init */
/**
 * @brief Initializes the ERPA thread.
 *
 * This function waits for ERPA event flags, creates an ERPA packet,
 * and yields control to other threads in an infinite loop.
 */
/* USER CODE END Header_ERPA_init */
void ERPA_init(void *argument)
{
  /* USER CODE BEGIN ERPA_init */
  /* Infinite loop */
	for (;;)
	{
		osEventFlagsWait(packet_event_flags, ERPA_FLAG, osFlagsWaitAny, osWaitForever);

		create_erpa_packet();

		osThreadYield();
	}
  /* USER CODE END ERPA_init */
}

/* USER CODE BEGIN Header_HK_init */
/**
 * @brief Initializes the HK thread.
 *
 * This function waits for HK event flags, creates an HK packet,
 * and yields control to other threads in an infinite loop.
 */
/* USER CODE END Header_HK_init */
void HK_init(void *argument)
{
  /* USER CODE BEGIN HK_init */
  /* Infinite loop */
	for (;;)
	{
		osEventFlagsWait(packet_event_flags, HK_FLAG, osFlagsWaitAny, osWaitForever);

		create_hk_packet();

		osThreadYield();
	}
  /* USER CODE END HK_init */
}

/* USER CODE BEGIN Header_AUTOINIT_init */
/**
 * @brief Initializes the AUTOINIT thread.
 *
 * This function waits for AUTOINIT event flags, enables all voltages
 * from SDN1 to 15V, updates the rail monitor, and yields control to
 * other threads in an infinite loop.
 */
/* USER CODE END Header_AUTOINIT_init */
void AUTOINIT_init(void *argument)
{
  /* USER CODE BEGIN AUTOINIT_init */
  /* Infinite loop */
	for (;;)
	{
		osEventFlagsWait(utility_event_flags, AUTOINIT_FLAG, osFlagsWaitAny, osWaitForever);

		// Enabling all voltages from SDN1 to 15V (inclusive)
		for (int i = GPIOS_INDEX_SDN1; i <= GPIOS_INDEX_15V; i++)
		{
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_SET);
			osDelay(PS_RAIL_DELAY);
		}

		// Telling rail monitor which rails are now enabled
		for (int i = RAIL_2v5; i <= RAIL_15v; i++)
		{
			set_rail_monitor_enable(i, ENABLED);
		}
		osThreadYield();
	}
  /* USER CODE END AUTOINIT_init */
}

/* USER CODE BEGIN Header_AUTODEINIT_init */
/**
 * @brief Initializes the AUTODEINIT thread.
 *
 * This function waits for AUTODEINIT event flags, disables all voltages
 * from 15V to SDN1, updates the rail monitor, and yields control to
 * other threads in an infinite loop.
 */
/* USER CODE END Header_AUTODEINIT_init */
void AUTODEINIT_init(void *argument)
{
  /* USER CODE BEGIN AUTODEINIT_init */
  /* Infinite loop */
	for (;;)
	{
		osEventFlagsWait(utility_event_flags, AUTODEINIT_FLAG, osFlagsWaitAny, osWaitForever);

		// Telling rail monitor which rails are now disabled
		for (int i = RAIL_15v; i >= RAIL_2v5; i--)
		{
			set_rail_monitor_enable(i, DISABLED);
		}

		// Disabling all voltages from 15V to SDN1 (inclusive)
		for (int i = GPIOS_INDEX_15V; i >= GPIOS_INDEX_SDN1; i--)
		{
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
			osDelay(PS_RAIL_DELAY);
		}

		osThreadYield();
	}
  /* USER CODE END AUTODEINIT_init */
}

/* USER CODE BEGIN Header_Voltage_Monitor_init */
/**
 * @brief Initializes the Voltage Monitor thread.
 *
 * This function waits for voltage monitor event flags, sets the rail monitor,
 * and checks rail status. If rails are out of bounds, it transitions to idle mode
 * before returning to science mode.
 */
/* USER CODE END Header_Voltage_Monitor_init */
void Voltage_Monitor_init(void *argument)
{
  /* USER CODE BEGIN Voltage_Monitor_init */
	uint8_t rails_in_bound = 1;
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(utility_event_flags, VOLTAGE_MONITOR_FLAG, osFlagsWaitAny, osWaitForever);
	  set_rail_monitor();
#ifdef ERROR_HANDLING_ENABLED
	  rails_in_bound = monitor_rails();
	  if (!rails_in_bound && !IDLING)
	  {
		  osEventFlagsSet(mode_event_flags, IDLE_FLAG);
		  while (!IDLING) {};
		  osDelay(IDLE_TO_SCIENCE_DELAY);
		  osEventFlagsSet(mode_event_flags, SCIENCE_FLAG);
	  }
#endif
  }
  /* USER CODE END Voltage_Monitor_init */
}

/* USER CODE BEGIN Header_STOP_init */
/**
 * @brief Initializes the STOP thread.
 *
 * This function waits for the STOP event flag, clears it, sets the IDLE flag,
 * and enters stop mode when idle.
 */
/* USER CODE END Header_STOP_init */
void STOP_init(void *argument)
{
  /* USER CODE BEGIN STOP_init */
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(utility_event_flags, STOP_FLAG, osFlagsWaitAny,osWaitForever);
	  osEventFlagsClear(utility_event_flags, STOP_FLAG);

	  osEventFlagsSet(mode_event_flags, IDLE_FLAG);
	  while (!IDLING) {};

	  enter_stop();
  }
  /* USER CODE END STOP_init */
}

/* USER CODE BEGIN Header_Science_init */
/**
 * @brief Initializes the Science thread.
 *
 * This function waits for the SCIENCE event flag, enables all voltages,
 * updates the rail monitor, starts DAC in DMA mode, and resets packet
 * sequence numbers, transitioning to the science mode.
 */
/* USER CODE END Header_Science_init */
void Science_init(void *argument)
{
  /* USER CODE BEGIN Science_init */

	/* Infinite loop */
  for(;;)
  {
		osEventFlagsWait(mode_event_flags, SCIENCE_FLAG, osFlagsWaitAny, osWaitForever);
		osThreadSuspend(Voltage_MonitorHandle);
		IDLING = 0;

		// Enabling all voltages
		for (int i = GPIOS_INDEX_SDN1; i <= GPIOS_INDEX_N800V; i++)
		{
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_SET);
			osDelay(PS_RAIL_DELAY);
		}

		// Telling rail monitor which voltages are now enabled
		for (int i = RAIL_busvmon; i <= RAIL_TMP1; i++)
		{
			set_rail_monitor_enable(i, ENABLED);
		}
		osThreadResume(Voltage_MonitorHandle);

		__disable_irq();
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, DAC_OUT, DAC_OUT_ARRAY_SIZE, DAC_ALIGN_12B_R);
		HK_ENABLED = ENABLED;
		ERPA_ENABLED = ENABLED;
		uptime_millis = 0;
		reset_packet_sequence_numbers();
		osEventFlagsSet(packet_event_flags, HK_FLAG);
		TIM2->CCR4 = ERPA_PWM_FREQ;
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
		__enable_irq();
		HAL_TIM_Base_Start_IT(&htim4);

		osThreadYield();
  }
  /* USER CODE END Science_init */
}

/* USER CODE BEGIN Header_Idle_init */
/**
 * @brief Initializes the Idle thread.
 *
 * This function waits for the IDLE event flag, disables ERPA and HK,
 * stops the DAC, updates the rail monitor, disables all voltages,
 * and yields control to other threads.
 */
/* USER CODE END Header_Idle_init */
void Idle_init(void *argument)
{
  /* USER CODE BEGIN Idle_init */

  /* Infinite loop */
  for(;;)
  {
		osEventFlagsWait(mode_event_flags, IDLE_FLAG, osFlagsWaitAny, osWaitForever);

		ERPA_ENABLED = DISABLED;
		TIM2->CCR4 = 0;
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		HK_ENABLED = DISABLED;
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		osThreadSuspend(Voltage_MonitorHandle);

		// Telling rail monitor which voltages are now disabled
		for (int i = RAIL_TMP1; i >= RAIL_busvmon; i--)
		{
			set_rail_monitor_enable(i, DISABLED);
		}

		// Disabling all voltages
		for (int i = GPIOS_INDEX_N800V; i >= GPIOS_INDEX_SDN1; i--)
		{
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
			osDelay(PS_RAIL_DELAY);
		}
		osDelay(IDLE_TO_VOLTAGE_MONITOR_DELAY);
		IDLING = 1;
		osThreadResume(Voltage_MonitorHandle);

		// Yield thread control
		osThreadYield();
  }
  /* USER CODE END Idle_init */
}

/* USER CODE BEGIN Header_Sync_init */
/**
 * @brief Initializes the Sync thread.
 *
 * This function waits for the SYNC event flag, sends an acknowledgment,
 * waits for a specific key value to be received, calibrates the RTC,
 * and handles UART communication for error counting and reset cause.
 */
/* USER CODE END Header_Sync_init */
void Sync_init(void *argument)
{
  /* USER CODE BEGIN Sync_init */
  /* Infinite loop */
  for(;;)
  {
	  	osEventFlagsWait(mode_event_flags, SYNC_FLAG, osFlagsWaitAny, osWaitForever);
	  	send_ACK();

	  	uint8_t key;
	  	uint8_t key_index = 0;
	  	uint8_t expected_key_value = 0xFF;
	  	uint8_t rtc_buffer_size = 9;
	  	ERROR_STRUCT reset_cause;

	  	// Wait for 0xFF to be received
	  	HAL_UART_AbortReceive(&huart1);
	  	do
	  	{
	  		HAL_UART_Receive(&huart1, UART_RX_BUFFER, rtc_buffer_size, UART_TIMEOUT_MS);
	  		key = UART_RX_BUFFER[key_index];
	  	} while (key != expected_key_value);

	  	calibrateRTC(UART_RX_BUFFER);
	  	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);
	  	osDelay(SYNC_DELAY);
	  	reset_cause = get_reset_cause();
	  	create_sync_packet(reset_cause);
  }
  /* USER CODE END Sync_init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

