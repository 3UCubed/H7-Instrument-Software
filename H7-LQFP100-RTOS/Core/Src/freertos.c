/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
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
/* Definitions for AUTOINIT_task */
osThreadId_t AUTOINIT_taskHandle;
const osThreadAttr_t AUTOINIT_task_attributes = {
  .name = "AUTOINIT_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AUTODEINIT_task */
osThreadId_t AUTODEINIT_taskHandle;
const osThreadAttr_t AUTODEINIT_task_attributes = {
  .name = "AUTODEINIT_task",
  .stack_size = 128 * 4,
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
/* Definitions for STOP_task */
osThreadId_t STOP_taskHandle;
const osThreadAttr_t STOP_task_attributes = {
  .name = "STOP_task",
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void PMT_init(void *argument);
void ERPA_init(void *argument);
void HK_init(void *argument);
void AUTOINIT_init(void *argument);
void AUTODEINIT_init(void *argument);
void UART_TX_init(void *argument);
void Voltage_Monitor_init(void *argument);
void STOP_init(void *argument);
void Science_init(void *argument);
void Idle_init(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationTickHook(void);

/* USER CODE BEGIN 3 */
void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

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

  /* creation of UART_TX_task */
  UART_TX_taskHandle = osThreadNew(UART_TX_init, NULL, &UART_TX_task_attributes);

  /* creation of Voltage_Monitor */
  Voltage_MonitorHandle = osThreadNew(Voltage_Monitor_init, NULL, &Voltage_Monitor_attributes);

  /* creation of STOP_task */
  STOP_taskHandle = osThreadNew(STOP_init, NULL, &STOP_task_attributes);

  /* creation of Science_task */
  Science_taskHandle = osThreadNew(Science_init, NULL, &Science_task_attributes);

  /* creation of Idle_task */
  Idle_taskHandle = osThreadNew(Idle_init, NULL, &Idle_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_PMT_init */
/**
  * @brief  Function implementing the PMT_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_PMT_init */
void PMT_init(void *argument)
{
  /* USER CODE BEGIN PMT_init */
  /* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END PMT_init */
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
		osDelay(1);
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
		osDelay(1);
	}
  /* USER CODE END HK_init */
}

/* USER CODE BEGIN Header_AUTOINIT_init */
/**
* @brief Function implementing the AUTOINIT_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AUTOINIT_init */
void AUTOINIT_init(void *argument)
{
  /* USER CODE BEGIN AUTOINIT_init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AUTOINIT_init */
}

/* USER CODE BEGIN Header_AUTODEINIT_init */
/**
* @brief Function implementing the AUTODEINIT_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AUTODEINIT_init */
void AUTODEINIT_init(void *argument)
{
  /* USER CODE BEGIN AUTODEINIT_init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AUTODEINIT_init */
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
	for (;;) {
		osDelay(1);
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
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END Voltage_Monitor_init */
}

/* USER CODE BEGIN Header_STOP_init */
/**
* @brief Function implementing the STOP_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_STOP_init */
void STOP_init(void *argument)
{
  /* USER CODE BEGIN STOP_init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END STOP_init */
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Idle_init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

