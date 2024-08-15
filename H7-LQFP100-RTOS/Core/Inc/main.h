/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"			// For freeRTOS commands
#include <stdio.h>				// For uint data types
#include "string.h"				// For memcpy

#include "packet_creation.h"	// For resetting packet sequence numbers
#include "voltage_monitor.h"	// For initializing voltage monitor in system_setup
#include "sample_data.h"		// For initializing adc dma in system_setup
#include "time_tagging.h"		// For letting sync calibrate RTC

#define UART_RX_BUFFER_SIZE 64


#define PMT_FLAG_ID 0x0001
#define ERPA_FLAG_ID 0x0002
#define HK_FLAG_ID 0x0004

#define VOLTAGE_MONITOR_FLAG_ID 0x0001
#define STOP_FLAG 0x0002
#define AUTOINIT_FLAG 0x0004
#define AUTODEINIT_FLAG 0x0008

#define SCIENCE_FLAG 0x0001
#define IDLE_FLAG 0x0002



/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
	GPIO_TypeDef *gpio;
	uint16_t pin;
} gpio_pins;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern osEventFlagsId_t packet_event_flags;
extern osEventFlagsId_t utility_event_flags;
extern osEventFlagsId_t mode_event_flags;

extern unsigned char UART_RX_BUFFER[UART_RX_BUFFER_SIZE];
extern const gpio_pins gpios[];
extern volatile uint32_t uptime_millis;
extern uint32_t DAC_OUT[32];
extern volatile uint8_t HK_ENABLED;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint8_t get_current_step();
void enter_stop();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
