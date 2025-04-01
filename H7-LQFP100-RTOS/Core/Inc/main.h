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
#include "cmsis_os.h"
#include <stdio.h>
#include "string.h"

#include "packet_creation.h"
#include "voltage_monitor.h"
#include "sample_data.h"
#include "time_tagging.h"

#define V_MAJOR 4
#define V_MINOR 3
#define V_PATCH 0

#define ERROR_HANDLING_ENABLED

#define ERPA_PWM_FREQ 312

#define UART_RX_BUFFER_SIZE 64
#define DAC_OUT_ARRAY_SIZE 32


#define PMT_FLAG 0x0001
#define ERPA_FLAG 0x0002
#define HK_FLAG 0x0004

#define VOLTAGE_MONITOR_FLAG 0x0001
#define STOP_FLAG 0x0002
#define AUTOINIT_FLAG 0x0004
#define AUTODEINIT_FLAG 0x0008

#define SCIENCE_FLAG 0x0001
#define IDLE_FLAG 0x0002
#define SYNC_FLAG 0x0004

#define ENABLED 1
#define DISABLED 0

#define UART_TIMEOUT_MS 100

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct
{
	GPIO_TypeDef *gpio;
	uint16_t pin;
} gpio_pins;

typedef enum
{
    STEP_0 = 0,
    STEP_1 = 1,
    STEP_2 = 2,
    STEP_3 = 3,
    STEP_4 = 4,
    STEP_5 = 5,
    STEP_6 = 6,
    STEP_7 = 7,
    INVALID_STEP = 255
} STEP_VALUES;

enum
{
	GPIOS_INDEX_SDN1 = 0,
	GPIOS_INDEX_SYS = 1,
	GPIOS_INDEX_5V = 2,
	GPIOS_INDEX_N3V3 = 3,
	GPIOS_INDEX_N5V = 4,
	GPIOS_INDEX_15V = 5,
	GPIOS_INDEX_N200V = 6,
	GPIOS_INDEX_N800V = 7
};

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
extern volatile uint8_t ERPA_ENABLED;
extern volatile uint8_t IDLING;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
STEP_VALUES get_current_step();
void enter_stop();
void send_ACK();
void send_NACK();
ERROR_STRUCT get_reset_cause();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
