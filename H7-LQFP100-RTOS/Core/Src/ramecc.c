/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : ramecc.c
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Implementation for initialization and handling of RAM ECC
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
#include "ramecc.h"

/* USER CODE BEGIN 0 */
#define ITCM_START_ADDRESS 0x00000000
#define ITCM_END_ADDRESS   0x0000FFFF

#define DTCM_START_ADDRESS 0x20000000
#define DTCM_END_ADDRESS 0x2001FFFF

void enable_ramecc_monitor_notifications(RAMECC_HandleTypeDef *hramecc);
void write_RAM(volatile uint32_t *start, volatile uint32_t *end);
/* USER CODE END 0 */

RAMECC_HandleTypeDef hramecc1_m1;
RAMECC_HandleTypeDef hramecc1_m2;
RAMECC_HandleTypeDef hramecc1_m3;
RAMECC_HandleTypeDef hramecc1_m4;
RAMECC_HandleTypeDef hramecc1_m5;
RAMECC_HandleTypeDef hramecc2_m1;
RAMECC_HandleTypeDef hramecc2_m2;
RAMECC_HandleTypeDef hramecc2_m3;
RAMECC_HandleTypeDef hramecc2_m4;
RAMECC_HandleTypeDef hramecc2_m5;
RAMECC_HandleTypeDef hramecc3_m1;
RAMECC_HandleTypeDef hramecc3_m2;

/* RAMECC init function */
/**
 * @brief Initializes the RAMECC (RAM Error Correction Code) monitors.
 *
 * This function sets up the RAMECC for various RAM regions, ensuring that ECC errors
 * are monitored and handled. It includes writing to the ITCM and DTCM on startup
 * to avoid triggering errors.
 */
void MX_RAMECC_Init(void)
{
	/* USER CODE BEGIN RAMECC_Init 0 */
	// ITCM and DTCM will trigger ECC error if not written to on startup
	write_RAM((volatile uint32_t*) ITCM_START_ADDRESS, (volatile uint32_t*) ITCM_END_ADDRESS);
	write_RAM((volatile uint32_t*) DTCM_START_ADDRESS, (volatile uint32_t*) DTCM_END_ADDRESS);
	/* USER CODE END RAMECC_Init 0 */

	/* USER CODE BEGIN RAMECC_Init 1 */

	/* USER CODE END RAMECC_Init 1 */

	/** Initialize RAMECC1 M1 : AXI SRAM
	 */
	hramecc1_m1.Instance = RAMECC1_Monitor1;
	if (HAL_RAMECC_Init(&hramecc1_m1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC1 M2 : ITCM-RAM
	 */
	hramecc1_m2.Instance = RAMECC1_Monitor2;
	if (HAL_RAMECC_Init(&hramecc1_m2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC1 M3 : D0TCM-RAM
	 */
	hramecc1_m3.Instance = RAMECC1_Monitor3;
	if (HAL_RAMECC_Init(&hramecc1_m3) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC1 M4 : D1TCM-RAM
	 */
	hramecc1_m4.Instance = RAMECC1_Monitor4;
	if (HAL_RAMECC_Init(&hramecc1_m4) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC1 M5 : ETM RAM
	 */
	hramecc1_m5.Instance = RAMECC1_Monitor5;
	if (HAL_RAMECC_Init(&hramecc1_m5) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC2 M1 : SRAM1_0
	 */
	hramecc2_m1.Instance = RAMECC2_Monitor1;
	if (HAL_RAMECC_Init(&hramecc2_m1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC2 M2 SRAM1_1
	 */
	hramecc2_m2.Instance = RAMECC2_Monitor2;
	if (HAL_RAMECC_Init(&hramecc2_m2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC2 M3 : SRAM2_0
	 */
	hramecc2_m3.Instance = RAMECC2_Monitor3;
	if (HAL_RAMECC_Init(&hramecc2_m3) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC2 M4 : SRAM2_1
	 */
	hramecc2_m4.Instance = RAMECC2_Monitor4;
	if (HAL_RAMECC_Init(&hramecc2_m4) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC2 M5 : SRAM3
	 */
	hramecc2_m5.Instance = RAMECC2_Monitor5;
	if (HAL_RAMECC_Init(&hramecc2_m5) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC3 M1 : SRAM4
	 */
	hramecc3_m1.Instance = RAMECC3_Monitor1;
	if (HAL_RAMECC_Init(&hramecc3_m1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initialize RAMECC3 M2 : Backup RAM
	 */
	hramecc3_m2.Instance = RAMECC3_Monitor2;
	if (HAL_RAMECC_Init(&hramecc3_m2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RAMECC_Init 2 */
	HAL_NVIC_SetPriority(ECC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ECC_IRQn);

	enable_ramecc_monitor_notifications(&hramecc1_m1);
	enable_ramecc_monitor_notifications(&hramecc1_m2);
	enable_ramecc_monitor_notifications(&hramecc1_m3);
	enable_ramecc_monitor_notifications(&hramecc1_m4);
	enable_ramecc_monitor_notifications(&hramecc1_m5);
	enable_ramecc_monitor_notifications(&hramecc2_m1);
	enable_ramecc_monitor_notifications(&hramecc2_m2);
	enable_ramecc_monitor_notifications(&hramecc2_m3);
	enable_ramecc_monitor_notifications(&hramecc2_m4);
	enable_ramecc_monitor_notifications(&hramecc2_m5);
	enable_ramecc_monitor_notifications(&hramecc3_m1);
	enable_ramecc_monitor_notifications(&hramecc3_m2);
	/* USER CODE END RAMECC_Init 2 */
}

/* USER CODE BEGIN 1 */
/**
 * @brief Writes zeros to a specified range of RAM.
 *
 * @param start Pointer to the starting address of the RAM range.
 * @param end Pointer to the ending address of the RAM range.
 */
void write_RAM(volatile uint32_t *start, volatile uint32_t *end)
{
	while (start <= end)
	{
		*start = 0;
		start++;
	}
}

/**
 * @brief Enables notifications for RAMECC monitoring.
 *
 * @param hramecc Pointer to the RAMECC handle.
 */
void enable_ramecc_monitor_notifications(RAMECC_HandleTypeDef *hramecc)
{
	if (HAL_RAMECC_EnableNotification(hramecc, (RAMECC_IT_MONITOR_SINGLEERR_R | RAMECC_IT_MONITOR_DOUBLEERR_R)) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_RAMECC_StartMonitor(hramecc) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Callback function for RAMECC error detection.
 *
 * This function handles single and double bit errors detected by RAMECC.
 *
 * @param hramecc Pointer to the RAMECC handle.
 */
void HAL_RAMECC_DetectErrorCallback(RAMECC_HandleTypeDef *hramecc)
{
	ERROR_STRUCT error;
	error.category = EC_seu;
	if ((HAL_RAMECC_GetRAMECCError(hramecc) & HAL_RAMECC_SINGLEERROR_DETECTED) != 0U)
	{
		error.detail = ED_single_bit_error_ram;
	}

	if ((HAL_RAMECC_GetRAMECCError(hramecc) & HAL_RAMECC_DOUBLEERROR_DETECTED) != 0U)
	{
		error.detail = ED_double_bit_error_ram;
	}
	handle_error(error);
}

/* USER CODE END 1 */
