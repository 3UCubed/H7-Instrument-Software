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
#include "iwdg.h"
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define FLIGHT_MODE

#define ACK 0xFF
#define NACK 0x00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
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

uint32_t DAC_OUT[32] = { 0, 0, 620, 620, 1241, 1241, 1861, 1861, 2482, 2482,
		3103, 3103, 3723, 3723, 4095, 4095, 4095, 4095, 3723, 3723, 3103, 3103,
		2482, 2482, 1861, 1861, 1241, 1241, 620, 620, 0, 0 }; // For 3.3 volts

osEventFlagsId_t packet_event_flags;
osEventFlagsId_t utility_event_flags;
osEventFlagsId_t mode_event_flags;

unsigned char UART_RX_BUFFER[UART_RX_BUFFER_SIZE];
volatile uint8_t HK_ENABLED = 0;
volatile uint8_t ERPA_ENABLED = 0;
volatile uint8_t step = 0;
volatile uint32_t cadence = 3125;
volatile uint32_t uptime_millis = 0;
volatile uint8_t tx_flag = 1;
volatile uint8_t HK_10_second_counter = 0;

volatile uint8_t HK_100_ms_counter = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void get_reset_cause();
void system_setup();
void send_ACK();
void send_NACK();
void sync();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
		osEventFlagsSet(packet_event_flags, PMT_FLAG_ID);
	}
	else if (htim == &htim2) {
		if (ERPA_ENABLED) {
			osEventFlagsSet(packet_event_flags, ERPA_FLAG_ID);
		}
		if (HK_100_ms_counter == 32) {
			osEventFlagsSet(utility_event_flags, VOLTAGE_MONITOR_FLAG_ID);

			if (HK_ENABLED){
				osEventFlagsSet(packet_event_flags, HK_FLAG_ID);
			}
			HK_100_ms_counter = 0;
		}
		HK_100_ms_counter++;

	} else {
		printf("Unknown Timer Interrupt\n");
	}
}


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
		set_rail_monitor_enable(RAIL_2v5, 1);
		break;
	}
	case 0x01: {
		printf("SYS OFF PB5\n");

		// Turning off all voltage enables (including high voltages) in order from highest to lowest, including SYS_ON
		for (int i = 8; i > 0; i--) {
			HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin, GPIO_PIN_RESET);
		}

		for (int i = RAIL_n800v; i >= RAIL_2v5; i--) {
			set_rail_monitor_enable(i, 0);
		}

		break;
	}
	case 0x12: {
		printf("3v3 ON PC10\n");
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_3v3, 1);

		break;
	}
	case 0x02: {
		printf("3v3 OFF PC10\n");
		HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_3v3, 0);
		break;
	}
	case 0x13: {
		printf("5v ON PC7\n");
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_5v, 1);

		break;
	}
	case 0x03: {
		printf("5v OFF PC7\n");
		HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_5v, 0);
		break;
	}
	case 0x14: {
		printf("n3v3 ON PC6\n");
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_n3v3, 1);

		break;
	}
	case 0x04: {
		printf("n3v3 OFF PC6\n");
		HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_n3v3, 0);
		break;
	}
	case 0x15: {
		printf("n5v ON PC8\n");
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_n5v, 1);

		break;
	}
	case 0x05: {
		printf("n5v OFF PC8\n");
		HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_n5v, 0);
		break;
	}
	case 0x16: {
		printf("15v ON PC9\n");
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_15v, 1);

		break;
	}
	case 0x06: {
		printf("15v OFF PC9\n");
		HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_15v, 0);
		break;
	}
	case 0x17: {
		printf("n200v ON PC13\n");
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_n200v, 1);

		break;
	}
	case 0x07: {
		printf("n200v OFF PC13\n");
		HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_n200v, 0);
		break;
	}
	case 0x18: {
		printf("800v ON PB6\n");
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_SET);
		set_rail_monitor_enable(RAIL_n800v, 1);

		break;
	}
	case 0x08: {
		printf("800v OFF PB6\n");
		HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_RESET);
		set_rail_monitor_enable(RAIL_n800v, 0);
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
		osEventFlagsSet(packet_event_flags, ERPA_FLAG_ID);
		TIM2->CCR4 = 312;
		ERPA_ENABLED = 1;

		break;
	}
	case 0x0A: {
		printf("ERPA OFF\n");
		ERPA_ENABLED = 0;
		TIM2->CCR4 = 0;
		break;
	}
	case 0x1B: {
		printf("PMT ON\n");
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
		osEventFlagsSet(packet_event_flags, PMT_FLAG_ID);
		break;
	}
	case 0x0B: {
		printf("PMT OFF\n");
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		break;
	}
	case 0x1C: {
		printf("HK ON \n");
		osEventFlagsSet(packet_event_flags, HK_FLAG_ID);
		HK_ENABLED = 1;
		break;
	}
	case 0x0C: {
		printf("HK OFF\n");
		HK_ENABLED = 0;
		break;
	}
	case 0x1D: {
		printf("Step Up\n");
		if (step < 14) {
			step += 2;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
					DAC_OUT[step]);
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		}
		break;
	}
	case 0x0D: {
		printf("Step Down\n");
		if (step > 1) {
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
		osEventFlagsSet(utility_event_flags, STOP_FLAG);
		break;
	}
	case 0xE0: {
		printf("Auto Init\n");
		osEventFlagsSet(utility_event_flags, AUTOINIT_FLAG);
		break;
	}
	case 0xD0: {
		printf("Auto Deinit\n");
		osEventFlagsSet(utility_event_flags, AUTODEINIT_FLAG);
		break;
	}
	case 0xAF: {
		sync();
		break;
	}
	case 0xBF: {
		osEventFlagsSet(mode_event_flags, SCIENCE_FLAG);
		break;
	}
	case 0xCF: {
		osEventFlagsSet(mode_event_flags, IDLE_FLAG);
		break;
	}
	case 0xDF: {
		reset_error_counters();
		break;
	}
	default: {
		printf("Unknown Command\n");
		break;
	}
	}
}

void get_reset_cause()
{
	ERROR_STRUCT error;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST))
    {
        error.category = EC_watchdog;
        error.detail = ED_UNDEFINED;
        handle_error(error);
    }
    // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to
    // ensure first that the reset cause is NOT a POR/PDR reset. See note
    // below.
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
    {
        error.category = EC_brownout;
        error.detail = ED_UNDEFINED;
        handle_error(error);
    }

    // Clear all the reset flags or else they will remain set during future
    // resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();
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
 // MX_IWDG1_Init();
  /* USER CODE BEGIN 2 */
  //get_reset_cause();
  system_setup();

  ERROR_STRUCT error;
  error.category = EC_watchdog;
  error.detail = ED_UNDEFINED;
  handle_error(error);

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
void system_setup() {
	// 1 -- Init event flags
	// 3 -- Init rail monitor
	// 4 -- Start timer 3
	// 5 -- Set timer 2 PWM
	// 6 -- Init ADC DMA
	// 7 -- Start UART receive interrupts
  	error_counter_init();


	packet_event_flags = osEventFlagsNew(NULL);
    if (packet_event_flags == NULL) {
        while (1);
    }

    utility_event_flags = osEventFlagsNew(NULL);
    if (utility_event_flags == NULL) {
        while (1);
    }

    mode_event_flags = osEventFlagsNew(NULL);
    if (mode_event_flags == NULL) {
        while (1);
    }

    TIM2->CCR4 = 0;
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);

	if (!voltage_monitor_init()) {
		while (1);
	}

	if (!init_adc_dma()) {
		while (1);
	}

	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);

}

void sync() {
	send_ACK();

	uint8_t key;

	// Wait for 0xFF to be received
	HAL_UART_AbortReceive(&huart1);
	do {
		HAL_UART_Receive(&huart1, UART_RX_BUFFER, 9, 100);
		key = UART_RX_BUFFER[0];
	} while (key != 0xFF);

	calibrateRTC(UART_RX_BUFFER); // TODO: calibrate rtc
	HAL_UART_Receive_IT(&huart1, UART_RX_BUFFER, 1);
	send_previous_error_packet();
}

void send_ACK() {
	static uint8_t tx_buffer[1];

	tx_buffer[0] = ACK;
	HAL_UART_Transmit(&huart1, tx_buffer, 1, 100);
}

void send_NACK() {
	static uint8_t tx_buffer[1];

	tx_buffer[0] = NACK;
	HAL_UART_Transmit(&huart1, tx_buffer, 1, 100);

}

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

void enter_stop() {
	  send_ACK();

	  vTaskSuspendAll();
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

		// When MCU is triggered to wake up, it resumes right here.
		// That's why it looks like we enter stop mode and then instantly
		// configure the clock and resume tasks, but in reality the MCU
		// just stops right here.
	  NVIC_SystemReset();
	  SystemClock_Config();
	  reset_packet_sequence_numbers();
	  xTaskResumeAll();
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
