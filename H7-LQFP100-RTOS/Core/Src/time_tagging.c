/*
 ******************************************************************************
 * @file           : time_tagging.c
 * @author 		   : Jared Morrison
 * @date	 	   : October 9, 2024
 * @brief          : Implementation for all time tagging related functions
 ******************************************************************************
 */

#include "time_tagging.h"

HAL_StatusTypeDef RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);

/**
 * @brief Gets the system uptime and stores it in a buffer.
 *
 * @param buffer Pointer to a buffer where the uptime will be stored as four bytes.
 */
void get_uptime(uint8_t *buffer)
{
	uint32_t uptime = 0;
	uint32_t ms = uptime_millis;
	uint32_t st = SysTick->VAL;

	// Ensuring uptime_millis hasn't rolled over
	if (ms != uptime_millis)
	{
		ms = uptime_millis;
		st = SysTick->VAL;
	}
	uptime = ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);

	if (ms == 0)
	{
		uptime = 0;
	}

	buffer[0] = ((uptime >> 24) & 0xFF);
	buffer[1] = ((uptime >> 16) & 0xFF);
	buffer[2] = ((uptime >> 8) & 0xFF);
	buffer[3] = uptime & 0xFF;
}

/**
 * @brief Retrieves the current Unix time and milliseconds, storing them in a buffer.
 *
 * @param buffer Pointer to a buffer where the Unix time (4 bytes) and milliseconds (2 bytes) will be stored.
 */
void get_unix_time(uint8_t* buffer)
{
	#define UNIX_TIME_CONST   (719561U)
	#define SECONDS_IN_1_HOUR (3600U)
	#define SECONDS_IN_1_MIN  (60U)
	#define DAYS_IN_SECONDS   (24U * SECONDS_IN_1_HOUR)

	RTC_TimeTypeDef current_time;
	RTC_DateTypeDef current_date;

	HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &current_date, RTC_FORMAT_BIN);
	uint16_t milliseconds = (10000 - (current_time.SubSeconds)) / 10;

	uint16_t y;
	uint8_t m;
	uint8_t d;
	uint64_t unix_tm_val = 0;

	y = current_date.Year + 2000;
	m = current_date.Month;
	d = current_date.Date;
	// January and February are counted as months 13 and 14 of the previous year
	if (m <= 2)
	{
		m += 12;
		y -= 1;
	}
	// convert years to days
	unix_tm_val = (365 * y) + (y / 4) - (y / 100) + (y / 400);
	// convert months to days
	unix_tm_val += (30 * m) + (3 * (m + 1) / 5) + d;
	// Unix time starts on January 1st, 1970
	unix_tm_val -= UNIX_TIME_CONST;
	// convert days to seconds
	unix_tm_val *= DAYS_IN_SECONDS;
	//Add hours, minutes and seconds
	unix_tm_val += (SECONDS_IN_1_HOUR * current_time.Hours) + (SECONDS_IN_1_MIN * current_time.Minutes) + current_time.Seconds;

	buffer[0] = ((unix_tm_val >> 24) & 0xFF);
	buffer[1] = ((unix_tm_val >> 16) & 0xFF);
	buffer[2] = ((unix_tm_val >> 8) & 0xFF);
	buffer[3] = unix_tm_val & 0xFF;
	buffer[4] = ((milliseconds >> 8) & 0xFF);
	buffer[5] = milliseconds & 0xFF;
}

/**
 * @brief Calibrates the RTC with the specified date and time values.
 *
 * @param buffer Pointer to a buffer containing the date and time values in the following order:
 *               [0] = 0xFF (ignored)
 *               [1] = Year
 *               [2] = Month
 *               [3] = Day
 *               [4] = Hour
 *               [5] = Minute
 *               [6] = Second
 *               [7] = ms MSB
 *               [8] = ms LSB
 */
void calibrateRTC(uint8_t *buffer)
{
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
	if (status != HAL_OK)
	{
		Error_Handler();
	}
	RTC_SetTime(&hrtc, &time_struct, RTC_FORMAT_BIN);
}

/**
 * @brief Sets the time for the RTC.
 *
 * This function configures the time in the RTC peripheral. The time is set in 24-hour format.
 *
 * @param hrtc Pointer to a RTC_HandleTypeDef structure that contains
 *              the configuration information for the specified RTC.
 * @param sTime Pointer to a RTC_TimeTypeDef structure that contains
 *               the time values to be set.
 * @param Format Specifies the format of the time (24-hour or 12-hour).
 *
 * @retval HAL_StatusTypeDef HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
	uint32_t tmpreg;
	HAL_StatusTypeDef status;

	__HAL_LOCK(hrtc);

	hrtc->State = HAL_RTC_STATE_BUSY;

	__HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

	status = RTC_EnterInitMode(hrtc);
	if (status == HAL_OK)
	{
		sTime->TimeFormat = 0x00U;
		assert_param(IS_RTC_HOUR24(sTime->Hours));
		assert_param(IS_RTC_MINUTES(sTime->Minutes));
		assert_param(IS_RTC_SECONDS(sTime->Seconds));

		tmpreg = (uint32_t) (((uint32_t) RTC_ByteToBcd2(sTime->Hours)
				<< RTC_TR_HU_Pos)
				| ((uint32_t) RTC_ByteToBcd2(sTime->Minutes) << RTC_TR_MNU_Pos)
				| ((uint32_t) RTC_ByteToBcd2(sTime->Seconds) << RTC_TR_SU_Pos)
				| (((uint32_t) sTime->TimeFormat) << RTC_TR_PM_Pos));

		hrtc->Instance->TR = (uint32_t) (tmpreg & RTC_TR_RESERVED_MASK);

		status = RTC_ExitInitMode(hrtc);
	}

	__HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

	if (status == HAL_OK)
	{
		hrtc->State = HAL_RTC_STATE_READY;
	}

	__HAL_UNLOCK(hrtc);
	return status;
}




