/*
 * time.c
 *
 *  Created on: Aug 6, 2024
 *      Author: 3ucubed
 */

#include "time_tagging.h"

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

	if (ms == 0){
		uptime = 0;
	}

	buffer[0] = ((uptime >> 24) & 0xFF);
	buffer[1] = ((uptime >> 16) & 0xFF);
	buffer[2] = ((uptime >> 8) & 0xFF);
	buffer[3] = uptime & 0xFF;
}

void get_unix_time(uint8_t* buffer) {
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




