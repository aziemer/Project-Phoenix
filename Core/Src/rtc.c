/**
 ******************************************************************************
 * @file    rtc.c
 * @brief   This file provides code for the configuration
 *          of the RTC instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "rtc.h"

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init( void )
{
	/** Initialize RTC Only */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
	if( HAL_RTC_Init( &hrtc ) != HAL_OK )
	{
		Error_Handler();
	}
}

void HAL_RTC_MspInit( RTC_HandleTypeDef *rtcHandle )
{
	if( rtcHandle->Instance == RTC )
	{
		HAL_PWR_EnableBkUpAccess();
		/* Enable BKP CLK enable for backup registers */
		__HAL_RCC_BKP_CLK_ENABLE( );
		/* RTC clock enable */
		__HAL_RCC_RTC_ENABLE( );
	}
}

void HAL_RTC_MspDeInit( RTC_HandleTypeDef *rtcHandle )
{
	if( rtcHandle->Instance == RTC )
	{
		/* Peripheral clock disable */
		__HAL_RCC_RTC_DISABLE( );
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
