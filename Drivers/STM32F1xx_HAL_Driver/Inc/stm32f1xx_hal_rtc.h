/**
  ******************************************************************************
  * @file    stm32f1xx_hal_rtc.h
  * @author  MCD Application Team
  * @brief   Header file of RTC HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1xx_HAL_RTC_H
#define __STM32F1xx_HAL_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal_def.h"

#define IS_RTC_ASYNCH_PREDIV(PREDIV)  (((PREDIV) <= 0xFFFFFU) || ((PREDIV) == RTC_AUTO_1_SECOND))
#define IS_RTC_HOUR24(HOUR)           ((HOUR) <= 23U)
#define IS_RTC_MINUTES(MINUTES)       ((MINUTES) <= 59U)
#define IS_RTC_SECONDS(SECONDS)       ((SECONDS) <= 59U)
#define IS_RTC_FORMAT(FORMAT)         (((FORMAT) == RTC_FORMAT_BIN) || ((FORMAT) == RTC_FORMAT_BCD))
#define IS_RTC_YEAR(YEAR)             ((YEAR) <= 99U)
#define IS_RTC_MONTH(MONTH)           (((MONTH) >= 1U) && ((MONTH) <= 12U))
#define IS_RTC_DATE(DATE)             (((DATE) >= 1U) && ((DATE) <= 31U))
#define IS_RTC_ALARM(ALARM)           ((ALARM) == RTC_ALARM_A)
#define IS_RTC_CALIB_OUTPUT(__OUTPUT__) (((__OUTPUT__) == RTC_OUTPUTSOURCE_NONE) || \
                                         ((__OUTPUT__) == RTC_OUTPUTSOURCE_CALIBCLOCK) || \
                                         ((__OUTPUT__) == RTC_OUTPUTSOURCE_ALARM) || \
                                         ((__OUTPUT__) == RTC_OUTPUTSOURCE_SECOND))

#define RTC_TIMEOUT_VALUE           1000U

#define RTC_EXTI_LINE_ALARM_EVENT   ((uint32_t)EXTI_IMR_MR17)  /*!< External interrupt line 17 Connected to the RTC Alarm event */

/**
  * RTC Time structure definition
  */
typedef struct
{
  uint8_t Hours;            /*!< Specifies the RTC Time Hour.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 23 */

  uint8_t Minutes;          /*!< Specifies the RTC Time Minutes.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t Seconds;          /*!< Specifies the RTC Time Seconds.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

} RTC_TimeTypeDef;

/**
  * HAL State structures definition
  */
typedef enum
{
  HAL_RTC_STATE_RESET             = 0x00U,  /*!< RTC not yet initialized or disabled */
  HAL_RTC_STATE_READY             = 0x01U,  /*!< RTC initialized and ready for use   */
  HAL_RTC_STATE_BUSY              = 0x02U,  /*!< RTC process is ongoing              */
  HAL_RTC_STATE_TIMEOUT           = 0x03U,  /*!< RTC timeout state                   */
  HAL_RTC_STATE_ERROR             = 0x04U   /*!< RTC error state                     */

} HAL_RTCStateTypeDef;

/**
  * RTC Configuration Structure definition
  */
typedef struct
{
  uint32_t AsynchPrediv;    /*!< Specifies the RTC Asynchronous Predivider value.
                                 This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFFFFF  or RTC_AUTO_1_SECOND
                                 If RTC_AUTO_1_SECOND is selected, AsynchPrediv will be set automatically to get 1sec timebase */

  uint32_t OutPut;          /*!< Specifies which signal will be routed to the RTC Tamper pin.
                                 This parameter can be a value of @ref RTC_output_source_to_output_on_the_Tamper_pin */

} RTC_InitTypeDef;

/**
  * RTC Date structure definition
  */
typedef struct
{
  uint8_t WeekDay;  /*!< Specifies the RTC Date WeekDay (not necessary for HAL_RTC_SetDate).
                         This parameter can be a value of @ref RTC_WeekDay_Definitions */

  uint8_t Month;    /*!< Specifies the RTC Date Month (in BCD format).
                         This parameter can be a value of @ref RTC_Month_Date_Definitions */

  uint8_t Date;     /*!< Specifies the RTC Date.
                         This parameter must be a number between Min_Data = 1 and Max_Data = 31 */

  uint8_t Year;     /*!< Specifies the RTC Date Year.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 99 */

} RTC_DateTypeDef;

/**
  * Time Handle Structure definition
  */
typedef struct
{
  RTC_TypeDef                 *Instance;  /*!< Register base address    */

  RTC_InitTypeDef             Init;       /*!< RTC required parameters  */

  RTC_DateTypeDef             DateToUpdate;       /*!< Current date set by user and updated automatically  */

  HAL_LockTypeDef             Lock;       /*!< RTC locking object       */

  __IO HAL_RTCStateTypeDef    State;      /*!< Time communication state */
} RTC_HandleTypeDef;

/**
  * RTC_Automatic_Prediv_1_Second Automatic calculation of prediv for 1sec timebase
  */
#define RTC_AUTO_1_SECOND                      0xFFFFFFFFU

/**
  * RTC_Input_parameter_format_definitions Input Parameter Format
  */
#define RTC_FORMAT_BIN                         0x000000000U
#define RTC_FORMAT_BCD                         0x000000001U

/**
  * RTC_Month_Date_Definitions Month Definitions
  */

/* Coded in BCD format */
#define RTC_MONTH_JANUARY              ((uint8_t)0x01)
#define RTC_MONTH_FEBRUARY             ((uint8_t)0x02)
#define RTC_MONTH_MARCH                ((uint8_t)0x03)
#define RTC_MONTH_APRIL                ((uint8_t)0x04)
#define RTC_MONTH_MAY                  ((uint8_t)0x05)
#define RTC_MONTH_JUNE                 ((uint8_t)0x06)
#define RTC_MONTH_JULY                 ((uint8_t)0x07)
#define RTC_MONTH_AUGUST               ((uint8_t)0x08)
#define RTC_MONTH_SEPTEMBER            ((uint8_t)0x09)
#define RTC_MONTH_OCTOBER              ((uint8_t)0x10)
#define RTC_MONTH_NOVEMBER             ((uint8_t)0x11)
#define RTC_MONTH_DECEMBER             ((uint8_t)0x12)

/**
  * RTC_WeekDay_Definitions WeekDay Definitions
  */
#define RTC_WEEKDAY_MONDAY             ((uint8_t)0x01)
#define RTC_WEEKDAY_TUESDAY            ((uint8_t)0x02)
#define RTC_WEEKDAY_WEDNESDAY          ((uint8_t)0x03)
#define RTC_WEEKDAY_THURSDAY           ((uint8_t)0x04)
#define RTC_WEEKDAY_FRIDAY             ((uint8_t)0x05)
#define RTC_WEEKDAY_SATURDAY           ((uint8_t)0x06)
#define RTC_WEEKDAY_SUNDAY             ((uint8_t)0x00)

/**
  * RTC_output_source_to_output_on_the_Tamper_pin Output source to output on the Tamper pin
  */
#define RTC_OUTPUTSOURCE_NONE               0x00000000U                       /*!< No output on the TAMPER pin  */
#define RTC_OUTPUTSOURCE_CALIBCLOCK         BKP_RTCCR_CCO                     /*!< RTC clock with a frequency divided by 64 on the TAMPER pin  */
#define RTC_OUTPUTSOURCE_ALARM              BKP_RTCCR_ASOE                    /*!< Alarm pulse signal on the TAMPER pin  */
#define RTC_OUTPUTSOURCE_SECOND             (BKP_RTCCR_ASOS | BKP_RTCCR_ASOE) /*!< Second pulse signal on the TAMPER pin  */

/**
  * RTC_Interrupts_Definitions Interrupts Definitions
  */
#define RTC_IT_OW            RTC_CRH_OWIE       /*!< Overflow interrupt */
#define RTC_IT_ALRA          RTC_CRH_ALRIE      /*!< Alarm interrupt */
#define RTC_IT_SEC           RTC_CRH_SECIE      /*!< Second interrupt */
#define RTC_IT_TAMP1         BKP_CSR_TPIE       /*!< TAMPER Pin interrupt enable */

/**
  * RTC_Flags_Definitions Flags Definitions
  */
#define RTC_FLAG_RTOFF       RTC_CRL_RTOFF      /*!< RTC Operation OFF flag */
#define RTC_FLAG_RSF         RTC_CRL_RSF        /*!< Registers Synchronized flag */
#define RTC_FLAG_OW          RTC_CRL_OWF        /*!< Overflow flag */
#define RTC_FLAG_ALRAF       RTC_CRL_ALRF       /*!< Alarm flag */
#define RTC_FLAG_SEC         RTC_CRL_SECF       /*!< Second flag */
#define RTC_FLAG_TAMP1F      BKP_CSR_TEF        /*!< Tamper Interrupt Flag */

/**
  * Reset RTC handle state
  */
#define __HAL_RTC_RESET_HANDLE_STATE(__HANDLE__)			  ((__HANDLE__)->State = HAL_RTC_STATE_RESET)

/**
  * Disable the write protection for RTC registers.
  */
#define __HAL_RTC_WRITEPROTECTION_DISABLE(__HANDLE__)         SET_BIT((__HANDLE__)->Instance->CRL, RTC_CRL_CNF)

/**
  * Enable the write protection for RTC registers.
  */
#define __HAL_RTC_WRITEPROTECTION_ENABLE(__HANDLE__)          CLEAR_BIT((__HANDLE__)->Instance->CRL, RTC_CRL_CNF)

/* Include RTC HAL Extension module */
#include "stm32f1xx_hal_rtc_ex.h"

/* Initialization and de-initialization functions  ****************************/
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);

/* RTC Time and Date functions ************************************************/
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);

/* Peripheral State functions *************************************************/
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);

/* Peripheral Control functions ***********************************************/
HAL_StatusTypeDef   HAL_RTC_WaitForSynchro(RTC_HandleTypeDef *hrtc);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_RTC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
