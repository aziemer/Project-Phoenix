/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>

#include "tft.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void debug( char *x )
{
	TFT_clearScreen( RGB( 100, 20, 20 ) );
	TFT_setXPos( 10 );
	TFT_setYPos( 10 );
	TFT_setFont( FONT_16X24 );
	TFT_setFontSize( 1 );
	TFT_printf( "%s", x );
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
	debug( "NMI !!!\n");
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	debug( "HARD FAULT EXCEPTION !!!\n");
#if 0
	uint32_t CFSR = *(uint32_t*)0xE000ED28;
	if( CFSR & (1ul<<25) ) printf( " - Division by zero\n" );
	if( CFSR & (1ul<<24) ) printf( " - Unaligned access\n" );

	if( CFSR & (1ul<<19) ) printf( " - No coprocessor\n" );
	if( CFSR & (1ul<<18) ) printf( " - Invalid PC load\n" );
	if( CFSR & (1ul<<17) ) printf( " - Invalid EPSR state\n" );
	if( CFSR & (1ul<<16) ) printf( " - Undefined instruction\n" );

	if( CFSR & (1ul<<7) ) printf( " - Fault Address valid (%p)\n", (void*)*(uint32_t*)0xE000ED34 );
	if( CFSR & (1ul<<5) ) printf( " - Floating-point fault\n" );
	if( CFSR & (1ul<<4) ) printf( " - Stacking fault\n" );
	if( CFSR & (1ul<<3) ) printf( " - Unstacking fault\n" );
	if( CFSR & (1ul<<1) ) printf( " - Data access violation\n" );
	if( CFSR & (1ul<<0) ) printf( " - Instruction access violation\n" );


	if( CFSR & (0xFFul<<8) ) printf( " - BCFSR Bus Fault\n" );

	printf( "CCR = 0x%08lX (Bit4=DIV0, Bit3=UNALIGNED)\n", *(uint32_t*)0xE000ED14 );
#endif
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
	debug( "MEM MANAGER EXCEPTION !!!\n");
#if 0
	uint32_t CFSR = *(uint32_t*)0xE000ED28;
	if( CFSR & (1ul<<25) ) printf( " - Division by zero\n" );
	if( CFSR & (1ul<<24) ) printf( " - Unaligned access\n" );

	if( CFSR & (1ul<<19) ) printf( " - No coprocessor\n" );
	if( CFSR & (1ul<<18) ) printf( " - Invalid PC load\n" );
	if( CFSR & (1ul<<17) ) printf( " - Invalid EPSR state\n" );
	if( CFSR & (1ul<<16) ) printf( " - Undefined instruction\n" );

	if( CFSR & (1ul<<7) ) printf( " - Fault Address valid (%p)\n", (void*)*(uint32_t*)0xE000ED34 );
	if( CFSR & (1ul<<5) ) printf( " - Floating-point fault\n" );
	if( CFSR & (1ul<<4) ) printf( " - Stacking fault\n" );
	if( CFSR & (1ul<<3) ) printf( " - Unstacking fault\n" );
	if( CFSR & (1ul<<1) ) printf( " - Data access violation\n" );
	if( CFSR & (1ul<<0) ) printf( " - Instruction access violation\n" );

	if( CFSR & (0xFFul<<8) ) printf( " - BCFSR Bus Fault\n" );

	printf( "CCR = 0x%08lX (Bit4=DIV0, Bit3=UNALIGNED)\n", *(uint32_t*)0xE000ED14 );
#endif
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
	debug( "BUS FAULT EXCEPTION !!!\n");
#if 0
	uint32_t CFSR = *(uint32_t*)0xE000ED28;
	if( CFSR & (1ul<<25) ) printf( " - Division by zero\n" );
	if( CFSR & (1ul<<24) ) printf( " - Unaligned access\n" );

	if( CFSR & (1ul<<19) ) printf( " - No coprocessor\n" );
	if( CFSR & (1ul<<18) ) printf( " - Invalid PC load\n" );
	if( CFSR & (1ul<<17) ) printf( " - Invalid EPSR state\n" );
	if( CFSR & (1ul<<16) ) printf( " - Undefined instruction\n" );

	if( CFSR & (1ul<<7) ) printf( " - Fault Address valid (%p)\n", (void*)*(uint32_t*)0xE000ED34 );
	if( CFSR & (1ul<<5) ) printf( " - Floating-point fault\n" );
	if( CFSR & (1ul<<4) ) printf( " - Stacking fault\n" );
	if( CFSR & (1ul<<3) ) printf( " - Unstacking fault\n" );
	if( CFSR & (1ul<<1) ) printf( " - Data access violation\n" );
	if( CFSR & (1ul<<0) ) printf( " - Instruction access violation\n" );

	if( CFSR & (0xFFul<<8) ) printf( " - BCFSR Bus Fault\n" );

	printf( "CCR = 0x%08lX (Bit4=DIV0, Bit3=UNALIGNED)\n", *(uint32_t*)0xE000ED14 );
#endif
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
	debug( "USAGE FAULT EXCEPTION !!!\n");
#if 0
	uint32_t CFSR = *(uint32_t*)0xE000ED28;

	if( CFSR & (1ul<<25) ) printf( " - Division by zero\n" );
	if( CFSR & (1ul<<24) ) printf( " - Unaligned access\n" );

	if( CFSR & (1ul<<19) ) printf( " - No coprocessor\n" );
	if( CFSR & (1ul<<18) ) printf( " - Invalid PC load\n" );
	if( CFSR & (1ul<<17) ) printf( " - Invalid EPSR state\n" );
	if( CFSR & (1ul<<16) ) printf( " - Undefined instruction\n" );

	if( CFSR & (1ul<<7) ) printf( " - Fault Address valid (%p)\n", (void*)*(uint32_t*)0xE000ED34 );
	if( CFSR & (1ul<<5) ) printf( " - Floating-point fault\n" );
	if( CFSR & (1ul<<4) ) printf( " - Stacking fault\n" );
	if( CFSR & (1ul<<3) ) printf( " - Unstacking fault\n" );
	if( CFSR & (1ul<<1) ) printf( " - Data access violation\n" );
	if( CFSR & (1ul<<0) ) printf( " - Instruction access violation\n" );

	if( CFSR & (0xFFul<<8) ) printf( " - BCFSR Bus Fault\n" );

	printf( "CCR = 0x%08lX (Bit4=DIV0, Bit3=UNALIGNED)\n", *(uint32_t*)0xE000ED14 );
#endif
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
