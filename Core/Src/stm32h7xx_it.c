/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_it.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* Latched fault registers for debugger inspection after the core stops in a fault handler. */
volatile uint32_t g_fault_hfsr = 0U;
volatile uint32_t g_fault_cfsr = 0U;
volatile uint32_t g_fault_bfar = 0U;
volatile uint32_t g_fault_mmar = 0U;

static void fault_set_leds(bool green_on, bool yellow_on, bool red_on)
{
  if (green_on) {
    BSP_LED_On(LED_GREEN);
  } else {
    BSP_LED_Off(LED_GREEN);
  }

  if (yellow_on) {
    BSP_LED_On(LED_YELLOW);
  } else {
    BSP_LED_Off(LED_YELLOW);
  }

  if (red_on) {
    BSP_LED_On(LED_RED);
  } else {
    BSP_LED_Off(LED_RED);
  }
}

static void fault_busy_delay(volatile uint32_t cycles)
{
  while (cycles > 0U) {
    __NOP();
    --cycles;
  }
}

static void fault_latch_registers(void)
{
  g_fault_hfsr = SCB->HFSR;
  g_fault_cfsr = SCB->CFSR;
  g_fault_bfar = SCB->BFAR;
  g_fault_mmar = SCB->MMFAR;
}

static uint32_t usage_fault_get_code(uint32_t cfsr)
{
  uint32_t usage_bits = cfsr & 0xFFFF0000UL;

  if (usage_bits == 0U) {
    return 7U;
  }

  if ((usage_bits & (usage_bits - 1U)) != 0U) {
    /* Multiple UsageFault bits set at once. */
    return 7U;
  }

  if ((cfsr & SCB_CFSR_UNDEFINSTR_Msk) != 0U) {
    return 1U;
  }

  if ((cfsr & SCB_CFSR_INVSTATE_Msk) != 0U) {
    return 2U;
  }

  if ((cfsr & SCB_CFSR_INVPC_Msk) != 0U) {
    return 3U;
  }

  if ((cfsr & SCB_CFSR_NOCP_Msk) != 0U) {
    return 4U;
  }

  if ((cfsr & SCB_CFSR_UNALIGNED_Msk) != 0U) {
    return 5U;
  }

  if ((cfsr & SCB_CFSR_DIVBYZERO_Msk) != 0U) {
    return 6U;
  }

  return 7U;
}

static void usage_fault_set_subtype_leds(uint32_t code)
{
  switch (code) {
    case 1U:
      /* UNDEFINSTR */
      fault_set_leds(true, false, false);
      break;

    case 2U:
      /* INVSTATE */
      fault_set_leds(false, true, false);
      break;

    case 3U:
      /* INVPC */
      fault_set_leds(false, false, true);
      break;

    case 4U:
      /* NOCP */
      fault_set_leds(true, true, false);
      break;

    case 5U:
      /* UNALIGNED */
      fault_set_leds(true, false, true);
      break;

    case 6U:
      /* DIVBYZERO */
      fault_set_leds(false, true, true);
      break;

    case 7U:
    default:
      /* Multiple bits set or unknown usage-fault subtype */
      fault_set_leds(true, true, true);
      break;
  }
}

static void usage_fault_signal_loop(uint32_t cfsr)
{
  uint32_t code = usage_fault_get_code(cfsr);
  /* Keep the pattern slow and deterministic enough to read by eye without a debugger. */
  uint32_t sync_delay_cycles = SystemCoreClock / 4U;
  uint32_t hold_delay_cycles = SystemCoreClock / 2U;

  if (sync_delay_cycles == 0U) {
    sync_delay_cycles = 1U;
  }

  if (hold_delay_cycles == 0U) {
    hold_delay_cycles = 1U;
  }

  while (1) {
    /* Frame sync: all LEDs on briefly so the following held color is the actual subtype code. */
    fault_set_leds(true, true, true);
    fault_busy_delay(sync_delay_cycles);

    fault_set_leds(false, false, false);
    fault_busy_delay(sync_delay_cycles);

    usage_fault_set_subtype_leds(code);
    fault_busy_delay(hold_delay_cycles);

    fault_set_leds(false, false, false);
    fault_busy_delay(hold_delay_cycles);
  }
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim1_up;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim6;

extern FDCAN_HandleTypeDef hfdcan1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

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
  fault_latch_registers();
  /* Red + yellow: hard fault */
  fault_set_leds(false, true, true);

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
  fault_latch_registers();
  /* Green + red: memory management fault */
  fault_set_leds(true, false, true);

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  fault_latch_registers();
  /* All LEDs on: bus fault */
  fault_set_leds(true, true, true);

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
  fault_latch_registers();
  usage_fault_signal_loop(g_fault_cfsr);

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
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

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_up);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  BSP_PB_IRQHandler(BUTTON_USER);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles FDCAN1 interrupt 0 (RxFifo0 new message).
  */
void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

/* USER CODE END 1 */
