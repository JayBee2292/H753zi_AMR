/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_uart_smoke.h"
#include "app_robot.h"
#include "can_telemetry.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* UART remote control + CAN FD telemetry runtime. */
#define DEFAULT_TASK_STACK_SIZE_BYTES (4096U * sizeof(StackType_t))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = DEFAULT_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);

/* USER CODE BEGIN 4 */
/* Lowest high-water mark observed for the default task, in FreeRTOS stack words.
 * Left in place because it was useful when tracking the earlier micro-ROS fault. */
volatile UBaseType_t g_default_task_stack_low_water_mark_words = 0U;

static void freertos_set_leds(bool green_on, bool yellow_on, bool red_on)
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

/* Unused micro-ROS LEDs functions removed */

static void freertos_stack_fault_loop(void)
{
   volatile uint32_t delay_cycles = 4000000U;

   taskDISABLE_INTERRUPTS();

   for (;;) {
      freertos_set_leds(false, false, true);
      for (volatile uint32_t i = 0U; i < delay_cycles; ++i) {
         __NOP();
      }

      freertos_set_leds(false, false, false);
      for (volatile uint32_t i = 0U; i < delay_cycles; ++i) {
         __NOP();
      }
   }
}

/**
  * @brief FreeRTOS stack overflow hook
  * @details
  * - `configCHECK_FOR_STACK_OVERFLOW`가 활성화된 경우 커널이 이 함수를 호출합니다.
  * - 현재 프로젝트에서는 별도 복구 없이 브레이크포인트/디버깅용 훅으로만 남겨둡니다.
  * @param xTask 오버플로우가 감지된 태스크 핸들
  * @param pcTaskName 오버플로우가 감지된 태스크 이름
  */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   (void) xTask;
   (void) pcTaskName;
   freertos_stack_fault_loop();
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  메인 제어 루프(Default Task)
 * @details
 * - UART3에서 Xbox/키보드 원격 주행 패킷을 수신해 좌/우 모터 duty를 갱신합니다.
 * - 같은 루프에서 엔코더 기반 차체/바퀴 속도를 갱신합니다.
 * - CAN FD+BRS telemetry 프레임으로 PC에 주행 데이터를 송신합니다.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void) argument;

  robot_status_t *robot = app_robot_get_state();

  if (!app_uart_smoke_init(robot, &huart3)) {
      freertos_set_leds(false, false, true);
      Error_Handler();
  }

  if (!can_telemetry_init()) {
      freertos_set_leds(false, false, true);
      Error_Handler();
  }

  freertos_set_leds(true, false, false); /* 초록: 통합 런타임 정상 기동 */

  const uint32_t period_ms = app_uart_smoke_get_period_ms();
  TickType_t last_wake_tick = xTaskGetTickCount();

  for (;;) {
      uint32_t now_ms = (uint32_t) osKernelGetTickCount();
      TickType_t next_wake_tick = last_wake_tick + pdMS_TO_TICKS(period_ms);
      int32_t wait_ticks = (int32_t) (next_wake_tick - now_ms);
      app_uart_smoke_control_state_t uart_state = {0};
      can_telemetry_drive_state_t drive_state = {0};
      uint32_t flags;

      if (wait_ticks < 0) {
          wait_ticks = 0;
      }

      flags = osThreadFlagsWait(0x01, osFlagsWaitAny, wait_ticks);

      now_ms = (uint32_t) osKernelGetTickCount();
      app_uart_smoke_process(now_ms);
      app_uart_smoke_get_control_state(&uart_state);

      drive_state.duty_percent = uart_state.duty_percent;
      drive_state.left_duty_percent = uart_state.left_duty_percent;
      drive_state.right_duty_percent = uart_state.right_duty_percent;
      drive_state.curve_ratio_percent = uart_state.curve_ratio_percent;
      drive_state.motion = uart_state.motion;
      drive_state.is_motion_active = uart_state.is_motion_active;

      can_telemetry_process(robot, &drive_state, now_ms);

      if (flags == osFlagsErrorTimeout ||
          (int32_t) (next_wake_tick - osKernelGetTickCount()) <= 0) {
          last_wake_tick = next_wake_tick;
      }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
