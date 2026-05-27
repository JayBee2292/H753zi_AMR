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
#include "app_motor_smoke.h"
#include "app_robot.h"
#include "mros_executor.h"
#include "mros_publisher.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 1 enables the all-wheel smoke/debug mode that drives motors and prints status over UART3. */
#define UART3_SMOKE_TEST 1
/* micro-ROS init/publish path is significantly deeper than the smoke loop.
 * Keep 16 KiB here because smaller stacks caused INVPC-like fault symptoms during bring-up. */
#define DEFAULT_TASK_STACK_SIZE_BYTES (4096U * sizeof(StackType_t))
/* Toggle green once every N successful publishes so the heartbeat is visible to the eye. */
#define MROS_PUBLISH_LED_DIVIDER 10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
/* Default task owns the top-level mode loop: all-wheel smoke/debug or micro-ROS publish. */
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

static void freertos_show_microros_waiting(void)
{
   /* Normal micro-ROS path uses green only.
    * Yellow/red are reserved for actual fault indication after the bring-up debug phase. */
   freertos_set_leds(true, false, false);
}

static void freertos_show_microros_publish_ok(bool toggle_heartbeat)
{
   if (toggle_heartbeat) {
      BSP_LED_Toggle(LED_GREEN);
   } else {
      BSP_LED_On(LED_GREEN);
   }

   /* Green only: at least one publish succeeded and the loop is alive. */
   BSP_LED_Off(LED_YELLOW);
   BSP_LED_Off(LED_RED);
}

static void freertos_show_microros_fault(void)
{
   /* Red steady: initialization or publish failed and the task is about to stop. */
   freertos_set_leds(false, false, true);
}

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
 * [알고리즘 설명]
  * - UART3_SMOKE_TEST:
  *   1. 4륜 전체를 같은 차량 진행 방향 기준으로 개루프(Open-loop) 구동합니다.
  *   2. 엔코더 raw delta, 절대 tick, 선속도, 각속도를 UART3로 출력합니다.
  *   3. 모터/엔코더/방향 핀/부호 설정이 올바른지 빠르게 점검하는 smoke/debug 모드입니다.
  * - 일반 모드(micro-ROS 모드):
  *   1. micro-ROS Executor와 Publisher를 초기화하여 ROS2 통신 환경을 구성합니다.
  *   2. 25ms 절대 주기로 제어 루프를 실행하며, 실제 경과 tick으로 dt를 계산합니다.
  *   3. `app_robot_update()`를 호출하여 엔코더 값 기반 현재 선속도 및 각속도(Kinematics)를 계산합니다.
  *   4. 계산된 로봇의 Twist(V, W) 상태를 micro-ROS(DDS)를 통해 상위 제어기(Host)로 퍼블리시(Publish)합니다.
  *   5. 정상 상태 LED는 초록 heartbeat만 사용하고, 노랑/빨강은 fault 전용으로 남깁니다.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void) argument;

#if UART3_SMOKE_TEST
  if (!app_motor_smoke_init(app_robot_get_state(), &huart3)) {
      Error_Handler();
  }

  for (;;) {
      app_motor_smoke_process(osKernelGetTickCount());
      osDelay(app_motor_smoke_get_period_ms());
  }
#else
  const uint32_t publish_period_ms = 25U; /* 목표 publish 주기(ms) */
  const uint32_t tick_freq_hz = osKernelGetTickFreq(); /* RTOS tick frequency(Hz) */
  TickType_t publish_period_ticks = pdMS_TO_TICKS(publish_period_ms); /* 목표 주기의 RTOS tick 표현 */
  mros_executor_context_t mros_executor = {0}; /* micro-ROS node/executor 수명주기 컨텍스트 */
  mros_publisher_context_t odom_publisher = {0}; /* odom_vel Twist 퍼블리셔 상태 */
  robot_status_t *robot = app_robot_get_state(); /* 공유 로봇 상태 저장소 */
  uint32_t publish_success_count = 0U; /* 사람 눈에 보이는 heartbeat 분주용 성공 카운터 */
  UBaseType_t stack_high_water_mark = uxTaskGetStackHighWaterMark(NULL);

  g_default_task_stack_low_water_mark_words = stack_high_water_mark;

  freertos_show_microros_waiting();

  if (tick_freq_hz == 0U) {
      freertos_show_microros_fault();
      Error_Handler();
  }

  if (publish_period_ticks == 0U) {
      publish_period_ticks = 1U;
  }

  if (!mros_executor_init(
      &mros_executor,
      &huart3,
      MROS_EXECUTOR_DEFAULT_NODE_NAME)) {
      freertos_show_microros_fault();
      Error_Handler();
  }

  if (!mros_publisher_init_twist(
      &odom_publisher,
      &mros_executor,
      MROS_PUBLISHER_DEFAULT_TWIST_TOPIC)) {
      freertos_show_microros_fault();
      Error_Handler();
  }

  app_robot_reset_measurements(robot);

  TickType_t last_wake_tick = xTaskGetTickCount(); /* vTaskDelayUntil 기준 시각 */
  TickType_t last_sample_tick = last_wake_tick;    /* dt 계산용 직전 샘플 시각 */

  for (;;) {
      /* Use an absolute wake time so the control/publish loop does not drift. */
      vTaskDelayUntil(&last_wake_tick, publish_period_ticks);

      TickType_t now_tick = xTaskGetTickCount();
      TickType_t elapsed_ticks = now_tick - last_sample_tick;
      float dt = (float) elapsed_ticks / (float) tick_freq_hz;
      if (dt <= 0.0f) {
          dt = (float) publish_period_ticks / (float) tick_freq_hz;
      }
      last_sample_tick = now_tick;

      stack_high_water_mark = uxTaskGetStackHighWaterMark(NULL);
      if (stack_high_water_mark < g_default_task_stack_low_water_mark_words) {
          g_default_task_stack_low_water_mark_words = stack_high_water_mark;
      }

      /* Keep the task loop thin: sample robot state, then publish it. */
      app_robot_update(robot, dt);

      if (!mros_publisher_publish_twist(
              &odom_publisher,
              robot->robot_linear_v,
              robot->robot_angular_w)) {
          freertos_show_microros_fault();
          Error_Handler();
      }

      ++publish_success_count;
      freertos_show_microros_publish_ok(
          (publish_success_count == 1U) ||
          ((publish_success_count % MROS_PUBLISH_LED_DIVIDER) == 0U));
  }
#endif
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

