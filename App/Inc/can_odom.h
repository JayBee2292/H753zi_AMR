#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * CAN 양방향 odometry / cmd_vel 전송 모듈
 *
 * 프레임 포맷 (CAN FD + BRS, DLC=8, little-endian float):
 *   0x100  STM32 → Jetson  [float linear_v (m/s), float angular_w (rad/s)]
 *   0x200  Jetson → STM32  [float linear_v (m/s), float angular_w (rad/s)]
 */

#define CAN_ODOM_TX_ID             0x100U  /* odometry  : STM32 → Jetson */
#define CAN_ODOM_RX_ID             0x200U  /* cmd_vel   : Jetson → STM32 */
#define CAN_ODOM_CMD_TIMEOUT_MS    500U    /* 수신 없을 경우 정지 판정 시간 (ms) */
#define CAN_ODOM_MAX_WHEEL_VEL_MPS 1.0f   /* duty 100% 에 해당하는 바퀴 선속도 (m/s) */

/**
 * @brief FDCAN1 필터 설정, 인터럽트 활성화, 버스 시작
 * @return true  초기화 성공
 * @return false HAL 오류
 */
bool can_odom_init(void);

/**
 * @brief odometry 프레임을 CAN 버스로 전송
 * @param linear_v  로봇 선속도 (m/s)
 * @param angular_w 로봇 각속도 (rad/s)
 * @return true  전송 성공
 * @return false TX FIFO 여유 없음 또는 HAL 오류
 */
bool can_odom_tx_odom(float linear_v, float angular_w);

/**
 * @brief 가장 최근에 수신된 cmd_vel 반환
 * @details now_ms 기준으로 CAN_ODOM_CMD_TIMEOUT_MS 이상 수신이 없으면 0, 0 반환
 * @param linear_v  [out] 목표 선속도 (m/s)
 * @param angular_w [out] 목표 각속도 (rad/s)
 * @param now_ms    현재 시각 (osKernelGetTickCount 등)
 */
void can_odom_get_cmd_vel(float *linear_v, float *angular_w, uint32_t now_ms);
