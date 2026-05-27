#ifndef APP_UART_SMOKE_H
#define APP_UART_SMOKE_H

#include "app_robot.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t duty_percent;
    int32_t left_duty_percent;
    int32_t right_duty_percent;
    int32_t curve_ratio_percent;
    uint32_t last_motion_command_ms;
    uint8_t motion;
    bool is_motion_active;
} app_uart_smoke_control_state_t;

/**
 * @brief UART 기반 원격 주행 제어를 초기화합니다.
 * @details
 * - UART로 Xbox/키보드 제어 패킷을 받아 좌/우 목표 속도를 갱신합니다.
 * - 엔코더 속도 피드백을 PID로 보정하여 좌/우 PWM duty를 출력합니다.
 * - 주행 상태 데이터는 CAN telemetry로 송신하며, 시리얼 모니터 확인용 UART 텍스트도 주기 출력합니다.
 * @param robot 제어 대상 로봇 상태
 * @param command_uart 명령 수신에 사용할 UART
 * @retval 초기화 성공 시 true
 */
bool app_uart_smoke_init(robot_status_t *robot, UART_HandleTypeDef *command_uart);

/**
 * @brief UART 원격 주행 제어 주기 작업을 수행합니다.
 * @param now_ms 현재 시스템 tick(ms)
 */
void app_uart_smoke_process(uint32_t now_ms);

/**
 * @brief 마지막 UART 주행 명령과 PID 출력 duty/motion 상태를 반환합니다.
 * @param state 상태를 저장할 포인터
 */
void app_uart_smoke_get_control_state(app_uart_smoke_control_state_t *state);

/**
 * @brief UART 원격 주행 제어의 권장 호출 주기(ms)를 반환합니다.
 * @retval 내부 갱신 주기(ms)
 */
uint32_t app_uart_smoke_get_period_ms(void);

#endif /* APP_UART_SMOKE_H */
