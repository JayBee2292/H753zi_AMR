#ifndef APP_MOTOR_SMOKE_H
#define APP_MOTOR_SMOKE_H

#include "app_robot.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 모터/엔코더 일괄 smoke test 상태를 초기화합니다.
 * @details
 * - PWM 출력을 0에서 시작하고 엔코더 측정 기준점을 재설정합니다.
 * - 이후 `app_motor_smoke_process()`를 주기적으로 호출하면 일정 duty로 모든 바퀴를 구동하면서
 *   엔코더 raw delta, 속도, 누적 tick을 UART로 확인할 수 있습니다.
 * @param robot 점검 대상 로봇 상태
 * @param debug_uart 상태 문자열 출력용 UART
 * @retval 초기화 성공 시 true
 */
bool app_motor_smoke_init(robot_status_t *robot, UART_HandleTypeDef *debug_uart);

/**
 * @brief smoke test 주기 작업을 한 번 수행합니다.
 * @param now_ms 현재 시스템 tick(ms 단위)
 */
void app_motor_smoke_process(uint32_t now_ms);

/**
 * @brief smoke test 권장 호출 주기(ms)를 반환합니다.
 * @retval 내부 엔코더/출력 갱신 주기
 */
uint32_t app_motor_smoke_get_period_ms(void);

#endif /* APP_MOTOR_SMOKE_H */
