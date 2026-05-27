#ifndef APP_MOTOR_DRIVE_H
#define APP_MOTOR_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_MOTOR_DRIVE_WHEEL_FL = 0,
    APP_MOTOR_DRIVE_WHEEL_FR,
    APP_MOTOR_DRIVE_WHEEL_RL,
    APP_MOTOR_DRIVE_WHEEL_RR,
    APP_MOTOR_DRIVE_WHEEL_COUNT
} app_motor_drive_wheel_t;

/**
 * @brief TIM1 PWM과 DIR GPIO를 초기화하고 모든 출력을 0으로 맞춥니다.
 * @details
 * - 실제 바퀴와 TIM1/DIR 핀 매핑은 이 모듈 한 곳에만 모아 둡니다.
 * - 같은 초기화 함수를 smoke test, UART open-loop test가 공통으로 사용합니다.
 * @retval PWM 시작까지 성공하면 true
 */
bool app_motor_drive_init(void);

/**
 * @brief 모든 바퀴 출력을 즉시 0으로 만듭니다.
 */
void app_motor_drive_stop(void);

/**
 * @brief 개별 바퀴 duty를 설정합니다.
 * @param wheel 설정할 바퀴 인덱스
 * @param duty_percent 차량 전진 기준 duty(-100 ~ 100)
 */
void app_motor_drive_set_wheel_output(
    app_motor_drive_wheel_t wheel,
    int32_t duty_percent);

/**
 * @brief 바퀴별 duty를 한 번에 설정합니다.
 * @param fl_duty_percent Front Left duty(-100 ~ 100)
 * @param fr_duty_percent Front Right duty(-100 ~ 100)
 * @param rl_duty_percent Rear Left duty(-100 ~ 100)
 * @param rr_duty_percent Rear Right duty(-100 ~ 100)
 */
void app_motor_drive_set_wheel_outputs(
    int32_t fl_duty_percent,
    int32_t fr_duty_percent,
    int32_t rl_duty_percent,
    int32_t rr_duty_percent);

/**
 * @brief 좌/우 duty를 4륜에 좌우 대칭으로 적용합니다.
 * @param left_duty_percent 좌측 두 바퀴 duty(-100 ~ 100)
 * @param right_duty_percent 우측 두 바퀴 duty(-100 ~ 100)
 */
void app_motor_drive_set_left_right_output(
    int32_t left_duty_percent,
    int32_t right_duty_percent);

/**
 * @brief 모든 바퀴에 같은 duty를 적용합니다.
 * @param duty_percent 전체 바퀴 공통 duty(-100 ~ 100)
 */
void app_motor_drive_set_all_output(int32_t duty_percent);

#endif /* APP_MOTOR_DRIVE_H */
