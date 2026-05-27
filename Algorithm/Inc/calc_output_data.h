#ifndef CALC_OUTPUT_DATA_H
#define CALC_OUTPUT_DATA_H

#include <stdint.h>
#include "calc_input_data.h"

/**
 * @brief 로봇 역기구학 목표 휠 속도 구조체
 * @details
 * - PID 제어기 또는 Open-loop 피드포워드 제어 시 각 바퀴가 도달해야 할 목표 선속도(m/s)를 담습니다.
 */
typedef struct {
    float fl_velocity_mps; /* Front Left 휠 목표 선속도 (m/s) */
    float fr_velocity_mps; /* Front Right 휠 목표 선속도 (m/s) */
    float rl_velocity_mps; /* Rear Left 휠 목표 선속도 (m/s) */
    float rr_velocity_mps; /* Rear Right 휠 목표 선속도 (m/s) */
} target_wheel_velocities_t;

/**
 * @brief 목표 차체 속도를 바탕으로 4륜 스키드 스티어 로봇의 역기구학을 계산합니다.
 * @param target_linear_v 목표 차체 전진 선속도 (m/s)
 * @param target_angular_w 목표 차체 요(Yaw) 회전 각속도 (rad/s)
 * @param target_wheels 계산된 각 휠의 목표 선속도를 저장할 구조체 포인터
 */
void calculate_inverse_kinematics(float target_linear_v, float target_angular_w, target_wheel_velocities_t *target_wheels);

#endif // CALC_OUTPUT_DATA_H
