#include "calc_output_data.h"

/**
 * @brief 로봇 역기구학(Inverse Kinematics) 기반 목표 바퀴 속도 계산
 * @details
 * [알고리즘 설명]
 * - 스키드 스티어(Skid-steer) 방식 4륜 구동 로봇의 역방향 기구학(Inverse Kinematics) 공식을 적용합니다.
 * - 목표 차체 선속도(V)와 각속도(W)를 입력받아 좌/우 트랙이 내야 할 목표 선속도(V_left, V_right)를 분리합니다.
 * - V_left  = V - (W * EFFECTIVE_TRACK_WIDTH_M / 2.0f)
 * - V_right = V + (W * EFFECTIVE_TRACK_WIDTH_M / 2.0f)
 * - 4륜 모델이므로 좌측 전/후방(FL, RL)은 V_left, 우측 전/후방(FR, RR)은 V_right 목표 속도를 동일하게 할당합니다.
 * - 산출된 휠 목표 선속도(m/s)는 이후 PID 제어기(Closed-loop)의 목표값(Setpoint)으로 사용되거나,
 *   Open-loop 제어의 Feedforward 값(직접 PWM 매핑)으로 활용될 수 있습니다.
 *
 * [회전 방향 기준]
 * - ROS 표준(Right Hand Rule)을 따릅니다: Z축이 위를 향하므로, W가 양수이면 로봇은 반시계(좌회전)를 합니다.
 * - 따라서 W가 양수일 때 V_right가 V_left보다 더 빨라져야(혹은 전진해야) 로봇이 좌회전하게 됩니다.
 */
void calculate_inverse_kinematics(float target_linear_v, float target_angular_w, target_wheel_velocities_t *target_wheels) {
    if (target_wheels == NULL) {
        return;
    }

    // 회전에 필요한 좌우 바퀴 간의 선속도 차이의 절반을 구합니다.
    float angular_component = (target_angular_w * EFFECTIVE_TRACK_WIDTH_M) / 2.0f;

    // 좌우측 목표 선속도(V_left, V_right) 계산
    float v_left = target_linear_v - angular_component;
    float v_right = target_linear_v + angular_component;

    // 4륜 스키드 스티어이므로, 좌측 트랙 두 바퀴와 우측 트랙 두 바퀴에 각각 동일한 목표 속도를 할당합니다.
    target_wheels->fl_velocity_mps = v_left;
    target_wheels->rl_velocity_mps = v_left;

    target_wheels->fr_velocity_mps = v_right;
    target_wheels->rr_velocity_mps = v_right;
}
