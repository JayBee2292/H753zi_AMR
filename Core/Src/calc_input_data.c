//
// Created by jongbeom on 26. 4. 2..
//

#include "calc_input_data.h"

void encoder_overflow_callback(robot_status_t *robot, TIM_HandleTypeDef *htim) {
    if (robot == NULL || htim == NULL) {
        return;
    }

    // 4개의 바퀴 중 현재 인터럽트가 발생한 타이머를 찾습니다.
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (robot->wheels[i].htim == htim) {
            // 방향 확인: DIR 비트가 1이면 DOWN 카운팅 (언더플로우), 0이면 UP 카운팅 (오버플로우)
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
                robot->wheels[i].overflow_cnt--;
            } else {
                robot->wheels[i].overflow_cnt++;
            }
            break;
        }
    }
}

int64_t get_total_encoder_tick(wheel_state_t *wheel) {
    if (wheel == NULL || wheel->htim == NULL) return 0;
    
    // 오버플로우 인터럽트와 메인 루프가 겹쳐서 값이 튀는 현상(Race Condition)을 방지하기 위해 
    // 값을 읽는 찰나의 순간에만 글로벌 인터럽트를 비활성화합니다. (Critical Section)
    uint32_t primask_bit = __get_PRIMASK(); // 현재 인터럽트 상태 저장
    __disable_irq();                        // 인터럽트 발생 차단

    uint32_t current_cnt = __HAL_TIM_GET_COUNTER(wheel->htim);
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(wheel->htim);
    int32_t current_overflow = wheel->overflow_cnt;

    // 만약 이 찰나의 순간에 이미 오버플로우가 하드웨어적으로 발생해버렸다면 (인터럽트는 막혀있지만 플래그는 뜸)
    // 플래그를 확인하여 수동으로 한 번 보정해줍니다.
    if (__HAL_TIM_GET_FLAG(wheel->htim, TIM_FLAG_UPDATE) != RESET) {
        // 방향에 따라 오버플로우/언더플로우 보정
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(wheel->htim)) {
            current_overflow--;
        } else {
            current_overflow++;
        }
        // 카운터도 방금 넘어간 최신 값으로 다시 읽습니다.
        current_cnt = __HAL_TIM_GET_COUNTER(wheel->htim);
    }

    __set_PRIMASK(primask_bit);             // 인터럽트 상태 원래대로 복구 (Critical Section 끝)

    // 총 틱 수 계산
    int64_t total_ticks = ((int64_t)current_overflow * (int64_t)(arr + 1)) + current_cnt;
    return total_ticks;
}

void wheel_update_velocity(wheel_state_t *wheel, float dt) {
    if (wheel == NULL) {
        return;
    }

    if (dt <= 0.0f) {
        wheel->velocity_mps = 0.0f;
        return;
    }

    // 오버플로우를 고려한 전체 64/32비트 틱 카운트 읽기
    wheel->total_tick = get_total_encoder_tick(wheel);

    // 델타 틱 계산 (이제 완전히 선형적으로 증가/감소하는 값을 사용하므로 오버플로우 걱정이 없음)
    int64_t delta_tick = wheel->total_tick - wheel->prev_tick;

    // 왼쪽/오른쪽 바퀴의 물리적인 장착 대칭 구조로 인한 부호 반전 처리
    if (wheel->is_inverted_direction) {
        delta_tick = -delta_tick;
    }

    if (wheel->ppr > 0) {
        // CMSIS-DSP의 PI 상수 사용
        float distance = ((float)delta_tick / (float)wheel->ppr) * PI * wheel->diameter_m;
        
        wheel->velocity_mps = distance / dt;
        wheel->total_distance_m += distance;
    } else {
        wheel->velocity_mps = 0.0f;
    }

    wheel->prev_tick = wheel->total_tick;
}

void calculate_robot_kinematics(robot_status_t *robot) {
    if (robot == NULL) {
        return;
    }

    // 4륜 스키드 스티어(Skid-steer) 또는 차동 구동(Differential drive) 로봇 가정
    // 인덱스 매핑 가정 (사용 환경에 맞게 수정 가능):
    // 0: Front Left (FL)
    // 1: Front Right (FR)
    // 2: Rear Left (RL)
    // 3: Rear Right (RR)
    
    float v_fl = robot->wheels[0].velocity_mps;
    float v_fr = robot->wheels[1].velocity_mps;
    float v_rl = robot->wheels[2].velocity_mps;
    float v_rr = robot->wheels[3].velocity_mps;

    // 좌우측 평균 선속도 계산
    float v_left = (v_fl + v_rl) / 2.0f;
    float v_right = (v_fr + v_rr) / 2.0f;

    // 로봇 전체 전진 선속도 (v, m/s): 좌우측 속도의 평균
    robot->robot_linear_v = (v_left + v_right) / 2.0f;

    // 로봇 전체 회전 각속도 (w, rad/s): (우측 속도 - 좌측 속도) / 윤거(ROBOT_WIDTH_M)
    if (ROBOT_WIDTH_M > 0.0f) {
        robot->robot_angular_w = (v_right - v_left) / ROBOT_WIDTH_M;
    } else {
        robot->robot_angular_w = 0.0f;
    }
}

#include <stdio.h>
#include <string.h>

void debug_print_robot_state(const robot_status_t *robot, UART_HandleTypeDef *huart) {
    if (robot == NULL || huart == NULL) {
        return;
    }
    
    char buf[128];
    // 선속도(V), 각속도(W), 그리고 1번 바퀴(Front Left)의 누적 이동 거리를 출력합니다.
    snprintf(buf, sizeof(buf), "V: %.3f m/s, W: %.3f rad/s, Dist(FL): %.3f m\r\n",
             robot->robot_linear_v,
             robot->robot_angular_w,
             robot->wheels[0].total_distance_m);
             
    HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), 10);
}
