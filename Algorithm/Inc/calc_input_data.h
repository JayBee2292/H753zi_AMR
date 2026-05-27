//
// Created by jongbeom on 26. 4. 2..
//

#ifndef CALC_INPUT_DATA_H
#define CALC_INPUT_DATA_H

#include "main.h" // STM32H753ZI 하드웨어 제어 및 기본 타입을 위한 헤더 포함
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "arm_math.h"
#include "kalman_filter.h"

#define WHEEL_COUNT         4U     /* 로봇 바퀴 개수. 현재 코드는 FL/FR/RL/RR 4륜을 가정합니다. */
#define ROBOT_WIDTH_M       0.45f  /* 물리적인 좌우 바퀴 중심 간 거리(미터). 실측치 보관용 상수입니다. */
#define EFFECTIVE_TRACK_WIDTH_M ROBOT_WIDTH_M /* 스키드 스티어 슬립을 반영한 유효 윤거(미터). 회전 오도메트리 튜닝값입니다. */
#define ROBOT_LENGTH_M      0.50f  /* 물리적인 앞뒤 바퀴 중심 간 거리(미터). 현재 기구학 계산에는 직접 사용하지 않습니다. */

/**
 * @brief 단일 바퀴의 엔코더/속도/거리 상태
 * @details
 * - 하드웨어 타이머의 raw CNT delta를 주기마다 누적해 64비트 절대 엔코더 틱을 추적합니다.
 * - 속도는 주기별 delta tick으로 계산하고, 누적 거리는 절대 tick을 직접 거리로 환산해 드리프트를 줄입니다.
 */
typedef struct {
    /* 하드웨어 상수 */
    uint32_t ppr;             /* 구동 바퀴 1회전당 엔코더 누적 count */
    float    diameter_m;      /* 바퀴 지름(미터). 둘레 계산에 사용됩니다. */

    /* 동적 상태 */
    int64_t  total_tick;      /* raw CNT delta를 누적한 64비트 엔코더 tick입니다. */
    int64_t  prev_tick;       /* 직전 샘플의 64비트 엔코더 tick입니다. */
    uint32_t last_counter;    /* 직전 샘플의 하드웨어 CNT raw 값입니다. */
    uint32_t delta_tick_abs;  /* 직전 샘플 대비 부호 보정된 엔코더 delta 절대값 */
    uint32_t fault_count;     /* 물리 한계를 넘는 delta를 감지한 누적 횟수 */
    volatile int32_t overflow_cnt; /* 이전 overflow 방식과의 호환용 필드입니다. 런타임 계산에는 쓰지 않습니다. */
    float    raw_velocity_mps;/* 필터 적용 전 엔코더 기반 바퀴 선속도(m/s) */
    float    velocity_mps;    /* 칼만필터 적용 후 현재 바퀴 선속도(m/s) */
    double   total_distance_m;/* 부호 보정된 절대 tick을 거리로 직접 환산한 누적 이동 거리(m) */
    kalman_filter_t velocity_filter; /* 엔코더 속도 측정값 필터 */
    bool     is_inverted_direction; /* 차량 전진을 양수로 맞추기 위한 부호 반전 여부 */
    bool     has_counter_sample; /* last_counter 기준점이 설정되었는지 여부 */
    bool     is_fault;        /* 향후 엔코더/구동 이상 진단에 사용할 오류 플래그 */
    TIM_HandleTypeDef* htim;  /* 이 바퀴에 연결된 엔코더 타이머 핸들 */
} wheel_state_t;

/**
 * @brief 로봇 전체 상태 집합
 * @details
 * - 바퀴별 상태와 차체 수준 속도, 외부 IMU 수신값, 시스템 준비 상태를 함께 보관합니다.
 * - 상위 제어 루프와 디버그 출력, micro-ROS publish가 모두 이 구조체를 참조합니다.
 */
typedef struct {
    wheel_state_t wheels[WHEEL_COUNT]; /* 바퀴 인덱스 순서: FL, FR, RL, RR */

    /* 로봇 통합 상태 */
    float robot_linear_v;     /* 로봇 차체 중심의 전진 선속도(v, m/s) */
    float robot_angular_w;    /* 로봇 차체 중심의 요(yaw) 회전 각속도(w, rad/s) */

    /* IMU (Jetson 등 외부에서 수신된 데이터) */
    struct {
        float acc_x;              /* x축 선형 가속도(m/s^2) */
        float gyro_z;             /* z축 자이로 각속도(rad/s) */
        uint32_t last_update_time;/* 마지막 IMU 수신 시각(ms). 데이터 freshness 판단용 */
    } imu;

    bool is_system_ready;     /* 초기화 완료 후 정상 동작 가능한 상태인지 여부 */
} robot_status_t;

/* 함수 프로토타입 */

/**
 * @brief 16비트 타이머의 오버플로우/언더플로우 인터럽트 발생 시 호출 (main.c에서 연동)
 */
void encoder_overflow_callback(robot_status_t *robot, TIM_HandleTypeDef *htim);

/**
 * @brief 마지막 갱신 시점까지 누적된 64비트 절대 엔코더 틱을 반환합니다.
 */
int64_t get_total_encoder_tick(wheel_state_t *wheel);

/**
 * @brief 바퀴의 속도 및 누적 이동 거리를 업데이트합니다.
 * @details
 * - 절대 엔코더 틱을 읽고 직전 샘플과의 차이로 속도를 계산합니다.
 * - 누적 거리는 `+=` 누산 대신 절대 tick 기반으로 직접 재계산해 장시간 정밀도 손실을 줄입니다.
 * @param wheel 상태를 업데이트할 개별 바퀴 구조체 포인터
 * @param dt 이전 계산 시점과의 시간 차이 (초 단위, sec)
 */
void wheel_update_velocity(wheel_state_t *wheel, float dt);

/**
 * @brief 각 바퀴의 속도를 취합하여 로봇의 전체 선속도 및 각속도를 기구학으로 계산합니다.
 * @details 스키드 스티어를 2트랙 모델로 근사하며, 회전 계산에는 `EFFECTIVE_TRACK_WIDTH_M`를 사용합니다.
 * @param robot 로봇의 전체 상태를 저장하는 구조체 포인터
 */
void calculate_robot_kinematics(robot_status_t *robot);

/**
 * @brief 계산된 로봇 선속도/각속도와 FL 바퀴 누적 거리를 UART로 전송합니다.
 * @param huart 전송에 사용할 UART 핸들 포인터
 */
void debug_print_robot_state(const robot_status_t *robot, UART_HandleTypeDef *huart);

#endif // CALC_INPUT_DATA_H
