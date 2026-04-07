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

#define WHEEL_COUNT         4U     /* 로봇의 총 바퀴 수 */
#define ROBOT_WIDTH_M       0.45f  /* 좌우 바퀴 중심 간 거리 (미터 단위) */
#define ROBOT_LENGTH_M      0.50f  /* 앞뒤 바퀴 중심 간 거리 (미터 단위) */

typedef struct {
    /* 하드웨어 상수 */
    uint32_t ppr;             /* 엔코더의 1회전당 펄스 수 (Pulses Per Revolution) */
    float    diameter_m;      /* 바퀴의 지름 (미터 단위) */
    
    /* 동적 상태 */
    int64_t  total_tick;      /* 오버플로우를 누적한 32/64비트 총 누적 틱 */
    int64_t  prev_tick;       /* 이전 측정 시점의 총 틱 값 */
    int32_t  overflow_cnt;    /* 인터럽트로 누적된 오버플로우 횟수 */
    float    velocity_mps;    /* 현재 바퀴의 선속도 (m/s) */
    float    total_distance_m;/* 해당 바퀴의 누적 이동 거리 (미터 단위) */
    bool     is_inverted_direction; /* 물리적 장착 방향에 따른 부호 반전 여부 (좌우 대칭 대응) */
    bool     is_fault;        /* 엔코더 또는 구동계 이상 유무 플래그 (정상: false, 에러: true) */
    TIM_HandleTypeDef* htim;  /* 이 바퀴에 할당된 타이머 핸들 (예: &htim3) */
} wheel_state_t;

typedef struct {
    wheel_state_t wheels[WHEEL_COUNT]; /* 4개 바퀴의 개별 상태 배열 */
    
    /* 로봇 통합 상태 */
    float robot_linear_v;     /* 로봇 몸체의 전진 선속도 (v, m/s) */
    float robot_angular_w;    /* 로봇 몸체의 회전 각속도 (w, rad/s) */
    
    /* IMU (Jetson 등 외부에서 수신된 데이터) */
    struct {
        float acc_x;              /* x축 가속도 (m/s^2) */
        float gyro_z;             /* z축 자이로스코프 회전 각속도 (rad/s) */
        uint32_t last_update_time;/* IMU 데이터 마지막 수신 시간 (ms 단위, 데이터 신선도 체크용) */
    } imu;

    bool is_system_ready;     /* 시스템 초기화 및 동작 준비 완료 상태 (준비됨: true) */
} robot_status_t;

/* 함수 프로토타입 */

/**
 * @brief 16비트 타이머의 오버플로우/언더플로우 인터럽트 발생 시 호출 (main.c에서 연동)
 */
void encoder_overflow_callback(robot_status_t *robot, TIM_HandleTypeDef *htim);

/**
 * @brief 현재 타이머 카운트와 오버플로우를 취합해 32/64비트 총 누적 틱 반환
 */
int64_t get_total_encoder_tick(wheel_state_t *wheel);

/**
 * @brief 바퀴의 속도 및 누적 이동 거리를 업데이트합니다.
 * @param wheel 상태를 업데이트할 개별 바퀴 구조체 포인터
 * @param dt 이전 계산 시점과의 시간 차이 (초 단위, sec)
 */
void wheel_update_velocity(wheel_state_t *wheel, float dt);

/**
 * @brief 각 바퀴의 속도를 취합하여 로봇의 전체 선속도 및 각속도를 기구학으로 계산합니다.
 * @param robot 로봇의 전체 상태를 저장하는 구조체 포인터
 */
void calculate_robot_kinematics(robot_status_t *robot);

/**
 * @brief 계산된 로봇의 속도, 각속도, 거리를 UART로 전송하여 디버깅합니다.
 * @param huart 전송에 사용할 UART 핸들 포인터
 */
void debug_print_robot_state(const robot_status_t *robot, UART_HandleTypeDef *huart);

#endif // CALC_INPUT_DATA_H
