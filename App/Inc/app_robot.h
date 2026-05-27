#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include "calc_input_data.h"

/**
 * @brief 로봇 바퀴별 엔코더 타이머 매핑 테이블
 * @details
 * - 보드 초기화 단계에서 실제 TIM 인스턴스를 로봇 논리 인덱스(FL/FR/RL/RR)에 연결할 때 사용합니다.
 * - 애플리케이션 계층은 이 구조체만 알면 되므로, 핀 배치나 CubeMX 설정 변경이 상위 로직으로 새지 않게 합니다.
 */
typedef struct {
    TIM_HandleTypeDef *front_left;   /* 전방 좌측(Front Left) 엔코더 타이머 */
    TIM_HandleTypeDef *front_right;  /* 전방 우측(Front Right) 엔코더 타이머 */
    TIM_HandleTypeDef *rear_left;    /* 후방 좌측(Rear Left) 엔코더 타이머 */
    TIM_HandleTypeDef *rear_right;   /* 후방 우측(Rear Right) 엔코더 타이머 */
} app_robot_timer_map_t;

/* 시스템 전체에서 공유하는 단일 로봇 상태 저장소 */
extern robot_status_t g_robot_state;

/**
 * @brief 전역 로봇 상태 저장소의 포인터를 반환합니다.
 * @retval 애플리케이션 전체에서 공유하는 `robot_status_t` 포인터
 */
robot_status_t *app_robot_get_state(void);

/**
 * @brief 로봇 상태 기본값을 적재하고 엔코더 타이머를 시작합니다.
 * @param timer_map 바퀴별 TIM 핸들 매핑 정보
 * @retval 초기화 성공 시 true, 입력이 잘못되었거나 HAL 시작에 실패하면 false
 */
bool app_robot_init(const app_robot_timer_map_t *timer_map);

/**
 * @brief 속도/거리/오버플로우 누적값을 현재 하드웨어 기준으로 다시 초기화합니다.
 * @details
 * - 타이머 CNT와 소프트웨어 누적 상태를 같은 기준점으로 맞출 때 사용합니다.
 * - 전원 인가 직후, smoke test 시작 직전, 오도메트리 재영점화 시점에 호출하기 적합합니다.
 * @param robot 초기화할 로봇 상태 포인터
 */
void app_robot_reset_measurements(robot_status_t *robot);

/**
 * @brief 각 바퀴 속도와 로봇 기구학 상태를 한 주기 갱신합니다.
 * @param robot 갱신 대상 로봇 상태 포인터
 * @param dt 직전 갱신 이후 경과 시간(초)
 */
void app_robot_update(robot_status_t *robot, float dt);

/**
 * @brief 엔코더 타이머 업데이트 인터럽트를 로봇 상태에 반영합니다.
 * @param htim 오버플로우/언더플로우가 발생한 타이머 핸들
 */
void app_robot_handle_encoder_overflow(TIM_HandleTypeDef *htim);

#endif
