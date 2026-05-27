#include "app_robot.h"

#include <string.h>

#define APP_ROBOT_MEASURED_SHAFT_COUNTS_10_REV 2439004U /* 손으로 돌린 모터 출력축 10회전 측정값 */
#define APP_ROBOT_MEASURED_SHAFT_REVS 10U
#define APP_ROBOT_SHAFT_TURNS_PER_WHEEL_REV_NUM 9U      /* 구동 바퀴 1회전 = 측정 축 9 / 4 회전 */
#define APP_ROBOT_SHAFT_TURNS_PER_WHEEL_REV_DEN 4U
#define APP_ROBOT_WHEEL_PPR \
    ((APP_ROBOT_MEASURED_SHAFT_COUNTS_10_REV * APP_ROBOT_SHAFT_TURNS_PER_WHEEL_REV_NUM + \
      ((APP_ROBOT_MEASURED_SHAFT_REVS * APP_ROBOT_SHAFT_TURNS_PER_WHEEL_REV_DEN) / 2U)) / \
     (APP_ROBOT_MEASURED_SHAFT_REVS * APP_ROBOT_SHAFT_TURNS_PER_WHEEL_REV_DEN))
#define APP_ROBOT_WHEEL_DIAMETER_M 0.21f /* 바닥 직진 실측과 비교하는 유효 진행 지름(m) */

/* 애플리케이션 전역에서 공유하는 단일 로봇 상태 인스턴스 */
robot_status_t g_robot_state;

/**
 * @brief 기본 하드웨어 상수를 채운 바퀴 상태를 생성합니다.
 * @param is_inverted_direction 차량 전진 기준으로 부호를 뒤집어야 하는지 여부
 * @retval 기본값이 채워진 `wheel_state_t`
 */
static wheel_state_t app_robot_make_wheel(bool is_inverted_direction)
{
    wheel_state_t wheel = {0};

    wheel.ppr = APP_ROBOT_WHEEL_PPR;
    wheel.diameter_m = APP_ROBOT_WHEEL_DIAMETER_M;
    wheel.is_inverted_direction = is_inverted_direction;

    return wheel;
}

/**
 * @brief 로봇 상태 구조체에 소프트웨어 기본값을 적재합니다.
 * @details
 * - 구조체 전체를 0으로 초기화한 뒤, 바퀴별 PPR/지름/부호 반전 여부를 채웁니다.
 * - 현재 차체 인덱스는 0:FL, 1:FR, 2:RL, 3:RR 순서를 사용합니다.
 * @param robot 기본값을 적재할 로봇 상태 포인터
 */
static void app_robot_load_defaults(robot_status_t *robot)
{
    if (robot == NULL) {
        return;
    }

    memset(robot, 0, sizeof(*robot));

    robot->wheels[0] = app_robot_make_wheel(false);
    robot->wheels[1] = app_robot_make_wheel(true);
    robot->wheels[2] = app_robot_make_wheel(false);
    robot->wheels[3] = app_robot_make_wheel(true);
}

/**
 * @brief 단일 엔코더 타이머를 시작합니다.
 * @param htim 시작할 엔코더 타이머
 * @retval 성공 시 true
 */
static bool app_robot_start_encoder(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) {
        return false;
    }

    if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK) {
        return false;
    }

    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

    return true;
}

/**
 * @brief 하드웨어 엔코더 카운터 초기화
 * @details
 * [알고리즘 설명]
 * - 모터 테스트(Smoke Test)나 초기 상태 등에서 모든 바퀴의 엔코더 측정값을 기준점(Zero Reference)으로
 *   동기화하기 위해 하드웨어 타이머의 카운터(CNT)를 0으로 강제 초기화합니다.
 * - 또한, 강제 초기화 과정에서 의도치 않게 발생했을 수 있는 타이머 업데이트(Overflow/Underflow)
 *   인터럽트 플래그를 지워서 잘못된 누적을 방지합니다.
 */
static void app_robot_reset_encoder_counter(wheel_state_t *wheel)
{
    if (wheel == NULL || wheel->htim == NULL) {
        return;
    }

    /* 모터 테스트(smoke-test) 구동 중 각 바퀴의 raw CNT 값을 직접 비교할 수 있도록
     * 동일한 영점(zero reference)을 사용합니다. */
    __HAL_TIM_SET_COUNTER(wheel->htim, 0U);
    __HAL_TIM_CLEAR_FLAG(wheel->htim, TIM_FLAG_UPDATE);
}

/**
 * @brief 전역 로봇 상태 저장소 포인터를 반환합니다.
 * @retval `g_robot_state`의 주소
 */
robot_status_t *app_robot_get_state(void)
{
    return &g_robot_state;
}

/**
 * @brief 로봇 애플리케이션 상태와 엔코더 하드웨어를 초기화합니다.
 * @details
 * - 입력된 타이머 매핑을 바퀴 상태에 연결하고, 각 타이머를 엔코더 모드로 시작합니다.
 * - 이후 측정 기준점을 재설정하여 raw CNT와 64비트 누적 tick이 서로 일관된 값이 되게 합니다.
 * @param timer_map 바퀴별 TIM 핸들 매핑
 * @retval 성공 시 true
 */
bool app_robot_init(const app_robot_timer_map_t *timer_map)
{
    if (timer_map == NULL ||
        timer_map->front_left == NULL ||
        timer_map->front_right == NULL ||
        timer_map->rear_left == NULL ||
        timer_map->rear_right == NULL) {
        return false;
    }

    app_robot_load_defaults(&g_robot_state);

    g_robot_state.wheels[0].htim = timer_map->front_left;
    g_robot_state.wheels[1].htim = timer_map->front_right;
    g_robot_state.wheels[2].htim = timer_map->rear_left;
    g_robot_state.wheels[3].htim = timer_map->rear_right;

    /* 보드 종속적인 배선 정보가 런타임 및 통신 모듈에 노출되지 않도록
     * 한 곳에서 모든 엔코더 타이머를 시작합니다. */
    for (uint32_t i = 0; i < WHEEL_COUNT; ++i) {
        if (!app_robot_start_encoder(g_robot_state.wheels[i].htim)) {
            return false;
        }
    }

    app_robot_reset_measurements(&g_robot_state);
    g_robot_state.is_system_ready = true;

    return true;
}

/**
 * @brief 속도/거리/64비트 tick 누적값을 현재 하드웨어 기준점으로 초기화합니다.
 * @details
 * - 하드웨어 CNT를 0으로 맞추고, 소프트웨어 누적값도 같이 0 기준으로 맞춥니다.
 * - 마지막에 `total_tick`과 `prev_tick`을 동일한 현재값으로 맞춰 첫 업데이트에서 가짜 delta가 생기지 않게 합니다.
 * @param robot 초기화할 로봇 상태 포인터
 */
void app_robot_reset_measurements(robot_status_t *robot)
{
    if (robot == NULL) {
        return;
    }

    robot->robot_linear_v = 0.0f;
    robot->robot_angular_w = 0.0f;

    for (uint32_t i = 0; i < WHEEL_COUNT; ++i) {
        wheel_state_t *wheel = &robot->wheels[i];

        app_robot_reset_encoder_counter(wheel);
        wheel->overflow_cnt = 0;
        wheel->last_counter = __HAL_TIM_GET_COUNTER(wheel->htim);
        wheel->has_counter_sample = true;
        wheel->delta_tick_abs = 0U;
        wheel->fault_count = 0U;
        wheel->raw_velocity_mps = 0.0f;
        wheel->velocity_mps = 0.0f;
        wheel->total_distance_m = 0.0;
        wheel->is_fault = false;
        wheel->total_tick = 0;
        wheel->prev_tick = 0;
        kalman_filter_init(&wheel->velocity_filter, 0.0f, 1.0f, 0.04f, 0.12f);
    }
}

/**
 * @brief 로봇 상태를 한 제어 주기만큼 갱신합니다.
 * @details
 * - 모든 바퀴의 엔코더 기반 속도/거리를 먼저 갱신한 뒤,
 *   그 결과를 사용해 차체 선속도와 각속도를 계산합니다.
 * @param robot 갱신 대상 로봇 상태 포인터
 * @param dt 직전 갱신 이후 경과 시간(초)
 */
void app_robot_update(robot_status_t *robot, float dt)
{
    if (robot == NULL) {
        return;
    }

    for (uint32_t i = 0; i < WHEEL_COUNT; ++i) {
        wheel_update_velocity(&robot->wheels[i], dt);
    }

    calculate_robot_kinematics(robot);
}

/**
 * @brief 엔코더 타이머 overflow/underflow 인터럽트를 전역 로봇 상태에 반영합니다.
 * @param htim 인터럽트를 발생시킨 엔코더 타이머
 */
void app_robot_handle_encoder_overflow(TIM_HandleTypeDef *htim)
{
    encoder_overflow_callback(&g_robot_state, htim);
}
