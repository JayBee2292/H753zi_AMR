//
// Created by jongbeom on 26. 4. 2..
//

#include "calc_input_data.h"

#define ENCODER_MAX_WHEEL_SPEED_MPS 3.5f
#define ENCODER_MAX_DELTA_MARGIN_TICKS 2048U
#define ENCODER_VELOCITY_FILTER_INITIAL_P 1.0f
#define ENCODER_VELOCITY_FILTER_Q 0.04f
#define ENCODER_VELOCITY_FILTER_R 0.12f

/**
 * @brief 차량 전진 기준으로 엔코더 부호를 맞춥니다.
 * @details
 * - 좌우 대칭 배치 때문에 같은 전진이라도 하드웨어 카운트 방향이 반대인 바퀴가 있을 수 있습니다.
 * - 이 함수는 절대 tick 또는 delta tick에 동일한 부호 규칙을 적용해 상위 계산을 단순화합니다.
 * @param wheel 부호 규칙을 참조할 바퀴 상태
 * @param tick 보정할 tick 값
 * @retval 차량 전진 기준으로 정규화된 tick 값
 */
static int64_t wheel_apply_direction(const wheel_state_t *wheel, int64_t tick)
{
    if (wheel != NULL && wheel->is_inverted_direction) {
        return -tick;
    }

    return tick;
}

static uint64_t encoder_timer_period(const TIM_HandleTypeDef *htim)
{
    if (htim == NULL) {
        return 0U;
    }

    return (uint64_t) __HAL_TIM_GET_AUTORELOAD((TIM_HandleTypeDef *) htim) + 1ULL;
}

static int64_t encoder_counter_delta(const wheel_state_t *wheel, uint32_t current_counter)
{
    uint64_t period;
    int64_t delta;
    int64_t half_period;

    if (wheel == NULL || wheel->htim == NULL) {
        return 0;
    }

    period = encoder_timer_period(wheel->htim);
    if (period <= 1ULL) {
        return 0;
    }

    delta = (int64_t) current_counter - (int64_t) wheel->last_counter;
    half_period = (int64_t) (period / 2ULL);

    if (delta > half_period) {
        delta -= (int64_t) period;
    } else if (delta < -half_period) {
        delta += (int64_t) period;
    }

    return delta;
}

static uint32_t encoder_max_delta_ticks(const wheel_state_t *wheel, float dt)
{
    double circumference_m;
    double max_delta;
    uint64_t period;
    uint64_t max_correctable_delta;

    if (wheel == NULL || wheel->ppr == 0U || wheel->diameter_m <= 0.0f || dt <= 0.0f) {
        return 0U;
    }

    circumference_m = (double) PI * (double) wheel->diameter_m;
    max_delta =
        ((double) ENCODER_MAX_WHEEL_SPEED_MPS * (double) dt / circumference_m) *
        (double) wheel->ppr;
    max_delta += (double) ENCODER_MAX_DELTA_MARGIN_TICKS;

    period = encoder_timer_period(wheel->htim);
    if (period > 2ULL) {
        max_correctable_delta = (period / 2ULL) - 1ULL;
        if (max_delta > (double) max_correctable_delta) {
            max_delta = (double) max_correctable_delta;
        }
    }

    if (max_delta < 0.0) {
        return 0U;
    }

    if (max_delta > (double) UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t) max_delta;
}

static bool encoder_delta_is_plausible(const wheel_state_t *wheel, int64_t delta_tick, float dt)
{
    int64_t abs_delta = (delta_tick < 0) ? -delta_tick : delta_tick;
    uint32_t max_delta = encoder_max_delta_ticks(wheel, dt);

    return abs_delta <= (int64_t) max_delta;
}

/**
 * @brief 엔코더 tick 값을 바퀴 선형 이동 거리로 변환합니다.
 * @param wheel 바퀴 지름과 PPR을 담고 있는 바퀴 상태
 * @param tick 거리로 바꿀 tick 값
 * @retval 변환된 이동 거리(m)
 */
static double wheel_ticks_to_distance_m(const wheel_state_t *wheel, int64_t tick)
{
    if (wheel == NULL || wheel->ppr == 0U) {
        return 0.0;
    }

    return ((double) tick / (double) wheel->ppr) * (double) PI * (double) wheel->diameter_m;
}

/**
 * @brief 타이머 오버플로우/언더플로우 콜백
 * @details
 * [알고리즘 설명]
 * - 현재 런타임은 update interrupt 누적 대신 주기 샘플링 delta를 64비트로 누적합니다.
 * - CubeMX 인터럽트 핸들러 호환을 위해 심볼은 유지하지만 계산에는 사용하지 않습니다.
 */
void encoder_overflow_callback(robot_status_t *robot, TIM_HandleTypeDef *htim) {
    (void) robot;
    (void) htim;
}

/**
 * @brief 누적된 절대 엔코더 틱 수를 반환
 * @details
 * [알고리즘 설명]
 * - `wheel_update_velocity()`가 누적해 둔 64비트 tick을 반환합니다.
 * - 하드웨어 CNT는 16비트/32비트 wrap-around 보정을 거쳐 주기마다 `total_tick`에 더해집니다.
 */
int64_t get_total_encoder_tick(wheel_state_t *wheel) {
    if (wheel == NULL || wheel->htim == NULL) return 0;

    return wheel->total_tick;
}

/**
 * @brief 휠(Wheel) 단위 선속도(Velocity) 갱신
 * @details
 * [알고리즘 설명]
 * - 이전 호출 시 저장된 총 틱(prev_tick)과 현재 계산된 총 틱(total_tick)의 차이를 구해(delta_tick)
 *   이번 루프 동안 엔코더가 얼마나 회전했는지 계산합니다.
 * - 모터 장착 방향에 따라 좌/우측의 회전 방향 부호가 반대로 잡히는 경우(is_inverted_direction),
 *   이를 소프트웨어적으로 부호 반전하여 동일한 전진 방향(양수)으로 맞춰줍니다.
 * - delta_tick을 구동 바퀴 1회전당 count로 나누어 바퀴 회전량을 구합니다.
 * - 이 비율에 바퀴의 둘레(PI * 지름)를 곱해 선형 이동 거리(Distance, 미터 단위)를 산출합니다.
 * - 마지막으로, 계산된 거리를 호출 주기(dt)로 나누어 현재 바퀴의 선속도(m/s)를 도출합니다.
 * - 누적 이동 거리는 float 누산(`+=`) 대신 절대 틱을 매번 직접 환산하여 장시간 주행 시 정밀도 손실을 줄입니다.
 */
void wheel_update_velocity(wheel_state_t *wheel, float dt) {
    uint32_t current_counter;
    int64_t raw_delta_tick;
    int64_t signed_delta_tick;
    int64_t current_tick;
    int64_t delta_tick_abs;

    if (wheel == NULL || wheel->htim == NULL) {
        return;
    }

    current_counter = __HAL_TIM_GET_COUNTER(wheel->htim);
    if (!wheel->has_counter_sample) {
        wheel->last_counter = current_counter;
        wheel->has_counter_sample = true;
        wheel->raw_velocity_mps = 0.0f;
        wheel->velocity_mps = 0.0f;
        wheel->delta_tick_abs = 0U;
        kalman_filter_reset(&wheel->velocity_filter, 0.0f);
        return;
    }

    if (dt <= 0.0f) {
        wheel->raw_velocity_mps = 0.0f;
        wheel->velocity_mps = 0.0f;
        wheel->delta_tick_abs = 0U;
        kalman_filter_reset(&wheel->velocity_filter, 0.0f);
        return;
    }

    raw_delta_tick = encoder_counter_delta(wheel, current_counter);

    if (!encoder_delta_is_plausible(wheel, raw_delta_tick, dt)) {
        wheel->last_counter = current_counter;
        wheel->prev_tick = wheel->total_tick;
        wheel->delta_tick_abs = 0U;
        wheel->raw_velocity_mps = 0.0f;
        wheel->velocity_mps = 0.0f;
        wheel->is_fault = true;
        kalman_filter_reset(&wheel->velocity_filter, 0.0f);
        if (wheel->fault_count < UINT32_MAX) {
            wheel->fault_count++;
        }
        return;
    }

    wheel->last_counter = current_counter;
    wheel->prev_tick = wheel->total_tick;
    wheel->total_tick += raw_delta_tick;
    wheel->is_fault = false;

    current_tick = wheel_apply_direction(wheel, wheel->total_tick);
    signed_delta_tick = wheel_apply_direction(wheel, raw_delta_tick);
    delta_tick_abs = (signed_delta_tick < 0) ? -signed_delta_tick : signed_delta_tick;

    if (delta_tick_abs > (int64_t) UINT32_MAX) {
        wheel->delta_tick_abs = UINT32_MAX;
    } else {
        wheel->delta_tick_abs = (uint32_t) delta_tick_abs;
    }

    if (wheel->ppr > 0) {
        double distance = wheel_ticks_to_distance_m(wheel, signed_delta_tick);
        float measured_velocity_mps = (float) (distance / (double) dt);

        if (!wheel->velocity_filter.is_initialized) {
            kalman_filter_init(
                &wheel->velocity_filter,
                measured_velocity_mps,
                ENCODER_VELOCITY_FILTER_INITIAL_P,
                ENCODER_VELOCITY_FILTER_Q,
                ENCODER_VELOCITY_FILTER_R);
        }

        wheel->raw_velocity_mps = measured_velocity_mps;
        wheel->velocity_mps = kalman_filter_update(
            &wheel->velocity_filter,
            measured_velocity_mps);
        // 절대 tick을 직접 환산해 누적 오차가 주기별 덧셈에 의해 커지는 것을 방지합니다.
        wheel->total_distance_m = wheel_ticks_to_distance_m(wheel, current_tick);
    } else {
        wheel->raw_velocity_mps = 0.0f;
        wheel->velocity_mps = 0.0f;
        wheel->total_distance_m = 0.0;
        kalman_filter_reset(&wheel->velocity_filter, 0.0f);
    }
}

/**
 * @brief 로봇 기구학(Kinematics) 기반 전진 선속도/회전 각속도 계산
 * @details
 * [알고리즘 설명]
 * - 스키드 스티어(Skid-steer) 방식 4륜 구동 로봇의 정방향 기구학(Forward Kinematics) 공식 적용.
 * - 먼저 좌측 전/후방 바퀴 속도의 평균을 내어 로봇 좌측 트랙의 통합 속도(v_left)를 구합니다.
 * - 마찬가지로 우측 전/후방 바퀴 속도의 평균을 내어 우측 트랙의 통합 속도(v_right)를 구합니다.
 * - 이를 통해 4륜 로봇을 가상의 2륜 차동 구동(Differential Drive) 로봇 모델로 단순화시킵니다.
 * - 로봇의 중심 기준 전진 선속도(V)는 이 두 통합 속도의 평균값으로 계산됩니다. [ V = (v_left + v_right) / 2 ]
 * - 로봇의 회전 각속도(W)는 양쪽 속도의 차이를 유효 윤거(EFFECTIVE_TRACK_WIDTH_M)로 나누어 계산합니다.
 *   물리 폭과 다른 값을 둘 수 있어 스키드 스티어 슬립 오차를 튜닝할 수 있습니다.
 */
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

    // 로봇 전체 회전 각속도 (w, rad/s): (우측 속도 - 좌측 속도) / 유효 윤거(EFFECTIVE_TRACK_WIDTH_M)
    if (EFFECTIVE_TRACK_WIDTH_M > 0.0f) {
        robot->robot_angular_w = (v_right - v_left) / EFFECTIVE_TRACK_WIDTH_M;
    } else {
        robot->robot_angular_w = 0.0f;
    }
}

#include <stdio.h>
#include <string.h>

/**
 * @brief 계산된 차체 속도와 FL 누적 거리를 UART로 출력합니다.
 * @details
 * - 디버그 문자열 길이를 고정 버퍼에 생성한 뒤 blocking UART 전송으로 내보냅니다.
 * - 현재는 대표값으로 Front Left 바퀴의 누적 거리만 함께 출력합니다.
 * @param robot 출력할 로봇 상태 포인터
 * @param huart 전송에 사용할 UART 핸들
 */
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
