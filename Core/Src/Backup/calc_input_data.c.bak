//
// Created by jongbeom on 26. 4. 2..
//

#include "calc_input_data.h"

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
 * - 모터 엔코더의 틱을 세는 타이머 카운터가 최댓값(AutoReload)을 넘어 0으로 가거나(Overflow), 
 *   0에서 최댓값으로 갈 때(Underflow) 인터럽트가 발생합니다.
 * - 이 함수는 인터럽트 발생 시 타이머가 카운팅하는 방향(DIR 비트)을 확인합니다.
 * - DIR 비트가 1이면 타이머가 감소 중이므로 Underflow 발생, 카운트를 감소시킵니다.
 * - DIR 비트가 0이면 타이머가 증가 중이므로 Overflow 발생, 카운트를 증가시킵니다.
 * - 이를 통해 16비트/32비트 하드웨어 타이머의 한계를 넘어 64비트의 누적 틱을 추적할 수 있습니다.
 */
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

/**
 * @brief 누적된 절대 엔코더 틱 수를 반환
 * @details
 * [알고리즘 설명]
 * - 인터럽트에서 누적한 오버플로우/언더플로우 횟수와 현재 하드웨어 타이머의 카운터 값을 합산하여
 *   64비트 크기의 절대 틱(Tick)을 계산합니다.
 * - Race Condition 방지: 이 함수가 실행되는 도중에 오버플로우/언더플로우가 발생하면 값이 왜곡될 수 있으므로,
 *   읽는 순간 글로벌 인터럽트를 비활성화(Critical Section)합니다.
 * - 엣지 케이스 보정: 인터럽트를 껐음에도 하드웨어 플래그(UPDATE FLAG)가 발생한 경우, 
 *   이미 타이머가 한 바퀴 돈 것이므로 방향에 맞춰 임시로 1을 보정해주어 정확한 누적 값을 유지합니다.
 */
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
    int64_t total_ticks = ((int64_t)current_overflow * ((int64_t)arr + 1LL)) + (int64_t) current_cnt;
    return total_ticks;
}

/**
 * @brief 휠(Wheel) 단위 선속도(Velocity) 갱신
 * @details
 * [알고리즘 설명]
 * - 이전 호출 시 저장된 총 틱(prev_tick)과 현재 계산된 총 틱(total_tick)의 차이를 구해(delta_tick)
 *   이번 루프 동안 엔코더가 얼마나 회전했는지 계산합니다.
 * - 모터 장착 방향에 따라 좌/우측의 회전 방향 부호가 반대로 잡히는 경우(is_inverted_direction),
 *   이를 소프트웨어적으로 부호 반전하여 동일한 전진 방향(양수)으로 맞춰줍니다.
 * - delta_tick을 모터의 해상도(PPR)로 나누어 몇 바퀴를 회전했는지 비율을 구합니다.
 * - 이 비율에 바퀴의 둘레(PI * 지름)를 곱해 선형 이동 거리(Distance, 미터 단위)를 산출합니다.
 * - 마지막으로, 계산된 거리를 호출 주기(dt)로 나누어 현재 바퀴의 선속도(m/s)를 도출합니다.
 * - 누적 이동 거리는 float 누산(`+=`) 대신 절대 틱을 매번 직접 환산하여 장시간 주행 시 정밀도 손실을 줄입니다.
 */
void wheel_update_velocity(wheel_state_t *wheel, float dt) {
    if (wheel == NULL) {
        return;
    }

    if (dt <= 0.0f) {
        wheel->velocity_mps = 0.0f;
        return;
    }

    // 오버플로우를 반영한 현재 64비트 절대 엔코더 틱 읽기
    wheel->total_tick = get_total_encoder_tick(wheel);
    // 현재값과 직전값 모두 동일한 부호 규칙으로 맞춘 뒤 delta를 계산합니다.
    int64_t current_tick = wheel_apply_direction(wheel, wheel->total_tick);
    int64_t prev_tick = wheel_apply_direction(wheel, wheel->prev_tick);

    // 델타 틱 계산 (이제 완전히 선형적으로 증가/감소하는 값을 사용하므로 오버플로우 걱정이 없음)
    int64_t delta_tick = current_tick - prev_tick;

    if (wheel->ppr > 0) {
        double distance = wheel_ticks_to_distance_m(wheel, delta_tick);

        wheel->velocity_mps = (float) (distance / (double) dt);
        // 절대 tick을 직접 환산해 누적 오차가 주기별 덧셈에 의해 커지는 것을 방지합니다.
        wheel->total_distance_m = wheel_ticks_to_distance_m(wheel, current_tick);
    } else {
        wheel->velocity_mps = 0.0f;
        wheel->total_distance_m = 0.0;
    }

    wheel->prev_tick = wheel->total_tick;
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
