#include "app_motor_smoke.h"

#include "app_motor_drive.h"

#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

enum {
    APP_MOTOR_SMOKE_UPDATE_MS = 20U, /* 엔코더/속도 갱신 주기 */
    APP_MOTOR_SMOKE_PRINT_MS = 200U, /* UART 상태 출력 주기 */
};

/* 전원 보드가 차단(tripping)되는 것을 방지하기 위해 전체 바퀴 스모크 테스트 시의
 * 모터 출력(duty)을 보수적으로 유지합니다. */
static const int32_t APP_MOTOR_SMOKE_DUTY_PERCENT = 50;

/**
 * @brief 모터 smoke test 내부 상태 저장소
 * @details
 * - raw CNT 기준 변화량과 `app_robot_update()`가 계산한 의미 있는 속도/누적 tick을 동시에 관찰하기 위해
 *   이전 하드웨어 카운터와 디버그 출력 상태를 별도로 유지합니다.
 */
typedef struct {
    robot_status_t *robot;               /* 점검 대상 로봇 상태 */
    UART_HandleTypeDef *debug_uart;      /* 상태 문자열 출력용 UART */
    int32_t front_left_raw_delta;        /* FL 하드웨어 CNT raw delta */
    int32_t front_right_raw_delta;       /* FR 하드웨어 CNT raw delta */
    int32_t rear_left_raw_delta;         /* RL 하드웨어 CNT raw delta */
    int32_t rear_right_raw_delta;        /* RR 하드웨어 CNT raw delta */
    uint32_t front_left_prev_cnt;        /* FL 직전 CNT 샘플 */
    uint32_t front_right_prev_cnt;       /* FR 직전 CNT 샘플 */
    uint32_t rear_left_prev_cnt;         /* RL 직전 CNT 샘플 */
    uint32_t rear_right_prev_cnt;        /* RR 직전 CNT 샘플 */
    uint32_t last_update_ms;             /* 마지막 속도 계산 시각(ms) */
    uint32_t last_print_ms;              /* 마지막 UART 출력 시각(ms) */
    bool is_initialized;                 /* 초기화 완료 여부 */
} app_motor_smoke_state_t;

/* smoke test는 단일 인스턴스로만 동작하므로 파일 지역 전역 상태를 사용합니다. */
static app_motor_smoke_state_t g_app_motor_smoke = {0};

/**
 * @brief 64비트 정수를 문자열로 변환 (표준 라이브러리 대체용)
 * @details
 * [알고리즘 설명]
 * - 임베디드 환경에서 64비트 정수(int64_t)의 `printf` 처리가 무겁거나 지원되지 않을 때를 대비한 커스텀 변환 함수입니다.
 * - 음수인 경우 절댓값(magnitude)을 구하기 위해 부호를 반전합니다. (단, 2의 보수 최솟값 엣지 케이스도 안전하게 처리합니다)
 * - 10으로 나눈 나머지(modulo 10) 연산을 반복하여 1의 자리부터 역순으로 문자 배열(`digits`)에 저장합니다.
 * - 저장된 역순 문자열을 뒤집어 원래 버퍼(`buffer`)에 복사하며, 필요시 앞에 마이너스 부호(`-`)를 추가합니다.
 */
static void app_motor_smoke_format_i64(int64_t value, char *buffer, size_t buffer_size)
{
    char digits[24];
    size_t digit_count = 0U;
    uint64_t magnitude;
    size_t index = 0U;

    if (buffer == NULL || buffer_size == 0U) {
        return;
    }

    if (value < 0) {
        magnitude = (uint64_t) (-(value + 1)) + 1ULL;
    } else {
        magnitude = (uint64_t) value;
    }

    do {
        digits[digit_count++] = (char) ('0' + (magnitude % 10ULL));
        magnitude /= 10ULL;
    } while (magnitude != 0ULL && digit_count < sizeof(digits));

    if (value < 0 && index < (buffer_size - 1U)) {
        buffer[index++] = '-';
    }

    while (digit_count > 0U && index < (buffer_size - 1U)) {
        buffer[index++] = digits[--digit_count];
    }

    buffer[index] = '\0';
}

/**
 * @brief 하드웨어 타이머 변화량(Raw Delta) 계산 및 Wrap-around 보정
 * @details
 * [알고리즘 설명]
 * - 이전 틱 카운트(previous_cnt)와 현재 틱 카운트(current_cnt)의 차이를 구하여(delta)
 *   두 샘플링 시점 간 엔코더가 얼만큼 회전했는지 계산합니다.
 * - 이 때, 16비트/32비트 하드웨어 타이머 특성상 최댓값을 넘어 0으로 돌아가거나(Overflow),
 *   0에서 최댓값으로 언더플로우(Underflow)되는 "Wrap-around" 현상이 발생할 수 있습니다.
 * - 모터가 두 샘플링 사이클 사이에 카운터 표현 범위의 절반 이상 이동하지 않는다고 가정합니다.
 * - 만약 차이(delta)가 주기의 절반보다 크다면, 이는 정상적인 정회전이 아니라 역방향으로
 *   언더플로우된 것으로 간주하고 주기(Period)만큼 빼서 음수 델타로 보정합니다.
 * - 반대로 델타가 -(주기의 절반)보다 작다면, 오버플로우된 정회전으로 간주하여
 *   주기만큼 더해 양수 델타로 보정합니다.
 */
static int32_t app_motor_smoke_compute_raw_delta(
    TIM_HandleTypeDef *htim,
    uint32_t previous_cnt,
    uint32_t current_cnt)
{
    uint64_t period;
    int64_t delta;

    if (htim == NULL) {
        return 0;
    }

    period = (uint64_t) __HAL_TIM_GET_AUTORELOAD(htim) + 1ULL;
    delta = (int64_t) current_cnt - (int64_t) previous_cnt;

    /* 16비트 CNT wrap-around를 고려해, 두 샘플 사이 이동량이 카운터 범위 절반을 넘지 않는다는
     * 가정 하에서 delta를 가장 짧은 signed 변화량으로 되접습니다. */
    if (period > 0ULL) {
        int64_t half_period = (int64_t) (period / 2ULL);

        if (delta > half_period) {
            delta -= (int64_t) period;
        } else if (delta < -half_period) {
            delta += (int64_t) period;
        }
    }

    return (int32_t) delta;
}

/**
 * @brief smoke test 상태를 사람이 읽기 쉬운 한 줄 문자열로 출력합니다.
 * @details
 * - CNT(raw), 절대 total tick, raw delta, 방향 반전 플래그, 계산된 속도를 바퀴별로 동시에 보여줍니다.
 * - 마지막의 `V`, `W`는 차체 수준 선속도/각속도입니다.
 */
static void app_motor_smoke_print_status(void)
{
    char line[512];
    char fl_total_tick[24];
    char fr_total_tick[24];
    char rl_total_tick[24];
    char rr_total_tick[24];
    robot_status_t *robot = g_app_motor_smoke.robot;
    uint32_t fl_cnt = __HAL_TIM_GET_COUNTER(robot->wheels[0].htim);
    uint32_t fr_cnt = __HAL_TIM_GET_COUNTER(robot->wheels[1].htim);
    uint32_t rl_cnt = __HAL_TIM_GET_COUNTER(robot->wheels[2].htim);
    uint32_t rr_cnt = __HAL_TIM_GET_COUNTER(robot->wheels[3].htim);
    long fl_mmps = (long) (robot->wheels[0].velocity_mps * 1000.0f);
    long fr_mmps = (long) (robot->wheels[1].velocity_mps * 1000.0f);
    long rl_mmps = (long) (robot->wheels[2].velocity_mps * 1000.0f);
    long rr_mmps = (long) (robot->wheels[3].velocity_mps * 1000.0f);
    long v_mmps = (long) (robot->robot_linear_v * 1000.0f);
    long w_mrad = (long) (robot->robot_angular_w * 1000.0f);

    app_motor_smoke_format_i64(robot->wheels[0].total_tick, fl_total_tick, sizeof(fl_total_tick));
    app_motor_smoke_format_i64(robot->wheels[1].total_tick, fr_total_tick, sizeof(fr_total_tick));
    app_motor_smoke_format_i64(robot->wheels[2].total_tick, rl_total_tick, sizeof(rl_total_tick));
    app_motor_smoke_format_i64(robot->wheels[3].total_tick, rr_total_tick, sizeof(rr_total_tick));

    int length = snprintf(
        line,
        sizeof(line),
        "FL c=%lu t=%s d=%ld i=%u v=%ld | FR c=%lu t=%s d=%ld i=%u v=%ld | RL c=%lu t=%s d=%ld i=%u v=%ld | RR c=%lu t=%s d=%ld i=%u v=%ld | V=%ld W=%ld\r\n",
        (unsigned long) fl_cnt,
        fl_total_tick,
        (long) g_app_motor_smoke.front_left_raw_delta,
        robot->wheels[0].is_inverted_direction ? 1U : 0U,
        fl_mmps,
        (unsigned long) fr_cnt,
        fr_total_tick,
        (long) g_app_motor_smoke.front_right_raw_delta,
        robot->wheels[1].is_inverted_direction ? 1U : 0U,
        fr_mmps,
        (unsigned long) rl_cnt,
        rl_total_tick,
        (long) g_app_motor_smoke.rear_left_raw_delta,
        robot->wheels[2].is_inverted_direction ? 1U : 0U,
        rl_mmps,
        (unsigned long) rr_cnt,
        rr_total_tick,
        (long) g_app_motor_smoke.rear_right_raw_delta,
        robot->wheels[3].is_inverted_direction ? 1U : 0U,
        rr_mmps,
        v_mmps,
        w_mrad);

    if (length > 0) {
        HAL_UART_Transmit(
            g_app_motor_smoke.debug_uart,
            (uint8_t *) line,
            (uint16_t) length,
            HAL_MAX_DELAY);
    }
}

/**
 * @brief 모터/엔코더 all-wheel smoke test를 시작합니다.
 * @details
 * - 측정값을 재영점화하고 PWM 출력을 시작한 뒤, 모든 바퀴에 동일한 전진 duty를 인가합니다.
 * - 함수가 성공하면 이후 `app_motor_smoke_process()`를 주기적으로 호출해야 속도 계산과 UART 출력이 진행됩니다.
 * @param robot 점검 대상 로봇 상태
 * @param debug_uart 디버그 출력용 UART
 * @retval 성공 시 true
 */
bool app_motor_smoke_init(robot_status_t *robot, UART_HandleTypeDef *debug_uart)
{
    if (robot == NULL || debug_uart == NULL) {
        return false;
    }

    memset(&g_app_motor_smoke, 0, sizeof(g_app_motor_smoke));
    g_app_motor_smoke.robot = robot;
    g_app_motor_smoke.debug_uart = debug_uart;

    app_robot_reset_measurements(robot);

    if (!app_motor_drive_init()) {
        return false;
    }

    /* raw CNT 변화량은 total_tick 기반 processed 데이터와 별도로 추적합니다. */
    g_app_motor_smoke.front_left_prev_cnt = __HAL_TIM_GET_COUNTER(robot->wheels[0].htim);
    g_app_motor_smoke.front_right_prev_cnt = __HAL_TIM_GET_COUNTER(robot->wheels[1].htim);
    g_app_motor_smoke.rear_left_prev_cnt = __HAL_TIM_GET_COUNTER(robot->wheels[2].htim);
    g_app_motor_smoke.rear_right_prev_cnt = __HAL_TIM_GET_COUNTER(robot->wheels[3].htim);

    /* 배선, DIR 핀 극성, 그리고 엔코더 부호를 함께 점검할 수 있도록
     * 네 바퀴 모두 차량 전진 방향으로 구동합니다. */
    app_motor_drive_set_all_output(APP_MOTOR_SMOKE_DUTY_PERCENT);

    g_app_motor_smoke.last_update_ms = osKernelGetTickCount();
    g_app_motor_smoke.last_print_ms = g_app_motor_smoke.last_update_ms;
    g_app_motor_smoke.is_initialized = true;
    return true;
}

/**
 * @brief smoke test 주기 작업을 수행합니다.
 * @details
 * - 현재 하드웨어 CNT를 읽어 raw delta를 계산하고, 같은 dt로 `app_robot_update()`를 호출해 의미 있는 속도도 갱신합니다.
 * - 지정된 출력 주기마다 바퀴별 raw/processed 상태를 UART로 한 줄 출력합니다.
 * @param now_ms 현재 시스템 tick(ms)
 */
void app_motor_smoke_process(uint32_t now_ms)
{
    robot_status_t *robot = g_app_motor_smoke.robot;
    float dt;
    uint32_t rl_now;
    uint32_t rr_now;
    uint32_t fl_now;
    uint32_t fr_now;

    if (!g_app_motor_smoke.is_initialized || robot == NULL) {
        return;
    }

    dt = (float) (now_ms - g_app_motor_smoke.last_update_ms) / 1000.0f;
    if (dt <= 0.0f) {
        return;
    }

    g_app_motor_smoke.last_update_ms = now_ms;

    fl_now = __HAL_TIM_GET_COUNTER(robot->wheels[0].htim);
    fr_now = __HAL_TIM_GET_COUNTER(robot->wheels[1].htim);
    rl_now = __HAL_TIM_GET_COUNTER(robot->wheels[2].htim);
    rr_now = __HAL_TIM_GET_COUNTER(robot->wheels[3].htim);

    g_app_motor_smoke.front_left_raw_delta =
        app_motor_smoke_compute_raw_delta(robot->wheels[0].htim, g_app_motor_smoke.front_left_prev_cnt, fl_now);
    g_app_motor_smoke.front_right_raw_delta =
        app_motor_smoke_compute_raw_delta(robot->wheels[1].htim, g_app_motor_smoke.front_right_prev_cnt, fr_now);
    g_app_motor_smoke.rear_left_raw_delta =
        app_motor_smoke_compute_raw_delta(robot->wheels[2].htim, g_app_motor_smoke.rear_left_prev_cnt, rl_now);
    g_app_motor_smoke.rear_right_raw_delta =
        app_motor_smoke_compute_raw_delta(robot->wheels[3].htim, g_app_motor_smoke.rear_right_prev_cnt, rr_now);

    g_app_motor_smoke.front_left_prev_cnt = fl_now;
    g_app_motor_smoke.front_right_prev_cnt = fr_now;
    g_app_motor_smoke.rear_left_prev_cnt = rl_now;
    g_app_motor_smoke.rear_right_prev_cnt = rr_now;

    app_robot_update(robot, dt);

    if ((now_ms - g_app_motor_smoke.last_print_ms) >= APP_MOTOR_SMOKE_PRINT_MS) {
        g_app_motor_smoke.last_print_ms = now_ms;
        app_motor_smoke_print_status();
    }
}

/**
 * @brief smoke test 내부 계산 주기를 반환합니다.
 * @retval 속도 계산과 raw delta 갱신에 기대하는 호출 주기(ms)
 */
uint32_t app_motor_smoke_get_period_ms(void)
{
    return APP_MOTOR_SMOKE_UPDATE_MS;
}
