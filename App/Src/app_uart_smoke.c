#include "app_uart_smoke.h"

#include "app_motor_drive.h"
#include "calc_output_data.h"
#include "cmsis_os2.h"
#include "pid_controller.h"

#include <stdio.h>
#include <string.h>

enum {
    APP_UART_SMOKE_UPDATE_MS = 20U,
    APP_UART_SMOKE_PRINT_MS = 200U,
    APP_UART_SMOKE_DEFAULT_DUTY_PERCENT = 35,
    APP_UART_SMOKE_DUTY_STEP_PERCENT = 5,
    APP_UART_SMOKE_MIN_DUTY_PERCENT = 5,
    APP_UART_SMOKE_MAX_DUTY_PERCENT = 100,
    /* 일반적으로 터미널은 키를 누르고 있는 동안 동일한 키 입력을 반복해서 전송합니다.
     * 최신 이동 명령을 짧은 시간 동안만 "활성화" 상태로 간주하여,
     * 키에서 손을 떼면 명시적인 'x' 입력 없이도 자연스럽게 정지(stop) 상태로 돌아가게 합니다. */
    APP_UART_SMOKE_COMMAND_HOLD_MS = 800U,
    APP_UART_SMOKE_RX_BUFFER_SIZE = 2048U,
    APP_UART_SMOKE_FRAME_HEADER_0 = 0xA5,
    APP_UART_SMOKE_FRAME_HEADER_1 = 0x5A,
    APP_UART_SMOKE_LEGACY_FRAME_LENGTH = 6U,
    APP_UART_SMOKE_TWIST_FRAME_LENGTH = 8U,
    APP_UART_SMOKE_FRAME_MAX_LENGTH = 8U,
};

#define APP_UART_SMOKE_MAX_TRACK_SPEED_MPS 0.60f
#define APP_UART_SMOKE_PID_KP 35.0f
#define APP_UART_SMOKE_PID_KI 12.0f
#define APP_UART_SMOKE_PID_KD 0.0f
#define APP_UART_SMOKE_PID_CORRECTION_LIMIT_PERCENT 35.0f
#define APP_UART_SMOKE_PID_INTEGRAL_LIMIT 1.0f
#define APP_UART_SMOKE_TARGET_DEADBAND_MPS 0.01f
#define APP_UART_SMOKE_FEEDBACK_SIGN_DEADBAND_MPS 0.03f
/* 0: PID 없는 open-loop smoke test. UART DMA parser는 정상 런타임과 동일하게 사용합니다.
 * 1: 엔코더 피드백 기반 PID speed control. */
#define APP_UART_SMOKE_USE_PID 0

typedef enum {
    APP_UART_SMOKE_PACKET_STOP = 0,
    APP_UART_SMOKE_PACKET_FORWARD = 1,
    APP_UART_SMOKE_PACKET_BACKWARD = 2,
    APP_UART_SMOKE_PACKET_TURN_LEFT = 3,
    APP_UART_SMOKE_PACKET_TURN_RIGHT = 4,
    APP_UART_SMOKE_PACKET_CURVE_LEFT = 5,
    APP_UART_SMOKE_PACKET_CURVE_RIGHT = 6,
    APP_UART_SMOKE_PACKET_BACKWARD_CURVE_LEFT = 7,
    APP_UART_SMOKE_PACKET_BACKWARD_CURVE_RIGHT = 8,
    APP_UART_SMOKE_PACKET_TWIST = 0x10,
} app_uart_smoke_packet_motion_t;

typedef enum {
    APP_UART_SMOKE_MOTION_STOP = 0,
    APP_UART_SMOKE_MOTION_FORWARD,
    APP_UART_SMOKE_MOTION_BACKWARD,
    APP_UART_SMOKE_MOTION_TURN_LEFT,
    APP_UART_SMOKE_MOTION_TURN_RIGHT,
    APP_UART_SMOKE_MOTION_CURVE_LEFT,
    APP_UART_SMOKE_MOTION_CURVE_RIGHT,
    APP_UART_SMOKE_MOTION_BACKWARD_CURVE_LEFT,
    APP_UART_SMOKE_MOTION_BACKWARD_CURVE_RIGHT,
} app_uart_smoke_motion_t;

typedef struct {
    robot_status_t *robot;
    UART_HandleTypeDef *uart;
    uint32_t last_update_ms;
    uint32_t last_print_ms;
    uint32_t print_delta_tick_abs[WHEEL_COUNT];
    int32_t duty_percent;
    int32_t left_duty_percent;
    int32_t right_duty_percent;
    int32_t curve_ratio_percent;
    float target_left_velocity_mps;
    float target_right_velocity_mps;
    float feedback_left_velocity_mps;
    float feedback_right_velocity_mps;
    bool left_pid_feedback_usable;
    bool right_pid_feedback_usable;
    pid_controller_t left_pid;
    pid_controller_t right_pid;
    uint32_t last_motion_command_ms;
    app_uart_smoke_motion_t motion;
    bool is_initialized;
} app_uart_smoke_state_t;

static app_uart_smoke_state_t g_app_uart_smoke = {0};
static uint8_t g_app_uart_smoke_rx_buffer[APP_UART_SMOKE_RX_BUFFER_SIZE] __attribute__((section(".dma_buffer")));
static char g_app_uart_smoke_tx_buffer[384] __attribute__((section(".dma_buffer")));
static uint32_t g_app_uart_smoke_rx_read_index = 0U;
static uint8_t g_app_uart_smoke_frame[APP_UART_SMOKE_FRAME_MAX_LENGTH];
static uint32_t g_app_uart_smoke_frame_index = 0U;
static osThreadId_t g_app_uart_smoke_task_id = NULL;

static void app_uart_smoke_write_string(const char *text);
static void app_uart_smoke_reset_pid(void);

static int64_t app_uart_smoke_normalized_tick(const wheel_state_t *wheel)
{
    if (wheel == NULL) {
        return 0;
    }

    return wheel->is_inverted_direction ? -wheel->total_tick : wheel->total_tick;
}

static void app_uart_smoke_format_i64(int64_t value, char *buffer, size_t buffer_size)
{
    uint64_t magnitude;
    char digits[21];
    size_t digit_count = 0U;
    size_t out_index = 0U;

    if (buffer == NULL || buffer_size == 0U) {
        return;
    }

    if (value < 0) {
        magnitude = (uint64_t) (-(value + 1)) + 1U;
        buffer[out_index++] = '-';
    } else {
        magnitude = (uint64_t) value;
    }

    do {
        digits[digit_count++] = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude != 0U && digit_count < sizeof(digits));

    while (digit_count > 0U && out_index + 1U < buffer_size) {
        buffer[out_index++] = digits[--digit_count];
    }

    buffer[out_index] = '\0';
}

static void app_uart_smoke_accumulate_print_delta(const robot_status_t *robot)
{
    if (robot == NULL) {
        return;
    }

    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i) {
        uint32_t delta = robot->wheels[i].delta_tick_abs;
        uint32_t current = g_app_uart_smoke.print_delta_tick_abs[i];

        if ((UINT32_MAX - current) < delta) {
            g_app_uart_smoke.print_delta_tick_abs[i] = UINT32_MAX;
        } else {
            g_app_uart_smoke.print_delta_tick_abs[i] = current + delta;
        }
    }
}

static uint32_t app_uart_smoke_encoder_fault_mask(const robot_status_t *robot)
{
    uint32_t mask = 0U;

    if (robot == NULL) {
        return 0U;
    }

    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i) {
        if (robot->wheels[i].is_fault) {
            mask |= 1UL << i;
        }
    }

    return mask;
}

static bool app_uart_smoke_start_rx_dma(void)
{
    if (g_app_uart_smoke.uart == NULL) {
        return false;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(
            g_app_uart_smoke.uart,
            g_app_uart_smoke_rx_buffer,
            APP_UART_SMOKE_RX_BUFFER_SIZE) != HAL_OK) {
        return false;
    }

    if (g_app_uart_smoke.uart->hdmarx != NULL) {
        /* DMA Half-Transfer 인터럽트를 끕니다. IDLE 또는 Full Transfer에 집중합니다. */
        __HAL_DMA_DISABLE_IT(g_app_uart_smoke.uart->hdmarx, DMA_IT_HT);
    }

    return true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == g_app_uart_smoke.uart && g_app_uart_smoke_task_id != NULL) {
        // IDLE이나 Full 등의 이유로 수신이 발생하면 해당 스레드를 즉시 깨웁니다.
        osThreadFlagsSet(g_app_uart_smoke_task_id, 0x01);
    }
}

static uint32_t app_uart_smoke_get_rx_write_index(void)
{
    if (g_app_uart_smoke.uart == NULL || g_app_uart_smoke.uart->hdmarx == NULL) {
        return 0U;
    }

    return (APP_UART_SMOKE_RX_BUFFER_SIZE -
            __HAL_DMA_GET_COUNTER(g_app_uart_smoke.uart->hdmarx)) %
        APP_UART_SMOKE_RX_BUFFER_SIZE;
}

static void app_uart_smoke_write_string(const char *text)
{
    size_t length;

    if (g_app_uart_smoke.uart == NULL || text == NULL) {
        return;
    }

    length = strlen(text);
    if (length == 0U) {
        return;
    }

    HAL_UART_Transmit(
        g_app_uart_smoke.uart,
        (uint8_t *) text,
        (uint16_t) length,
        HAL_MAX_DELAY);
}

static uint8_t app_uart_smoke_compute_checksum(
    uint8_t header0,
    uint8_t header1,
    uint8_t motion,
    uint8_t duty,
    uint8_t ratio)
{
    return (uint8_t) (header0 ^ header1 ^ motion ^ duty ^ ratio);
}

static uint8_t app_uart_smoke_compute_frame_checksum(
    const uint8_t *frame,
    uint32_t payload_length)
{
    uint8_t checksum = 0U;

    if (frame == NULL) {
        return 0U;
    }

    for (uint32_t i = 0U; i < payload_length; ++i) {
        checksum ^= frame[i];
    }

    return checksum;
}

static uint32_t app_uart_smoke_expected_frame_length(void)
{
    if (g_app_uart_smoke_frame_index >= 3U &&
        g_app_uart_smoke_frame[2] == APP_UART_SMOKE_PACKET_TWIST) {
        return APP_UART_SMOKE_TWIST_FRAME_LENGTH;
    }

    return APP_UART_SMOKE_LEGACY_FRAME_LENGTH;
}

static int16_t app_uart_smoke_read_i16_le(const uint8_t *data)
{
    uint16_t raw;

    if (data == NULL) {
        return 0;
    }

    raw = (uint16_t) data[0] | ((uint16_t) data[1] << 8);
    return (int16_t) raw;
}

static int32_t app_uart_smoke_clamp_duty(int32_t duty_percent)
{
    if (duty_percent > APP_UART_SMOKE_MAX_DUTY_PERCENT) {
        return APP_UART_SMOKE_MAX_DUTY_PERCENT;
    }

    if (duty_percent < -APP_UART_SMOKE_MAX_DUTY_PERCENT) {
        return -APP_UART_SMOKE_MAX_DUTY_PERCENT;
    }

    return duty_percent;
}

static int32_t app_uart_smoke_round_to_i32(float value)
{
    return (value >= 0.0f) ?
        (int32_t) (value + 0.5f) :
        (int32_t) (value - 0.5f);
}

static int32_t app_uart_smoke_velocity_sign(float velocity_mps, float deadband_mps)
{
    if (velocity_mps > deadband_mps) {
        return 1;
    }

    if (velocity_mps < -deadband_mps) {
        return -1;
    }

    return 0;
}

static bool app_uart_smoke_velocity_direction_changed(
    float previous_velocity_mps,
    float next_velocity_mps)
{
    int32_t previous_sign = app_uart_smoke_velocity_sign(
        previous_velocity_mps,
        APP_UART_SMOKE_TARGET_DEADBAND_MPS);
    int32_t next_sign = app_uart_smoke_velocity_sign(
        next_velocity_mps,
        APP_UART_SMOKE_TARGET_DEADBAND_MPS);

    return previous_sign != 0 && next_sign != 0 && previous_sign != next_sign;
}

#if APP_UART_SMOKE_USE_PID
static float app_uart_smoke_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool app_uart_smoke_feedback_is_usable(
    float target_velocity_mps,
    float feedback_velocity_mps)
{
    int32_t target_sign;
    int32_t feedback_sign;

    if (app_uart_smoke_absf(target_velocity_mps) <= APP_UART_SMOKE_TARGET_DEADBAND_MPS) {
        return false;
    }

    if (app_uart_smoke_absf(feedback_velocity_mps) <= APP_UART_SMOKE_FEEDBACK_SIGN_DEADBAND_MPS) {
        return true;
    }

    target_sign = app_uart_smoke_velocity_sign(
        target_velocity_mps,
        APP_UART_SMOKE_TARGET_DEADBAND_MPS);
    feedback_sign = app_uart_smoke_velocity_sign(
        feedback_velocity_mps,
        APP_UART_SMOKE_FEEDBACK_SIGN_DEADBAND_MPS);

    return target_sign == feedback_sign;
}

static int32_t app_uart_smoke_limit_output_to_target_direction(
    int32_t output_duty_percent,
    float target_velocity_mps)
{
    int32_t target_sign = app_uart_smoke_velocity_sign(
        target_velocity_mps,
        APP_UART_SMOKE_TARGET_DEADBAND_MPS);

    if (target_sign == 0) {
        return 0;
    }

    if (target_sign > 0 && output_duty_percent < 0) {
        return 0;
    }

    if (target_sign < 0 && output_duty_percent > 0) {
        return 0;
    }

    return output_duty_percent;
}
#endif

static float app_uart_smoke_duty_to_target_velocity(int32_t duty_percent)
{
    return ((float) duty_percent / 100.0f) * APP_UART_SMOKE_MAX_TRACK_SPEED_MPS;
}

static int32_t app_uart_smoke_target_velocity_to_duty(float velocity_mps)
{
    float duty = (velocity_mps / APP_UART_SMOKE_MAX_TRACK_SPEED_MPS) * 100.0f;

    return app_uart_smoke_clamp_duty(app_uart_smoke_round_to_i32(duty));
}

static void app_uart_smoke_set_target_from_duty(
    int32_t left_duty_percent,
    int32_t right_duty_percent)
{
    g_app_uart_smoke.target_left_velocity_mps =
        app_uart_smoke_duty_to_target_velocity(left_duty_percent);
    g_app_uart_smoke.target_right_velocity_mps =
        app_uart_smoke_duty_to_target_velocity(right_duty_percent);
}

static app_uart_smoke_motion_t app_uart_smoke_motion_from_targets(
    float left_velocity_mps,
    float right_velocity_mps)
{
    const float deadband_mps = 0.01f;

    if (left_velocity_mps > -deadband_mps && left_velocity_mps < deadband_mps &&
        right_velocity_mps > -deadband_mps && right_velocity_mps < deadband_mps) {
        return APP_UART_SMOKE_MOTION_STOP;
    }

    if (left_velocity_mps >= 0.0f && right_velocity_mps >= 0.0f) {
        if (left_velocity_mps > right_velocity_mps + deadband_mps) {
            return APP_UART_SMOKE_MOTION_CURVE_RIGHT;
        }
        if (right_velocity_mps > left_velocity_mps + deadband_mps) {
            return APP_UART_SMOKE_MOTION_CURVE_LEFT;
        }
        return APP_UART_SMOKE_MOTION_FORWARD;
    }

    if (left_velocity_mps <= 0.0f && right_velocity_mps <= 0.0f) {
        if ((-left_velocity_mps) > (-right_velocity_mps) + deadband_mps) {
            return APP_UART_SMOKE_MOTION_BACKWARD_CURVE_LEFT;
        }
        if ((-right_velocity_mps) > (-left_velocity_mps) + deadband_mps) {
            return APP_UART_SMOKE_MOTION_BACKWARD_CURVE_RIGHT;
        }
        return APP_UART_SMOKE_MOTION_BACKWARD;
    }

    if (left_velocity_mps > right_velocity_mps) {
        return APP_UART_SMOKE_MOTION_TURN_RIGHT;
    }

    return APP_UART_SMOKE_MOTION_TURN_LEFT;
}

static void app_uart_smoke_update_legacy_fields_from_targets(void)
{
    float left_abs = g_app_uart_smoke.target_left_velocity_mps;
    float right_abs = g_app_uart_smoke.target_right_velocity_mps;
    float max_abs;
    float min_abs;

    if (left_abs < 0.0f) {
        left_abs = -left_abs;
    }

    if (right_abs < 0.0f) {
        right_abs = -right_abs;
    }

    max_abs = (left_abs > right_abs) ? left_abs : right_abs;
    min_abs = (left_abs < right_abs) ? left_abs : right_abs;

    g_app_uart_smoke.duty_percent =
        app_uart_smoke_target_velocity_to_duty(max_abs);
    if (g_app_uart_smoke.duty_percent < 0) {
        g_app_uart_smoke.duty_percent = -g_app_uart_smoke.duty_percent;
    }

    if (max_abs > 0.0f) {
        g_app_uart_smoke.curve_ratio_percent =
            app_uart_smoke_round_to_i32((min_abs / max_abs) * 100.0f);
    } else {
        g_app_uart_smoke.curve_ratio_percent = 100;
    }

    if (g_app_uart_smoke.curve_ratio_percent < 0) {
        g_app_uart_smoke.curve_ratio_percent = 0;
    } else if (g_app_uart_smoke.curve_ratio_percent > 100) {
        g_app_uart_smoke.curve_ratio_percent = 100;
    }
}

static void app_uart_smoke_limit_target_velocities(
    float *left_velocity_mps,
    float *right_velocity_mps)
{
    float left_abs = *left_velocity_mps;
    float right_abs = *right_velocity_mps;
    float max_abs;

    if (left_abs < 0.0f) {
        left_abs = -left_abs;
    }
    if (right_abs < 0.0f) {
        right_abs = -right_abs;
    }

    max_abs = (left_abs > right_abs) ? left_abs : right_abs;
    if (max_abs > APP_UART_SMOKE_MAX_TRACK_SPEED_MPS) {
        float scale = APP_UART_SMOKE_MAX_TRACK_SPEED_MPS / max_abs;
        *left_velocity_mps *= scale;
        *right_velocity_mps *= scale;
    }
}

static void app_uart_smoke_apply_target_velocities(
    float left_velocity_mps,
    float right_velocity_mps,
    uint32_t now_ms)
{
    app_uart_smoke_limit_target_velocities(
        &left_velocity_mps,
        &right_velocity_mps);

    if (app_uart_smoke_velocity_direction_changed(
            g_app_uart_smoke.target_left_velocity_mps,
            left_velocity_mps)) {
        pid_controller_reset(&g_app_uart_smoke.left_pid);
    }

    if (app_uart_smoke_velocity_direction_changed(
            g_app_uart_smoke.target_right_velocity_mps,
            right_velocity_mps)) {
        pid_controller_reset(&g_app_uart_smoke.right_pid);
    }

    g_app_uart_smoke.target_left_velocity_mps = left_velocity_mps;
    g_app_uart_smoke.target_right_velocity_mps = right_velocity_mps;
    g_app_uart_smoke.motion = app_uart_smoke_motion_from_targets(
        left_velocity_mps,
        right_velocity_mps);
    g_app_uart_smoke.last_motion_command_ms = now_ms;
    app_uart_smoke_update_legacy_fields_from_targets();

    if (g_app_uart_smoke.motion == APP_UART_SMOKE_MOTION_STOP) {
        g_app_uart_smoke.left_duty_percent = 0;
        g_app_uart_smoke.right_duty_percent = 0;
        g_app_uart_smoke.feedback_left_velocity_mps = 0.0f;
        g_app_uart_smoke.feedback_right_velocity_mps = 0.0f;
        g_app_uart_smoke.left_pid_feedback_usable = false;
        g_app_uart_smoke.right_pid_feedback_usable = false;
        app_uart_smoke_reset_pid();
        app_motor_drive_stop();
    }
}

static void app_uart_smoke_reset_pid(void)
{
    pid_controller_reset(&g_app_uart_smoke.left_pid);
    pid_controller_reset(&g_app_uart_smoke.right_pid);
}

static bool app_uart_smoke_decode_packet_motion(
    uint8_t packet_motion,
    app_uart_smoke_motion_t *motion)
{
    if (motion == NULL) {
        return false;
    }

    switch (packet_motion) {
    case APP_UART_SMOKE_PACKET_STOP:
        *motion = APP_UART_SMOKE_MOTION_STOP;
        return true;
    case APP_UART_SMOKE_PACKET_FORWARD:
        *motion = APP_UART_SMOKE_MOTION_FORWARD;
        return true;
    case APP_UART_SMOKE_PACKET_BACKWARD:
        *motion = APP_UART_SMOKE_MOTION_BACKWARD;
        return true;
    case APP_UART_SMOKE_PACKET_TURN_LEFT:
        *motion = APP_UART_SMOKE_MOTION_TURN_LEFT;
        return true;
    case APP_UART_SMOKE_PACKET_TURN_RIGHT:
        *motion = APP_UART_SMOKE_MOTION_TURN_RIGHT;
        return true;
    case APP_UART_SMOKE_PACKET_CURVE_LEFT:
        *motion = APP_UART_SMOKE_MOTION_CURVE_LEFT;
        return true;
    case APP_UART_SMOKE_PACKET_CURVE_RIGHT:
        *motion = APP_UART_SMOKE_MOTION_CURVE_RIGHT;
        return true;
    case APP_UART_SMOKE_PACKET_BACKWARD_CURVE_LEFT:
        *motion = APP_UART_SMOKE_MOTION_BACKWARD_CURVE_LEFT;
        return true;
    case APP_UART_SMOKE_PACKET_BACKWARD_CURVE_RIGHT:
        *motion = APP_UART_SMOKE_MOTION_BACKWARD_CURVE_RIGHT;
        return true;
    default:
        return false;
    }
}

static void app_uart_smoke_apply_motion(void)
{
    int32_t command_left_duty_percent = 0;
    int32_t command_right_duty_percent = 0;

    switch (g_app_uart_smoke.motion) {
    case APP_UART_SMOKE_MOTION_FORWARD:
        command_left_duty_percent = g_app_uart_smoke.duty_percent;
        command_right_duty_percent = g_app_uart_smoke.duty_percent;
        break;
    case APP_UART_SMOKE_MOTION_BACKWARD:
        command_left_duty_percent = -g_app_uart_smoke.duty_percent;
        command_right_duty_percent = -g_app_uart_smoke.duty_percent;
        break;
    case APP_UART_SMOKE_MOTION_TURN_LEFT:
        command_left_duty_percent = -g_app_uart_smoke.duty_percent;
        command_right_duty_percent = g_app_uart_smoke.duty_percent;
        break;
    case APP_UART_SMOKE_MOTION_TURN_RIGHT:
        command_left_duty_percent = g_app_uart_smoke.duty_percent;
        command_right_duty_percent = -g_app_uart_smoke.duty_percent;
        break;
    case APP_UART_SMOKE_MOTION_CURVE_LEFT:
        command_left_duty_percent =
            (g_app_uart_smoke.duty_percent * g_app_uart_smoke.curve_ratio_percent) / 100;
        command_right_duty_percent = g_app_uart_smoke.duty_percent;
        break;
    case APP_UART_SMOKE_MOTION_CURVE_RIGHT:
        command_left_duty_percent = g_app_uart_smoke.duty_percent;
        command_right_duty_percent =
            (g_app_uart_smoke.duty_percent * g_app_uart_smoke.curve_ratio_percent) / 100;
        break;
    case APP_UART_SMOKE_MOTION_BACKWARD_CURVE_LEFT:
        command_left_duty_percent = -g_app_uart_smoke.duty_percent;
        command_right_duty_percent =
            -(g_app_uart_smoke.duty_percent * g_app_uart_smoke.curve_ratio_percent) / 100;
        break;
    case APP_UART_SMOKE_MOTION_BACKWARD_CURVE_RIGHT:
        command_left_duty_percent =
            -(g_app_uart_smoke.duty_percent * g_app_uart_smoke.curve_ratio_percent) / 100;
        command_right_duty_percent = -g_app_uart_smoke.duty_percent;
        break;
    case APP_UART_SMOKE_MOTION_STOP:
    default:
        command_left_duty_percent = 0;
        command_right_duty_percent = 0;
        break;
    }

    app_uart_smoke_set_target_from_duty(
        command_left_duty_percent,
        command_right_duty_percent);

    if (g_app_uart_smoke.motion == APP_UART_SMOKE_MOTION_STOP) {
        g_app_uart_smoke.left_duty_percent = 0;
        g_app_uart_smoke.right_duty_percent = 0;
        g_app_uart_smoke.feedback_left_velocity_mps = 0.0f;
        g_app_uart_smoke.feedback_right_velocity_mps = 0.0f;
        app_uart_smoke_reset_pid();
        app_motor_drive_stop();
    }
}

static void app_uart_smoke_print_help(void)
{
    app_uart_smoke_write_string(
        "\r\nUART remote drive control\r\n"
        "  hold w: forward\r\n"
        "  hold s: backward\r\n"
        "  hold a: turn left\r\n"
        "  hold d: turn right\r\n"
        "  hold q: curve left\r\n"
        "  hold e: curve right\r\n"
        "  hold z: backward curve left\r\n"
        "  hold c: backward curve right\r\n"
        "  x: stop\r\n"
        "  ] / [: duty up/down\r\n"
        "  > / <: curve ratio up/down\r\n"
        "  h or ?: help\r\n");
}

static void app_uart_smoke_print_status(void)
{
    const robot_status_t *robot = g_app_uart_smoke.robot;
    char fl_tick[24];
    char fr_tick[24];
    char rl_tick[24];
    char rr_tick[24];
    int length;

    if (robot == NULL || g_app_uart_smoke.uart == NULL) {
        return;
    }

    app_uart_smoke_format_i64(
        app_uart_smoke_normalized_tick(&robot->wheels[0]),
        fl_tick,
        sizeof(fl_tick));
    app_uart_smoke_format_i64(
        app_uart_smoke_normalized_tick(&robot->wheels[1]),
        fr_tick,
        sizeof(fr_tick));
    app_uart_smoke_format_i64(
        app_uart_smoke_normalized_tick(&robot->wheels[2]),
        rl_tick,
        sizeof(rl_tick));
    app_uart_smoke_format_i64(
        app_uart_smoke_normalized_tick(&robot->wheels[3]),
        rr_tick,
        sizeof(rr_tick));

    length = snprintf(
        g_app_uart_smoke_tx_buffer,
        sizeof(g_app_uart_smoke_tx_buffer),
        "%s V=%ld W=%ld target(L,R)=%ld,%ld fb(L,R)=%ld,%ld pwm(L,R)=%ld,%ld pid(L,R)=%u,%u enc(FL,FR,RL,RR)=%s,%s,%s,%s delta=%lu,%lu,%lu,%lu fault=0x%lX\r\n",
#if APP_UART_SMOKE_USE_PID
        "PID",
#else
        "OPENLOOP",
#endif
        (long) (robot->robot_linear_v * 1000.0f),
        (long) (robot->robot_angular_w * 1000.0f),
        (long) (g_app_uart_smoke.target_left_velocity_mps * 1000.0f),
        (long) (g_app_uart_smoke.target_right_velocity_mps * 1000.0f),
        (long) (g_app_uart_smoke.feedback_left_velocity_mps * 1000.0f),
        (long) (g_app_uart_smoke.feedback_right_velocity_mps * 1000.0f),
        (long) g_app_uart_smoke.left_duty_percent,
        (long) g_app_uart_smoke.right_duty_percent,
        g_app_uart_smoke.left_pid_feedback_usable ? 1U : 0U,
        g_app_uart_smoke.right_pid_feedback_usable ? 1U : 0U,
        fl_tick,
        fr_tick,
        rl_tick,
        rr_tick,
        (unsigned long) g_app_uart_smoke.print_delta_tick_abs[0],
        (unsigned long) g_app_uart_smoke.print_delta_tick_abs[1],
        (unsigned long) g_app_uart_smoke.print_delta_tick_abs[2],
        (unsigned long) g_app_uart_smoke.print_delta_tick_abs[3],
        (unsigned long) app_uart_smoke_encoder_fault_mask(robot));

    if (length <= 0) {
        return;
    }

    if ((size_t) length >= sizeof(g_app_uart_smoke_tx_buffer)) {
        length = (int) sizeof(g_app_uart_smoke_tx_buffer) - 1;
    }

    (void) HAL_UART_Transmit(
        g_app_uart_smoke.uart,
        (uint8_t *) g_app_uart_smoke_tx_buffer,
        (uint16_t) length,
        20U);
    memset(
        g_app_uart_smoke.print_delta_tick_abs,
        0,
        sizeof(g_app_uart_smoke.print_delta_tick_abs));
}

static void app_uart_smoke_update_closed_loop(robot_status_t *robot, float dt)
{
    int32_t base_left_duty;
    int32_t base_right_duty;
#if APP_UART_SMOKE_USE_PID
    float left_correction;
    float right_correction;
#endif
    int32_t left_output;
    int32_t right_output;

    if (robot == NULL || dt <= 0.0f) {
        return;
    }

    if (g_app_uart_smoke.motion == APP_UART_SMOKE_MOTION_STOP) {
        g_app_uart_smoke.left_duty_percent = 0;
        g_app_uart_smoke.right_duty_percent = 0;
        g_app_uart_smoke.feedback_left_velocity_mps = 0.0f;
        g_app_uart_smoke.feedback_right_velocity_mps = 0.0f;
        g_app_uart_smoke.left_pid_feedback_usable = false;
        g_app_uart_smoke.right_pid_feedback_usable = false;
        app_motor_drive_stop();
        return;
    }

#if APP_UART_SMOKE_USE_PID
    if (app_uart_smoke_encoder_fault_mask(robot) != 0U) {
        g_app_uart_smoke.left_duty_percent = 0;
        g_app_uart_smoke.right_duty_percent = 0;
        app_uart_smoke_reset_pid();
        app_motor_drive_stop();
        return;
    }
#endif

    g_app_uart_smoke.feedback_left_velocity_mps =
        (robot->wheels[0].velocity_mps + robot->wheels[2].velocity_mps) / 2.0f;
    g_app_uart_smoke.feedback_right_velocity_mps =
        (robot->wheels[1].velocity_mps + robot->wheels[3].velocity_mps) / 2.0f;

#if APP_UART_SMOKE_USE_PID
    g_app_uart_smoke.left_pid_feedback_usable = app_uart_smoke_feedback_is_usable(
        g_app_uart_smoke.target_left_velocity_mps,
        g_app_uart_smoke.feedback_left_velocity_mps);
    g_app_uart_smoke.right_pid_feedback_usable = app_uart_smoke_feedback_is_usable(
        g_app_uart_smoke.target_right_velocity_mps,
        g_app_uart_smoke.feedback_right_velocity_mps);
#else
    g_app_uart_smoke.left_pid_feedback_usable = false;
    g_app_uart_smoke.right_pid_feedback_usable = false;
    app_uart_smoke_reset_pid();
#endif

    base_left_duty =
        app_uart_smoke_target_velocity_to_duty(g_app_uart_smoke.target_left_velocity_mps);
    base_right_duty =
        app_uart_smoke_target_velocity_to_duty(g_app_uart_smoke.target_right_velocity_mps);

#if APP_UART_SMOKE_USE_PID
    if (g_app_uart_smoke.left_pid_feedback_usable) {
        left_correction = pid_controller_update(
            &g_app_uart_smoke.left_pid,
            g_app_uart_smoke.target_left_velocity_mps,
            g_app_uart_smoke.feedback_left_velocity_mps,
            dt);
    } else {
        left_correction = 0.0f;
        pid_controller_reset(&g_app_uart_smoke.left_pid);
    }

    if (g_app_uart_smoke.right_pid_feedback_usable) {
        right_correction = pid_controller_update(
            &g_app_uart_smoke.right_pid,
            g_app_uart_smoke.target_right_velocity_mps,
            g_app_uart_smoke.feedback_right_velocity_mps,
            dt);
    } else {
        right_correction = 0.0f;
        pid_controller_reset(&g_app_uart_smoke.right_pid);
    }

    left_output = app_uart_smoke_clamp_duty(
        base_left_duty + app_uart_smoke_round_to_i32(left_correction));
    right_output = app_uart_smoke_clamp_duty(
        base_right_duty + app_uart_smoke_round_to_i32(right_correction));
    left_output = app_uart_smoke_limit_output_to_target_direction(
        left_output,
        g_app_uart_smoke.target_left_velocity_mps);
    right_output = app_uart_smoke_limit_output_to_target_direction(
        right_output,
        g_app_uart_smoke.target_right_velocity_mps);
#else
    left_output = base_left_duty;
    right_output = base_right_duty;
#endif

    g_app_uart_smoke.left_duty_percent = left_output;
    g_app_uart_smoke.right_duty_percent = right_output;
    app_motor_drive_set_left_right_output(left_output, right_output);
}

static void app_uart_smoke_update_duty(int32_t delta_percent)
{
    int32_t next_duty = g_app_uart_smoke.duty_percent + delta_percent;

    if (next_duty < APP_UART_SMOKE_MIN_DUTY_PERCENT) {
        next_duty = APP_UART_SMOKE_MIN_DUTY_PERCENT;
    }

    if (next_duty > APP_UART_SMOKE_MAX_DUTY_PERCENT) {
        next_duty = APP_UART_SMOKE_MAX_DUTY_PERCENT;
    }

    g_app_uart_smoke.duty_percent = next_duty;
    app_uart_smoke_apply_motion();
}

static void app_uart_smoke_update_curve_ratio(int32_t delta_percent)
{
    int32_t next_ratio = g_app_uart_smoke.curve_ratio_percent + delta_percent;

    if (next_ratio < 0) {
        next_ratio = 0;
    }

    if (next_ratio > 100) {
        next_ratio = 100;
    }

    g_app_uart_smoke.curve_ratio_percent = next_ratio;
    app_uart_smoke_apply_motion();
}

static void app_uart_smoke_apply_packet(uint8_t packet_motion, uint8_t packet_duty, uint8_t packet_ratio, uint32_t now_ms)
{
    app_uart_smoke_motion_t motion;
    int32_t duty_percent;
    int32_t ratio_percent;

    if (!app_uart_smoke_decode_packet_motion(packet_motion, &motion)) {
        return;
    }

    duty_percent = (int32_t) packet_duty;
    if (duty_percent < APP_UART_SMOKE_MIN_DUTY_PERCENT &&
        motion != APP_UART_SMOKE_MOTION_STOP) {
        duty_percent = APP_UART_SMOKE_MIN_DUTY_PERCENT;
    }

    if (duty_percent > APP_UART_SMOKE_MAX_DUTY_PERCENT) {
        duty_percent = APP_UART_SMOKE_MAX_DUTY_PERCENT;
    }

    ratio_percent = (int32_t) packet_ratio;
    if (ratio_percent < 0) {
        ratio_percent = 0;
    } else if (ratio_percent > 100) {
        ratio_percent = 100;
    }

    g_app_uart_smoke.duty_percent = duty_percent;
    g_app_uart_smoke.curve_ratio_percent = ratio_percent;
    g_app_uart_smoke.motion = motion;
    g_app_uart_smoke.last_motion_command_ms = now_ms;
    app_uart_smoke_apply_motion();
}

static void app_uart_smoke_apply_twist_packet(
    int16_t linear_mmps,
    int16_t angular_mradps,
    uint32_t now_ms)
{
    target_wheel_velocities_t target_wheels = {0};
    float linear_mps = (float) linear_mmps / 1000.0f;
    float angular_radps = (float) angular_mradps / 1000.0f;
    float left_velocity_mps;
    float right_velocity_mps;

    calculate_inverse_kinematics(linear_mps, angular_radps, &target_wheels);

    left_velocity_mps =
        (target_wheels.fl_velocity_mps + target_wheels.rl_velocity_mps) / 2.0f;
    right_velocity_mps =
        (target_wheels.fr_velocity_mps + target_wheels.rr_velocity_mps) / 2.0f;

    app_uart_smoke_apply_target_velocities(
        left_velocity_mps,
        right_velocity_mps,
        now_ms);
}

static void app_uart_smoke_handle_motion_command(
    app_uart_smoke_motion_t motion,
    uint32_t now_ms)
{
    g_app_uart_smoke.motion = motion;
    g_app_uart_smoke.last_motion_command_ms = now_ms;
    app_uart_smoke_apply_motion();
}

static void app_uart_smoke_handle_command(uint8_t command, uint32_t now_ms)
{
    switch (command) {
    case 'w':
    case 'W':
        app_uart_smoke_handle_motion_command(APP_UART_SMOKE_MOTION_FORWARD, now_ms);
        break;
    case 's':
    case 'S':
        app_uart_smoke_handle_motion_command(APP_UART_SMOKE_MOTION_BACKWARD, now_ms);
        break;
    case 'a':
    case 'A':
        app_uart_smoke_handle_motion_command(APP_UART_SMOKE_MOTION_TURN_LEFT, now_ms);
        break;
    case 'd':
    case 'D':
        app_uart_smoke_handle_motion_command(APP_UART_SMOKE_MOTION_TURN_RIGHT, now_ms);
        break;
    case 'q':
    case 'Q':
        app_uart_smoke_handle_motion_command(APP_UART_SMOKE_MOTION_CURVE_LEFT, now_ms);
        break;
    case 'e':
    case 'E':
        app_uart_smoke_handle_motion_command(APP_UART_SMOKE_MOTION_CURVE_RIGHT, now_ms);
        break;
    case 'z':
    case 'Z':
        app_uart_smoke_handle_motion_command(APP_UART_SMOKE_MOTION_BACKWARD_CURVE_LEFT, now_ms);
        break;
    case 'c':
    case 'C':
        app_uart_smoke_handle_motion_command(APP_UART_SMOKE_MOTION_BACKWARD_CURVE_RIGHT, now_ms);
        break;
    case 'x':
    case 'X':
        g_app_uart_smoke.motion = APP_UART_SMOKE_MOTION_STOP;
        g_app_uart_smoke.last_motion_command_ms = now_ms;
        app_uart_smoke_apply_motion();
        break;
    case ']':
        app_uart_smoke_update_duty(APP_UART_SMOKE_DUTY_STEP_PERCENT);
        break;
    case '[':
        app_uart_smoke_update_duty(-APP_UART_SMOKE_DUTY_STEP_PERCENT);
        break;
    case '>':
    case '.':
        app_uart_smoke_update_curve_ratio(APP_UART_SMOKE_DUTY_STEP_PERCENT);
        break;
    case '<':
    case ',':
        app_uart_smoke_update_curve_ratio(-APP_UART_SMOKE_DUTY_STEP_PERCENT);
        break;
    case 'h':
    case 'H':
    case '?':
        app_uart_smoke_print_help();
        break;
    case '\r':
    case '\n':
    case '\0':
        break;
    default:
        if (command >= 0x20U && command <= 0x7eU) {
            app_uart_smoke_write_string("unknown command, press h for help\r\n");
        }
        break;
    }
}

static void app_uart_smoke_consume_rx_byte(uint8_t value, uint32_t now_ms)
{
    if (g_app_uart_smoke_frame_index == 0U) {
        if (value == APP_UART_SMOKE_FRAME_HEADER_0) {
            g_app_uart_smoke_frame[g_app_uart_smoke_frame_index++] = value;
            return;
        }

        app_uart_smoke_handle_command(value, now_ms);
        return;
    }

    if (g_app_uart_smoke_frame_index == 1U) {
        if (value == APP_UART_SMOKE_FRAME_HEADER_1) {
            g_app_uart_smoke_frame[g_app_uart_smoke_frame_index++] = value;
            return;
        }

        g_app_uart_smoke_frame_index = 0U;
        if (value == APP_UART_SMOKE_FRAME_HEADER_0) {
            g_app_uart_smoke_frame[g_app_uart_smoke_frame_index++] = value;
        } else {
            app_uart_smoke_handle_command(value, now_ms);
        }
        return;
    }

    if (g_app_uart_smoke_frame_index >= APP_UART_SMOKE_FRAME_MAX_LENGTH) {
        g_app_uart_smoke_frame_index = 0U;
        return;
    }

    g_app_uart_smoke_frame[g_app_uart_smoke_frame_index++] = value;
    if (g_app_uart_smoke_frame_index < app_uart_smoke_expected_frame_length()) {
        return;
    }

    if (g_app_uart_smoke_frame[2] == APP_UART_SMOKE_PACKET_TWIST) {
        if (g_app_uart_smoke_frame[APP_UART_SMOKE_TWIST_FRAME_LENGTH - 1U] ==
            app_uart_smoke_compute_frame_checksum(
                g_app_uart_smoke_frame,
                APP_UART_SMOKE_TWIST_FRAME_LENGTH - 1U)) {
            app_uart_smoke_apply_twist_packet(
                app_uart_smoke_read_i16_le(&g_app_uart_smoke_frame[3]),
                app_uart_smoke_read_i16_le(&g_app_uart_smoke_frame[5]),
                now_ms);
        }
    } else if (g_app_uart_smoke_frame[5] == app_uart_smoke_compute_checksum(
                   g_app_uart_smoke_frame[0],
                   g_app_uart_smoke_frame[1],
                   g_app_uart_smoke_frame[2],
                   g_app_uart_smoke_frame[3],
                   g_app_uart_smoke_frame[4])) {
        app_uart_smoke_apply_packet(
            g_app_uart_smoke_frame[2],
            g_app_uart_smoke_frame[3],
            g_app_uart_smoke_frame[4],
            now_ms);
    }

    g_app_uart_smoke_frame_index = 0U;
}

static void app_uart_smoke_poll_uart(uint32_t now_ms)
{
    uint32_t write_index = app_uart_smoke_get_rx_write_index();

    while (g_app_uart_smoke_rx_read_index != write_index) {
        uint8_t command = g_app_uart_smoke_rx_buffer[g_app_uart_smoke_rx_read_index];
        app_uart_smoke_consume_rx_byte(command, now_ms);
        g_app_uart_smoke_rx_read_index =
            (g_app_uart_smoke_rx_read_index + 1U) % APP_UART_SMOKE_RX_BUFFER_SIZE;
    }
}

bool app_uart_smoke_init(robot_status_t *robot, UART_HandleTypeDef *command_uart)
{
    if (robot == NULL || command_uart == NULL) {
        return false;
    }

    memset(&g_app_uart_smoke, 0, sizeof(g_app_uart_smoke));
    g_app_uart_smoke.robot = robot;
    g_app_uart_smoke.uart = command_uart;
    g_app_uart_smoke.duty_percent = APP_UART_SMOKE_DEFAULT_DUTY_PERCENT;
    g_app_uart_smoke.curve_ratio_percent = 50;
    g_app_uart_smoke.motion = APP_UART_SMOKE_MOTION_STOP;
    g_app_uart_smoke.last_motion_command_ms = osKernelGetTickCount();
    g_app_uart_smoke_rx_read_index = 0U;
    g_app_uart_smoke_frame_index = 0U;
    memset(g_app_uart_smoke_rx_buffer, 0, sizeof(g_app_uart_smoke_rx_buffer));
    memset(g_app_uart_smoke_frame, 0, sizeof(g_app_uart_smoke_frame));
    pid_controller_init(
        &g_app_uart_smoke.left_pid,
        APP_UART_SMOKE_PID_KP,
        APP_UART_SMOKE_PID_KI,
        APP_UART_SMOKE_PID_KD,
        -APP_UART_SMOKE_PID_CORRECTION_LIMIT_PERCENT,
        APP_UART_SMOKE_PID_CORRECTION_LIMIT_PERCENT,
        -APP_UART_SMOKE_PID_INTEGRAL_LIMIT,
        APP_UART_SMOKE_PID_INTEGRAL_LIMIT);
    pid_controller_init(
        &g_app_uart_smoke.right_pid,
        APP_UART_SMOKE_PID_KP,
        APP_UART_SMOKE_PID_KI,
        APP_UART_SMOKE_PID_KD,
        -APP_UART_SMOKE_PID_CORRECTION_LIMIT_PERCENT,
        APP_UART_SMOKE_PID_CORRECTION_LIMIT_PERCENT,
        -APP_UART_SMOKE_PID_INTEGRAL_LIMIT,
        APP_UART_SMOKE_PID_INTEGRAL_LIMIT);

    app_robot_reset_measurements(robot);

    if (!app_motor_drive_init()) {
        return false;
    }

    g_app_uart_smoke_task_id = osThreadGetId();

    if (!app_uart_smoke_start_rx_dma()) {
        return false;
    }

    app_uart_smoke_apply_motion();

    g_app_uart_smoke.last_update_ms = osKernelGetTickCount();
    g_app_uart_smoke.last_print_ms = g_app_uart_smoke.last_update_ms;
    g_app_uart_smoke.is_initialized = true;
    app_uart_smoke_print_status();
    return true;
}

void app_uart_smoke_process(uint32_t now_ms)
{
    robot_status_t *robot = g_app_uart_smoke.robot;
    uint32_t elapsed_ms;
    float dt;

    if (!g_app_uart_smoke.is_initialized || robot == NULL) {
        return;
    }

    app_uart_smoke_poll_uart(now_ms);

    if (g_app_uart_smoke.motion != APP_UART_SMOKE_MOTION_STOP &&
        (now_ms - g_app_uart_smoke.last_motion_command_ms) > APP_UART_SMOKE_COMMAND_HOLD_MS) {
        g_app_uart_smoke.motion = APP_UART_SMOKE_MOTION_STOP;
        app_uart_smoke_apply_motion();
    }

    elapsed_ms = now_ms - g_app_uart_smoke.last_update_ms;
    if (elapsed_ms < APP_UART_SMOKE_UPDATE_MS) {
        return;
    }

    dt = (float) elapsed_ms / 1000.0f;
    g_app_uart_smoke.last_update_ms = now_ms;
    app_robot_update(robot, dt);
    app_uart_smoke_update_closed_loop(robot, dt);
    app_uart_smoke_accumulate_print_delta(robot);

    if ((now_ms - g_app_uart_smoke.last_print_ms) >= APP_UART_SMOKE_PRINT_MS) {
        g_app_uart_smoke.last_print_ms = now_ms;
        app_uart_smoke_print_status();
    }
}

void app_uart_smoke_get_control_state(app_uart_smoke_control_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->duty_percent = g_app_uart_smoke.duty_percent;
    state->left_duty_percent = g_app_uart_smoke.left_duty_percent;
    state->right_duty_percent = g_app_uart_smoke.right_duty_percent;
    state->curve_ratio_percent = g_app_uart_smoke.curve_ratio_percent;
    state->last_motion_command_ms = g_app_uart_smoke.last_motion_command_ms;
    state->motion = (uint8_t) g_app_uart_smoke.motion;
    state->is_motion_active = g_app_uart_smoke.motion != APP_UART_SMOKE_MOTION_STOP;
}

uint32_t app_uart_smoke_get_period_ms(void)
{
    return APP_UART_SMOKE_UPDATE_MS;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (!g_app_uart_smoke.is_initialized || huart != g_app_uart_smoke.uart) {
        return;
    }

    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    app_uart_smoke_start_rx_dma();
}
