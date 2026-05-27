#include "app_motor_drive.h"

#include "gpio.h"
#include "tim.h"

typedef struct {
    uint32_t pwm_channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_PinState forward_pin_state;
} app_motor_drive_channel_t;

/* 두 스모크 테스트 경로가 동일한 기준을 공유할 수 있도록
 * 바퀴와 출력 핀 간의 매핑을 이곳으로 중앙집중화합니다.
 * 현재 하드웨어 핀 배열 기준은 다음과 같습니다:
 * - CH1 / PE7  -> FL
 * - CH2 / PE8  -> FR
 * - CH3 / PE10 -> RL
 * - CH4 / PE12 -> RR
 * 좌우측 모터의 전진 극성이 다르므로, 각 바퀴별로 전진 시의 DIR 핀 상태도 저장합니다. */
static const app_motor_drive_channel_t g_app_motor_drive_channels[APP_MOTOR_DRIVE_WHEEL_COUNT] = {
    [APP_MOTOR_DRIVE_WHEEL_FL] = {
        .pwm_channel = TIM_CHANNEL_1,
        .dir_port = GPIOE,
        .dir_pin = GPIO_PIN_7,
        .forward_pin_state = GPIO_PIN_SET,
    },
    [APP_MOTOR_DRIVE_WHEEL_FR] = {
        .pwm_channel = TIM_CHANNEL_2,
        .dir_port = GPIOE,
        .dir_pin = GPIO_PIN_8,
        .forward_pin_state = GPIO_PIN_RESET,
    },
    [APP_MOTOR_DRIVE_WHEEL_RL] = {
        .pwm_channel = TIM_CHANNEL_3,
        .dir_port = GPIOE,
        .dir_pin = GPIO_PIN_10,
        .forward_pin_state = GPIO_PIN_SET,
    },
    [APP_MOTOR_DRIVE_WHEEL_RR] = {
        .pwm_channel = TIM_CHANNEL_4,
        .dir_port = GPIOE,
        .dir_pin = GPIO_PIN_12,
        .forward_pin_state = GPIO_PIN_RESET,
    },
};

static bool g_app_motor_drive_is_initialized = false;

static uint32_t app_motor_drive_percent_to_compare(int32_t duty_percent)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t magnitude = (duty_percent < 0) ? (uint32_t) (-duty_percent) : (uint32_t) duty_percent;

    if (magnitude > 100U) {
        magnitude = 100U;
    }

    return (arr * magnitude) / 100U;
}

static void app_motor_drive_apply_direction(
    const app_motor_drive_channel_t *channel,
    int32_t duty_percent)
{
    GPIO_PinState pin_state;

    if (channel == NULL || duty_percent == 0) {
        return;
    }

    pin_state = (duty_percent > 0) ?
        channel->forward_pin_state :
        (channel->forward_pin_state == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(channel->dir_port, channel->dir_pin, pin_state);
}

bool app_motor_drive_init(void)
{
    if (!g_app_motor_drive_is_initialized) {
        if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
            return false;
        }

        if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
            return false;
        }

        if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
            return false;
        }

        if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
            return false;
        }

        g_app_motor_drive_is_initialized = true;
    }

    app_motor_drive_stop();
    return true;
}

void app_motor_drive_stop(void)
{
    for (uint32_t i = 0; i < APP_MOTOR_DRIVE_WHEEL_COUNT; ++i) {
        const app_motor_drive_channel_t *channel = &g_app_motor_drive_channels[i];

        __HAL_TIM_SET_COMPARE(&htim1, channel->pwm_channel, 0U);
        HAL_GPIO_WritePin(channel->dir_port, channel->dir_pin, channel->forward_pin_state);
    }
}

void app_motor_drive_set_wheel_output(
    app_motor_drive_wheel_t wheel,
    int32_t duty_percent)
{
    const app_motor_drive_channel_t *channel;

    if ((uint32_t) wheel >= APP_MOTOR_DRIVE_WHEEL_COUNT) {
        return;
    }

    channel = &g_app_motor_drive_channels[wheel];
    app_motor_drive_apply_direction(channel, duty_percent);
    __HAL_TIM_SET_COMPARE(
        &htim1,
        channel->pwm_channel,
        app_motor_drive_percent_to_compare(duty_percent));
}

void app_motor_drive_set_wheel_outputs(
    int32_t fl_duty_percent,
    int32_t fr_duty_percent,
    int32_t rl_duty_percent,
    int32_t rr_duty_percent)
{
    app_motor_drive_set_wheel_output(APP_MOTOR_DRIVE_WHEEL_FL, fl_duty_percent);
    app_motor_drive_set_wheel_output(APP_MOTOR_DRIVE_WHEEL_FR, fr_duty_percent);
    app_motor_drive_set_wheel_output(APP_MOTOR_DRIVE_WHEEL_RL, rl_duty_percent);
    app_motor_drive_set_wheel_output(APP_MOTOR_DRIVE_WHEEL_RR, rr_duty_percent);
}

void app_motor_drive_set_left_right_output(
    int32_t left_duty_percent,
    int32_t right_duty_percent)
{
    app_motor_drive_set_wheel_outputs(
        left_duty_percent,
        right_duty_percent,
        left_duty_percent,
        right_duty_percent);
}

void app_motor_drive_set_all_output(int32_t duty_percent)
{
    app_motor_drive_set_wheel_outputs(
        duty_percent,
        duty_percent,
        duty_percent,
        duty_percent);
}
