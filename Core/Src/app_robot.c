#include "app_robot.h"

#include <string.h>

#define APP_ROBOT_WHEEL_PPR 244000U
#define APP_ROBOT_WHEEL_DIAMETER_M 0.21f

robot_status_t g_robot_state;

static wheel_state_t app_robot_make_wheel(bool is_inverted_direction)
{
    wheel_state_t wheel = {0};

    wheel.ppr = APP_ROBOT_WHEEL_PPR;
    wheel.diameter_m = APP_ROBOT_WHEEL_DIAMETER_M;
    wheel.is_inverted_direction = is_inverted_direction;

    return wheel;
}

static void app_robot_load_defaults(robot_status_t *robot)
{
    if (robot == NULL) {
        return;
    }

    memset(robot, 0, sizeof(*robot));

    robot->wheels[0] = app_robot_make_wheel(true);
    robot->wheels[1] = app_robot_make_wheel(false);
    robot->wheels[2] = app_robot_make_wheel(true);
    robot->wheels[3] = app_robot_make_wheel(false);
}

static bool app_robot_start_encoder(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) {
        return false;
    }

    if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK) {
        return false;
    }

    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
    return true;
}

robot_status_t *app_robot_get_state(void)
{
    return &g_robot_state;
}

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

    for (uint32_t i = 0; i < WHEEL_COUNT; ++i) {
        if (!app_robot_start_encoder(g_robot_state.wheels[i].htim)) {
            return false;
        }
    }

    app_robot_reset_measurements(&g_robot_state);
    g_robot_state.is_system_ready = true;

    return true;
}

void app_robot_reset_measurements(robot_status_t *robot)
{
    if (robot == NULL) {
        return;
    }

    robot->robot_linear_v = 0.0f;
    robot->robot_angular_w = 0.0f;

    for (uint32_t i = 0; i < WHEEL_COUNT; ++i) {
        wheel_state_t *wheel = &robot->wheels[i];

        wheel->overflow_cnt = 0;
        wheel->velocity_mps = 0.0f;
        wheel->total_distance_m = 0.0f;
        wheel->is_fault = false;
        wheel->total_tick = get_total_encoder_tick(wheel);
        wheel->prev_tick = wheel->total_tick;
    }
}

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

void app_robot_handle_encoder_overflow(TIM_HandleTypeDef *htim)
{
    encoder_overflow_callback(&g_robot_state, htim);
}
