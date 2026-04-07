#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include "calc_input_data.h"

typedef struct {
    TIM_HandleTypeDef *front_left;
    TIM_HandleTypeDef *front_right;
    TIM_HandleTypeDef *rear_left;
    TIM_HandleTypeDef *rear_right;
} app_robot_timer_map_t;

extern robot_status_t g_robot_state;

robot_status_t *app_robot_get_state(void);
bool app_robot_init(const app_robot_timer_map_t *timer_map);
void app_robot_reset_measurements(robot_status_t *robot);
void app_robot_update(robot_status_t *robot, float dt);
void app_robot_handle_encoder_overflow(TIM_HandleTypeDef *htim);

#endif
