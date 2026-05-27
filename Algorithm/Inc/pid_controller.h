#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    bool has_previous_error;
} pid_controller_t;

void pid_controller_init(
    pid_controller_t *pid,
    float kp,
    float ki,
    float kd,
    float output_min,
    float output_max,
    float integral_min,
    float integral_max);

void pid_controller_reset(pid_controller_t *pid);

float pid_controller_update(
    pid_controller_t *pid,
    float setpoint,
    float measurement,
    float dt);

#endif /* PID_CONTROLLER_H */
