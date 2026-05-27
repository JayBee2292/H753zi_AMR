#include "pid_controller.h"

#include <stddef.h>

static float pid_controller_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

void pid_controller_init(
    pid_controller_t *pid,
    float kp,
    float ki,
    float kd,
    float output_min,
    float output_max,
    float integral_min,
    float integral_max)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->has_previous_error = false;
}

void pid_controller_reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous_error = false;
}

float pid_controller_update(
    pid_controller_t *pid,
    float setpoint,
    float measurement,
    float dt)
{
    float error;
    float derivative = 0.0f;
    float output;

    if (pid == NULL || dt <= 0.0f) {
        return 0.0f;
    }

    error = setpoint - measurement;
    pid->integral += error * dt;
    pid->integral = pid_controller_clamp(
        pid->integral,
        pid->integral_min,
        pid->integral_max);

    if (pid->has_previous_error) {
        derivative = (error - pid->previous_error) / dt;
    } else {
        pid->has_previous_error = true;
    }

    pid->previous_error = error;
    output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative;

    return pid_controller_clamp(output, pid->output_min, pid->output_max);
}
