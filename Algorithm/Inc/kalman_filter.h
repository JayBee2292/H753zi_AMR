#ifndef STM_MICROROS_KALMAN_FILTER_H
#define STM_MICROROS_KALMAN_FILTER_H

#include <stdbool.h>

typedef struct {
    float x;
    float p;
    float q;
    float r;
    bool is_initialized;
} kalman_filter_t;

void kalman_filter_init(
    kalman_filter_t *filter,
    float initial_value,
    float initial_uncertainty,
    float process_noise,
    float measurement_noise);

void kalman_filter_reset(kalman_filter_t *filter, float initial_value);

float kalman_filter_update(kalman_filter_t *filter, float measurement);

#endif //STM_MICROROS_KALMAN_FILTER_H
