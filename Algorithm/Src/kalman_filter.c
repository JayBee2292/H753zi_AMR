#include "kalman_filter.h"

#include <stddef.h>

static float kalman_filter_positive_or_default(float value, float default_value)
{
    return (value > 0.0f) ? value : default_value;
}

void kalman_filter_init(
    kalman_filter_t *filter,
    float initial_value,
    float initial_uncertainty,
    float process_noise,
    float measurement_noise)
{
    if (filter == NULL) {
        return;
    }

    filter->x = initial_value;
    filter->p = kalman_filter_positive_or_default(initial_uncertainty, 1.0f);
    filter->q = kalman_filter_positive_or_default(process_noise, 0.01f);
    filter->r = kalman_filter_positive_or_default(measurement_noise, 0.05f);
    filter->is_initialized = true;
}

void kalman_filter_reset(kalman_filter_t *filter, float initial_value)
{
    if (filter == NULL) {
        return;
    }

    kalman_filter_init(filter, initial_value, filter->p, filter->q, filter->r);
}

float kalman_filter_update(kalman_filter_t *filter, float measurement)
{
    float gain;

    if (filter == NULL) {
        return measurement;
    }

    if (!filter->is_initialized) {
        kalman_filter_init(filter, measurement, 1.0f, 0.04f, 0.12f);
        return measurement;
    }

    filter->p += filter->q;
    gain = filter->p / (filter->p + filter->r);
    filter->x += gain * (measurement - filter->x);
    filter->p = (1.0f - gain) * filter->p;

    return filter->x;
}
