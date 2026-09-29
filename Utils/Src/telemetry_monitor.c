#include "telemetry_monitor.h"

#include <math.h>
#include <stddef.h>

#define TELEMETRY_PI_F (3.14159265358979323846F)
#define TELEMETRY_TWO_PI_F (2.0F * TELEMETRY_PI_F)

void TelemetryMonitor_Init(TelemetryMonitorState_t* state,
                           float sample_time_s,
                           float time_constant_s)
{
    if (state == NULL)
        return;
    *state = (TelemetryMonitorState_t){0};
    if (isfinite(sample_time_s) && sample_time_s > 0.0F
        && isfinite(time_constant_s) && time_constant_s >= 0.0F)
    {
        state->alpha = sample_time_s / (time_constant_s + sample_time_s);
    }
    else
        state->alpha = 1.0F;
}

float TelemetryMonitor_FilterSpeed(TelemetryMonitorState_t* state,
                                   float raw_speed_rpm)
{
    if (state == NULL)
        return raw_speed_rpm;
    if (!isfinite(raw_speed_rpm))
        return state->speed_initialized ? state->speed_rpm : 0.0F;
    if (!state->speed_initialized)
    {
        state->speed_rpm = raw_speed_rpm;
        state->speed_initialized = true;
    }
    else
        state->speed_rpm += state->alpha * (raw_speed_rpm - state->speed_rpm);
    return state->speed_rpm;
}

float TelemetryMonitor_AngleErrorDeg(float measured_rad,
                                     float estimated_rad)
{
    if (!isfinite(measured_rad) || !isfinite(estimated_rad))
        return 0.0F;
    /* Both angle producers publish [0, 2*pi), so one wrap is sufficient.
     * Keep the 5 kHz telemetry path free of a libm remainder call. */
    float error = measured_rad - estimated_rad;
    if (error >= TELEMETRY_PI_F)
        error -= TELEMETRY_TWO_PI_F;
    else if (error < -TELEMETRY_PI_F)
        error += TELEMETRY_TWO_PI_F;
    return error * (180.0F / TELEMETRY_PI_F);
}
