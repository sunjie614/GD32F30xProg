#ifndef TELEMETRY_MONITOR_H
#define TELEMETRY_MONITOR_H

#include <stdbool.h>

/* Serial-monitor state only. Never feed this filtered value into control. */
typedef struct
{
    float speed_rpm;
    float alpha;
    bool speed_initialized;
} TelemetryMonitorState_t;

void TelemetryMonitor_Init(TelemetryMonitorState_t* state,
                           float sample_time_s,
                           float time_constant_s);
float TelemetryMonitor_FilterSpeed(TelemetryMonitorState_t* state,
                                   float raw_speed_rpm);
float TelemetryMonitor_AngleErrorDeg(float measured_rad,
                                     float estimated_rad);

#endif
