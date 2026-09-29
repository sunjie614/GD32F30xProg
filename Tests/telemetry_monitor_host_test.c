#include <math.h>
#include <stdio.h>
#include "telemetry_monitor.h"

static int close_to(float value, float expected, float tolerance)
{
    return fabsf(value - expected) <= tolerance;
}

int main(void)
{
    TelemetryMonitorState_t monitor;
    TelemetryMonitor_Init(&monitor, 0.0002F, 0.010F);
    if (!close_to(monitor.alpha, 1.0F / 51.0F, 1.0e-6F))
        return 1;
    if (!close_to(TelemetryMonitor_FilterSpeed(&monitor, 0.0F), 0.0F,
                  1.0e-6F))
        return 2;
    float filtered = TelemetryMonitor_FilterSpeed(&monitor, 1000.0F);
    if (!close_to(filtered, 1000.0F / 51.0F, 1.0e-3F))
        return 3;
    if (!close_to(TelemetryMonitor_FilterSpeed(&monitor, NAN), filtered,
                  1.0e-6F))
        return 4;
    if (!close_to(TelemetryMonitor_AngleErrorDeg(0.1F, 6.2F),
                  10.504F, 0.01F))
        return 5;
    if (!close_to(TelemetryMonitor_AngleErrorDeg(6.2F, 0.1F),
                  -10.504F, 0.01F))
        return 6;
    puts("PASS: serial-only estimated speed filter and angle error");
    return 0;
}
