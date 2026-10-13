#ifndef FIXED_GATEWAY_TYPES_H
#define FIXED_GATEWAY_TYPES_H

#include <stdbool.h>

/* Physical-unit compatibility registers, never algorithm state. */
typedef struct
{
    volatile float Kp, Ki, Kd;
    volatile float integral, previous_error;
    volatile float MaxOutput, MinOutput, output, IntegralLimit, Ts;
    volatile bool Reset;
} A2lPidMirror_t;

#endif
