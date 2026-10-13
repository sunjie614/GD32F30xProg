#ifndef SENSORLESS_MODE_H
#define SENSORLESS_MODE_H

/* Keep the legacy enum names and numeric values in both A2L backends.
 * FLYING=0 is an enum value, not a usable bit mask. */
typedef enum {
    FLYING = 0,
    LES_OBSERVER = 1,
    HF_INJECTION = 2
} sensorless_method_t;

#endif
