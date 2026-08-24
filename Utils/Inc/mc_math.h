#ifndef MC_MATH_H
#define MC_MATH_H

#include <stdbool.h>
#include <stdint.h>

#if defined(MC_NUMERIC_IQMATH)
#include "IQmathLib.h"
typedef _iq24 mc_real_t;
_Static_assert(sizeof(mc_real_t) == sizeof(int32_t),
               "IQ24 storage must be a 32-bit signed value");
#define MC_CONST(value) ((_iq24)_IQ24(value))
#define MC_ZERO         ((_iq24)0)
#define MC_ONE          MC_CONST(1.0)
#define MC_HALF         MC_CONST(0.5)
#define MC_Q_FRACTIONAL_BITS 24
#else
#include <math.h>
typedef float mc_real_t;
#define MC_CONST(value) ((float)(value))
#define MC_ZERO         (0.0F)
#define MC_ONE          (1.0F)
#define MC_HALF         (0.5F)
#define MC_Q_FRACTIONAL_BITS 0
#endif

typedef struct
{
    volatile uint32_t saturation_count;
    volatile uint32_t divide_by_zero_count;
    volatile uint32_t invalid_input_count;
} McMathDiagnostics_t;

extern McMathDiagnostics_t McMath_Diagnostics;

void McMath_ResetDiagnostics(void);
mc_real_t McMath_FromFloat(float value);
float McMath_ToFloat(mc_real_t value);

static inline mc_real_t McMath_Add(mc_real_t a, mc_real_t b)
{
#if defined(MC_NUMERIC_IQMATH)
    int64_t sum = (int64_t)a + (int64_t)b;
    if (sum > INT32_MAX)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MAX;
    }
    if (sum < INT32_MIN)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MIN;
    }
    return (mc_real_t)sum;
#else
    return a + b;
#endif
}

static inline mc_real_t McMath_Sub(mc_real_t a, mc_real_t b)
{
#if defined(MC_NUMERIC_IQMATH)
    int64_t difference = (int64_t)a - (int64_t)b;
    if (difference > INT32_MAX)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MAX;
    }
    if (difference < INT32_MIN)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MIN;
    }
    return (mc_real_t)difference;
#else
    return a - b;
#endif
}

static inline mc_real_t McMath_Neg(mc_real_t value)
{
#if defined(MC_NUMERIC_IQMATH)
    if (value == (mc_real_t)INT32_MIN)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MAX;
    }
#endif
    return -value;
}

static inline mc_real_t McMath_Abs(mc_real_t value)
{
    return value < MC_ZERO ? McMath_Neg(value) : value;
}

static inline mc_real_t McMath_Mul(mc_real_t a, mc_real_t b)
{
#if defined(MC_NUMERIC_IQMATH)
    int64_t product = ((int64_t)a * (int64_t)b) >> 24;
    if (product > INT32_MAX)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MAX;
    }
    if (product < INT32_MIN)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MIN;
    }
    /* Use the supplied library for the actual in-range IQ24 operation. */
    return _IQ24mpy(a, b);
#else
    return a * b;
#endif
}

static inline mc_real_t McMath_Div(mc_real_t numerator,
                                   mc_real_t denominator)
{
    if (denominator == MC_ZERO)
    {
        McMath_Diagnostics.divide_by_zero_count++;
        return numerator < MC_ZERO ?
#if defined(MC_NUMERIC_IQMATH)
            (mc_real_t)INT32_MIN : (mc_real_t)INT32_MAX;
#else
            -INFINITY : INFINITY;
#endif
    }
#if defined(MC_NUMERIC_IQMATH)
    return _IQ24div(numerator, denominator);
#else
    return numerator / denominator;
#endif
}

static inline mc_real_t McMath_Clamp(mc_real_t value,
                                     mc_real_t minimum,
                                     mc_real_t maximum)
{
    if (value < minimum)
    {
        McMath_Diagnostics.saturation_count++;
        return minimum;
    }
    if (value > maximum)
    {
        McMath_Diagnostics.saturation_count++;
        return maximum;
    }
    return value;
}

/* Angles for these two functions are per-unit: 1 pu is one revolution. */
static inline mc_real_t McMath_SinPu(mc_real_t angle_pu)
{
#if defined(MC_NUMERIC_IQMATH)
    return _IQ24sinPU(angle_pu);
#else
    return sinf(angle_pu * 6.28318530717958647692F);
#endif
}

static inline mc_real_t McMath_CosPu(mc_real_t angle_pu)
{
#if defined(MC_NUMERIC_IQMATH)
    return _IQ24cosPU(angle_pu);
#else
    return cosf(angle_pu * 6.28318530717958647692F);
#endif
}

static inline mc_real_t McMath_Atan2Pu(mc_real_t y, mc_real_t x)
{
#if defined(MC_NUMERIC_IQMATH)
    return _IQ24atan2PU(y, x);
#else
    float angle = atan2f(y, x) * 0.15915494309189535F;
    return angle < 0.0F ? angle + 1.0F : angle;
#endif
}

static inline mc_real_t McMath_Sqrt(mc_real_t value)
{
    if (value < MC_ZERO)
    {
        McMath_Diagnostics.invalid_input_count++;
        return MC_ZERO;
    }
#if defined(MC_NUMERIC_IQMATH)
    return _IQ24sqrt(value);
#else
    return sqrtf(value);
#endif
}

#endif
