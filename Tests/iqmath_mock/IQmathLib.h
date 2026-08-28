#ifndef TEST_IQMATHLIB_H
#define TEST_IQMATHLIB_H

#include <math.h>
#include <stdint.h>

typedef int32_t _iq24;

#define _IQ24(value) ((_iq24)((value) >= 0.0 \
    ? (value) * 16777216.0 + 0.5 : (value) * 16777216.0 - 0.5))

static inline _iq24 _IQ24mpy(_iq24 a, _iq24 b)
{
    return (_iq24)(((int64_t)a * b) >> 24);
}

static inline _iq24 _IQ24rmpy(_iq24 a, _iq24 b)
{
    int64_t product = (int64_t)a * b;
    int64_t magnitude = product < 0 ? -product : product;
    int64_t rounded = (magnitude + ((int64_t)1 << 23)) >> 24;
    return (_iq24)(product < 0 ? -rounded : rounded);
}

static inline _iq24 _IQ24div(_iq24 numerator, _iq24 denominator)
{
    return (_iq24)(((int64_t)numerator * ((int64_t)1 << 24)) / denominator);
}

static inline _iq24 _IQ24sinPU(_iq24 angle)
{
    return _IQ24(sin((double)angle / 16777216.0 * 6.283185307179586));
}

static inline _iq24 _IQ24cosPU(_iq24 angle)
{
    return _IQ24(cos((double)angle / 16777216.0 * 6.283185307179586));
}

static inline _iq24 _IQ24atan2PU(_iq24 y, _iq24 x)
{
    double angle = atan2((double)y, (double)x) / 6.283185307179586;
    if (angle < 0.0)
        angle += 1.0;
    return _IQ24(angle);
}

static inline _iq24 _IQ24sqrt(_iq24 value)
{
    return _IQ24(sqrt((double)value / 16777216.0));
}

static inline float _IQ24toF(_iq24 value)
{
    return (float)value / 16777216.0F;
}

#endif
