#include "mc_math.h"

#include <math.h>

McMathDiagnostics_t McMath_Diagnostics = {0};

void McMath_ResetDiagnostics(void)
{
    McMath_Diagnostics.saturation_count     = 0U;
    McMath_Diagnostics.divide_by_zero_count = 0U;
    McMath_Diagnostics.invalid_input_count  = 0U;
}

mc_real_t McMath_FromFloat(float value)
{
#if defined(MC_NUMERIC_IQMATH)
    if (!isfinite(value))
    {
        McMath_Diagnostics.invalid_input_count++;
        return MC_ZERO;
    }
    if (value >= 127.99999994F)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MAX;
    }
    if (value <= -128.0F)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MIN;
    }
    float scaled = value * 16777216.0F;
    return (mc_real_t)(scaled >= 0.0F ? scaled + 0.5F : scaled - 0.5F);
#else
    return value;
#endif
}

float McMath_ToFloat(mc_real_t value)
{
#if defined(MC_NUMERIC_IQMATH)
    return _IQ24toF(value);
#else
    return value;
#endif
}
