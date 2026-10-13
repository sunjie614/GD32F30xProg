#include <math.h>
#include <stdio.h>
/* Test the real private solver and accumulator without a synthetic duplicate. */
#include "../Application/Src/identification_iq.c"

static int close_relative(mc_real_t actual, float expected)
{
    return fabsf(McMath_ToFloat(actual) - expected) <= fabsf(expected) * 0.01F;
}

int main(void)
{
    IdentificationIqPoint_t points[7] = {0};
    mc_real_t linear, power, ssr, r2;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        float flux = 0.01F * (float)(i + 1U);
        points[i] = (IdentificationIqPoint_t){
            .maximum_flux_pu = McMath_FromFloat(flux),
            .current_pu = McMath_FromFloat(0.5F * flux + 5.0F * flux * flux),
            .valid = true};
    }
    if (!identification_fit_axis(points, 4U, 1U, &linear, &power, &ssr, &r2)
        || !close_relative(linear, 0.5F) || !close_relative(power, 5.0F))
    {
        fprintf(stderr, "FAIL low-flux Q fit: linear=%g power=%g\n",
                McMath_ToFloat(linear), McMath_ToFloat(power));
        return 1;
    }
    for (unsigned i = 0U; i < 7U; ++i)
    {
        float flux = 0.1F * (float)(i + 1U);
        points[i] = (IdentificationIqPoint_t){
            .maximum_flux_pu = McMath_FromFloat(flux),
            .current_pu = McMath_FromFloat(0.2F * flux + 2.0F * powf(flux, 6.0F)),
            .valid = true};
    }
    if (!identification_fit_axis(points, 7U, 5U, &linear, &power, &ssr, &r2)
        || !close_relative(linear, 0.2F) || !close_relative(power, 2.0F)) return 2;
    IdentificationIqState_t state;
    IdentificationIq_Init(&state, NULL);
    state.config.max_steps = 20U;
    state.config.current_step_pu = McMath_FromFloat(2.0F / MC_CURRENT_BASE_A);
    state.config.current_final_pu = McMath_FromFloat(12.0F / MC_CURRENT_BASE_A);
    state.current_target_pu = McMath_FromFloat(11.0F / MC_CURRENT_BASE_A);
    if (!identification_current_series_complete(&state)) return 9;
    state.config.current_step_pu = McMath_FromFloat(-2.0F / MC_CURRENT_BASE_A);
    state.config.current_final_pu = McMath_FromFloat(3.0F / MC_CURRENT_BASE_A);
    state.current_target_pu = McMath_FromFloat(4.0F / MC_CURRENT_BASE_A);
    if (!identification_current_series_complete(&state)) return 10;
    state.sample_count = 8U;
    state.coefficients.ad0 = MC_CONST(0.2);
    state.coefficients.aq0 = MC_CONST(0.3);
    for (unsigned i = 0U; i < 8U; ++i)
    {
        float d = i < 4U ? -0.03F * (float)(4U - i)
                          : 0.03F * (float)(i - 3U);
        float q = 0.75F * d;
        float x1 = 0.5F * d * d * q * q;
        float x2 = d * d * d * q / 3.0F;
        state.flux_d[i] = McMath_FromFloat(d);
        state.flux_q[i] = McMath_FromFloat(q);
        state.current_d[i] = McMath_FromFloat(0.2F * d + 10.0F * x1);
        state.current_q[i] = McMath_FromFloat(0.3F * q + 10.0F * x2);
    }
    identification_process_dq(&state);
    identification_finish_dq(&state);
    if (!state.coefficients.valid || !close_relative(state.coefficients.adq, 10.0F))
    {
        fprintf(stderr, "FAIL Q48 DQ fit: adq=%g state=%u\n",
            McMath_ToFloat(state.coefficients.adq), (unsigned)state.state);
        return 3;
    }
    if (McMath_Diagnostics.saturation_count || McMath_Diagnostics.divide_by_zero_count
        || McMath_Diagnostics.invalid_input_count) return 4;
#if defined(MC_NUMERIC_IQMATH)
    if (!identification_accum_ratio(INT64_C(1000000000000000000),
        INT64_C(2000000000000000000), &linear) || linear != MC_HALF) return 5;
    if (!identification_accum_ratio(-INT64_C(1000000000000000000),
        INT64_C(2000000000000000000), &linear) || linear != -MC_HALF) return 6;
    if (identification_accum_ratio(128, 1, &linear)) return 7;
    mc_accum_t sum = INT64_MAX;
    if (identification_accumulate_product(&sum, MC_ONE, MC_ONE)) return 8;
#endif
    printf("PASS: normalized low-flux fit and Q48 DQ accumulation adq=%g\n",
        McMath_ToFloat(state.coefficients.adq));
    return 0;
}
