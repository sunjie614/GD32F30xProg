#include "identification_iq.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>
#include "fixed_numeric_config.h"
#include "parameters.h"

static mc_real_t identification_average(mc_accum_t sum, uint32_t count)
{
    if (count == 0U)
    {
        McMath_Diagnostics.divide_by_zero_count++;
        return MC_ZERO;
    }
#if defined(MC_NUMERIC_IQMATH)
    int64_t result = sum / (int64_t)count;
    if (result > INT32_MAX)
        return (mc_real_t)INT32_MAX;
    if (result < INT32_MIN)
        return (mc_real_t)INT32_MIN;
    return (mc_real_t)result;
#else
    return (mc_real_t)(sum / (mc_accum_t)count);
#endif
}

static mc_real_t identification_average_pair(mc_real_t accumulated_average,
                                              mc_real_t sample,
                                              uint16_t accumulated_count)
{
    mc_accum_t sum = (mc_accum_t)accumulated_average * accumulated_count
                   + sample;
    return identification_average(sum, (uint32_t)accumulated_count + 1U);
}

static mc_real_t identification_power(mc_real_t value, uint8_t exponent)
{
    mc_real_t result = MC_ONE;
    for (uint8_t power = 0U; power < exponent; ++power)
        result = McMath_Mul(result, value);
    return result;
}

/* Keep matrix products in Q48 until division. Reducing each X*X to Q24
 * before summation discards legitimate small-flux samples. */
static bool identification_accumulate_product(mc_accum_t* sum,
                                               mc_real_t a, mc_real_t b)
{
#if defined(MC_NUMERIC_IQMATH)
    int64_t product = (int64_t)a * (int64_t)b;
    if ((product > 0 && *sum > INT64_MAX - product)
        || (product < 0 && *sum < INT64_MIN - product))
    {
        McMath_Diagnostics.saturation_count++;
        return false;
    }
#else
    mc_accum_t product = (mc_accum_t)a * (mc_accum_t)b;
#endif
    *sum += product;
    return true;
}

static bool identification_accum_ratio(mc_accum_t numerator,
                                        mc_accum_t denominator,
                                        mc_real_t* result)
{
    if (denominator <= 0)
        return false;
#if defined(MC_NUMERIC_IQMATH)
    bool negative = numerator < 0;
    uint64_t n = negative ? (uint64_t)(-(numerator + 1)) + 1U
                          : (uint64_t)numerator;
    uint64_t d = (uint64_t)denominator;
    uint64_t integer = n / d;
    uint64_t remainder = n % d;
    /* Bitwise long division avoids overflowing (numerator << 24). */
    if (integer >= 128U)
    {
        McMath_Diagnostics.saturation_count++;
        return false;
    }
    uint64_t raw = integer << MC_Q_FRACTIONAL_BITS;
    for (unsigned bit = 0U; bit < MC_Q_FRACTIONAL_BITS; ++bit)
    {
        bool one = remainder >= d - remainder;
        remainder = one ? remainder - (d - remainder) : remainder + remainder;
        if (one) raw |= (uint64_t)1 << (MC_Q_FRACTIONAL_BITS - bit - 1U);
    }
    if (remainder >= d - remainder) raw++;
    uint64_t maximum = negative ? (uint64_t)INT32_MAX + 1U : (uint64_t)INT32_MAX;
    if (raw > maximum)
    {
        McMath_Diagnostics.saturation_count++;
        return false;
    }
    *result = negative ? (mc_real_t)(-(int64_t)raw) : (mc_real_t)raw;
#else
    *result = (mc_real_t)(numerator / denominator);
#endif
    return true;
}

void IdentificationIq_DefaultConfig(IdentificationIqConfig_t* config)
{
    if (config == NULL)
        return;
    *config = (IdentificationIqConfig_t){
        .manual_control = false,
        .sample_capacity = IDENTIFICATION_IQ_SAMPLE_CAPACITY,
        .repeat_times = 3U,
        .max_steps = IDENTIFICATION_IQ_MAX_STEPS,
        .wait_edges = 3U,
        .injection_half_period_cycles = 250U,
        .rs_hold_cycles = 15000U,
        .current_start_pu = MC_CONST(3.0 / MC_CURRENT_BASE_A),
        .current_final_pu = MC_CONST(12.0 / MC_CURRENT_BASE_A),
        .dq_current_pu = MC_CONST(
            IDENTIFICATION_IQ_DQ_CURRENT_DEFAULT_A / MC_CURRENT_BASE_A),
        /* The default PU-only fixture has ten inclusive grid points. Derive
         * its step from the represented endpoints so constant-macro rounding
         * cannot move its tenth point beyond the represented 12 A endpoint.
         * The legacy A2L path instead uses exact whole-ampere enumeration. */
        .current_step_pu = (MC_CONST(12.0 / MC_CURRENT_BASE_A)
                         - MC_CONST(3.0 / MC_CURRENT_BASE_A)) / 9,
        .integer_current_scan = false,
        .current_start_a = 3,
        .current_final_a = 12,
        .current_step_a = 1,
        .current_limit_pu = MC_CONST(10.0 / MC_CURRENT_BASE_A),
        .rs_current_target_pu = MC_CONST(
            10.0 * 0.8 / MC_CURRENT_BASE_A),
        .rs_threshold_pu = MC_CONST(
            0.01 * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        .rs_voltage_step_pu = MC_CONST(1.0 / MC_VOLTAGE_BASE_V),
        .injection_voltage_pu = MC_CONST(100.0 / MC_VOLTAGE_BASE_V),
        .injection_voltage_d_pu = MC_CONST(100.0 / MC_VOLTAGE_BASE_V),
        .injection_voltage_q_pu = MC_CONST(100.0 / MC_VOLTAGE_BASE_V),
        .voltage_to_flux_step = MC_CONST(
            MAIN_LOOP_TIME * MC_VOLTAGE_BASE_V / MC_FLUX_BASE_WB)};
}

static bool identification_current_from_amperes(int32_t amperes,
                                                mc_real_t* result)
{
#if defined(MC_NUMERIC_IQMATH)
    /* The project contract uses an integer 30 A base. Use a 64-bit product
     * instead of float or accumulated Q24 steps; rounding error <= 0.5 LSB
     * for EVERY target, independently of the number/direction of steps. */
    _Static_assert(MC_CURRENT_BASE_A_INTEGER > 0
                   && MC_CURRENT_BASE_A_INTEGER <= INT32_MAX / 10000,
                   "Integer current base must support the delta threshold");
    const int32_t base = MC_CURRENT_BASE_A_INTEGER;
    int64_t raw = (int64_t)amperes * INT64_C(16777216);
    raw = (raw + (raw >= 0 ? base / 2 : -(base / 2))) / base;
    if (raw > INT32_MAX || raw < INT32_MIN)
        return false;
    *result = (mc_real_t)raw;
#else
    *result = (mc_real_t)amperes / MC_CURRENT_BASE_A;
#endif
    return true;
}

static bool identification_set_scan_current(IdentificationIqState_t* state,
                                              int64_t amperes)
{
    if (amperes < INT32_MIN || amperes > INT32_MAX
        || !identification_current_from_amperes((int32_t)amperes,
                                                 &state->current_target_pu))
    {
        state->error = IDENTIFICATION_IQ_INVALID_CONFIG;
        state->state = IDENTIFICATION_IQ_FAILED;
        return false;
    }
    state->current_target_a = (int32_t)amperes;
    return true;
}

static bool identification_start_current_series(IdentificationIqState_t* state)
{
    if (state->config.integer_current_scan)
    {
        int32_t start = state->config.current_start_a;
        int32_t final = state->config.current_final_a;
        int32_t step = state->config.current_step_a;
        mc_real_t ignored;
        if (step == 0 || (start < final && step < 0)
            || (start > final && step > 0)
            || !identification_current_from_amperes(final, &ignored))
        {
            state->error = IDENTIFICATION_IQ_INVALID_CONFIG;
            state->state = IDENTIFICATION_IQ_FAILED;
            return false;
        }
        return identification_set_scan_current(state, state->config.current_start_a);
    }
    state->current_target_pu = state->config.current_start_pu;
    return true;
}

static bool identification_set_dq_current(IdentificationIqState_t* state)
{
    /* This is a threshold magnitude, independent of the D/Q scan direction.
     * The gateway checks physical-unit inputs before quantization; direct
     * core callers must also provide a strictly positive represented value. */
    if (!(state->config.dq_current_pu > MC_ZERO))
    {
        state->error = IDENTIFICATION_IQ_INVALID_CONFIG;
        state->state = IDENTIFICATION_IQ_FAILED;
        return false;
    }
    state->current_target_pu = state->config.dq_current_pu;
    return true;
}

static bool identification_advance_current(IdentificationIqState_t* state)
{
    if (state->config.integer_current_scan)
        return identification_set_scan_current(state,
            (int64_t)state->current_target_a + state->config.current_step_a);
    state->current_target_pu = McMath_Add(state->current_target_pu,
                                         state->config.current_step_pu);
    return true;
}

static void identification_prepare_reset_state(IdentificationIqState_t* state)
{
    if (state->config.sample_capacity == 0U
        || state->config.sample_capacity > IDENTIFICATION_IQ_SAMPLE_CAPACITY
        || state->config.max_steps == 0U
        || state->config.max_steps > IDENTIFICATION_IQ_MAX_STEPS
        || state->config.repeat_times == 0U)
    {
        state->error = IDENTIFICATION_IQ_INVALID_CONFIG;
        state->state = IDENTIFICATION_IQ_FAILED;
        return;
    }
    if (state->config.integer_current_scan)
    {
        mc_real_t ignored;
        int32_t start = state->config.current_start_a;
        int32_t final = state->config.current_final_a;
        int32_t step = state->config.current_step_a;
        if (step == 0 || (start < final && step < 0)
            || (start > final && step > 0)
            || !identification_current_from_amperes(start, &ignored)
            || !identification_current_from_amperes(final, &ignored))
        {
            state->error = IDENTIFICATION_IQ_INVALID_CONFIG;
            state->state = IDENTIFICATION_IQ_FAILED;
            return;
        }
    }
    state->state = IDENTIFICATION_IQ_WAIT;
    if (!state->config.manual_control && !identification_start_current_series(state))
        return;
    state->rs_voltage_pu = state->config.rs_voltage_step_pu;
    state->rs_first_sample = true;
}

void IdentificationIq_Init(IdentificationIqState_t* state,
                           const IdentificationIqConfig_t* config)
{
    if (state == NULL)
        return;
    *state = (IdentificationIqState_t){0};
    if (config != NULL)
        state->config = *config;
    else
        IdentificationIq_DefaultConfig(&state->config);
    identification_prepare_reset_state(state);
}

void IdentificationIq_Reset(IdentificationIqState_t* state)
{
    if (state == NULL)
        return;
    /* Like the legacy Experiment flow, invalidate captured data instead of
     * erasing six 512-point arrays inside the 5 kHz control interrupt.
     * Clear every result/valid flag and accumulator, preserving configuration.
     * Capture overwrites each sample before increasing sample_count and starts
     * flux[0] from zero, so stale payload cannot enter a new experiment. */
    unsigned char* bytes = (unsigned char*)state;
    memset(bytes, 0, offsetof(IdentificationIqState_t, config));
    memset(bytes + offsetof(IdentificationIqState_t, coefficients), 0,
        offsetof(IdentificationIqState_t, voltage_d)
        - offsetof(IdentificationIqState_t, coefficients));
    memset(bytes + offsetof(IdentificationIqState_t, resistance_pu), 0,
        sizeof(*state) - offsetof(IdentificationIqState_t, resistance_pu));
    identification_prepare_reset_state(state);
}

void IdentificationIq_Start(IdentificationIqState_t* state)
{
    if (state == NULL || state->error != IDENTIFICATION_IQ_NO_ERROR)
        return;
    state->start_requested = true;
}

static void identification_reset_capture(IdentificationIqState_t* state)
{
    state->sample_count = 0U;
    state->edge_count = 0U;
    state->cycle_count = 0U;
    state->injection_positive = true;
    state->injection_d_positive = true;
    state->injection_q_positive = true;
    state->accumulated_flux = MC_ZERO;
    state->accumulated_current = MC_ZERO;
    state->last_voltage_d_pu = MC_ZERO;
    state->last_voltage_q_pu = MC_ZERO;
}

static void identification_run_rs(IdentificationIqState_t* state,
                                  const IdentificationIqInput_t* input,
                                  IdentificationIqOutput_t* output)
{
    output->voltage_d_pu = state->rs_voltage_pu;
    output->voltage_q_pu = MC_ZERO;
    state->rs_current_filtered_pu = McMath_Add(
        McMath_Mul(MC_CONST(0.9), state->rs_current_filtered_pu),
        McMath_Mul(MC_CONST(0.1), input->current_d_pu));
    /* Legacy Estimate_Rs evaluates on call HOLD_CYCLES + 1, not on the
     * HOLD_CYCLES-th call. Preserve the output-update timing as well. */
    if (state->rs_hold_count < state->config.rs_hold_cycles)
    {
        state->rs_hold_count++;
        return;
    }
    state->rs_hold_count = 0U;

    mc_real_t absolute_current = McMath_Abs(state->rs_current_filtered_pu);
    if (state->rs_first_sample)
    {
        state->rs_previous_voltage_pu = state->rs_voltage_pu;
        state->rs_previous_current_pu = state->rs_current_filtered_pu;
        state->rs_first_sample = false;
        state->rs_voltage_pu = McMath_Add(state->rs_voltage_pu,
                                          state->config.rs_voltage_step_pu);
        return;
    }
    mc_real_t current_delta = McMath_Sub(
        state->rs_current_filtered_pu, state->rs_previous_current_pu);
    /* Directed rounding expresses the old |delta_I| < 1e-4 A test without
     * losing the last Q24 value strictly below that physical threshold. */
#if defined(MC_NUMERIC_IQMATH)
    const int32_t epsilon_denominator = MC_CURRENT_BASE_A_INTEGER * 10000;
    const mc_real_t delta_threshold = (mc_real_t)(
        (INT64_C(16777216) + epsilon_denominator - 1) / epsilon_denominator);
#else
    const mc_real_t delta_threshold = MC_CONST(0.0001 / MC_CURRENT_BASE_A);
#endif
    if (McMath_Abs(current_delta) < delta_threshold)
    {
        state->rs_first_sample = true;
        state->rs_estimation_step = 0U;
        return;
    }
    mc_real_t estimate = McMath_Div(
        McMath_Sub(state->rs_voltage_pu, state->rs_previous_voltage_pu),
        current_delta);
    mc_real_t change = McMath_Abs(McMath_Sub(
        estimate, state->rs_previous_pu));
    /* Keep invalid/divide-by-zero or clipped arithmetic as an explicit
     * failure rather than treating old done_flag=2/4 as a successful fit. */
    if (estimate == MC_ZERO
#if defined(MC_NUMERIC_IQMATH)
        || estimate == INT32_MIN || estimate == INT32_MAX
#else
        || !isfinite(estimate)
#endif
        )
    {
        state->error = IDENTIFICATION_IQ_RS_NOT_CONVERGED;
        state->state = IDENTIFICATION_IQ_FAILED;
        output->voltage_d_pu = MC_ZERO;
        return;
    }
    if (state->rs_estimation_step > 0U
        && change < state->config.rs_threshold_pu
        && absolute_current >= state->config.rs_current_target_pu)
    {
        state->resistance_pu = estimate;
        state->rs_first_sample = true;
        state->rs_estimation_step = 0U;
        state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        state->injection_mode = IDENTIFICATION_IQ_INJECT_D;
        identification_reset_capture(state);
        return;
    }
    if (absolute_current > state->config.current_limit_pu
        || ++state->rs_estimation_step > IDENTIFICATION_IQ_MAX_STEPS)
    {
        state->error = IDENTIFICATION_IQ_RS_NOT_CONVERGED;
        state->state = IDENTIFICATION_IQ_FAILED;
        state->rs_first_sample = true;
        state->rs_estimation_step = 0U;
        output->voltage_d_pu = MC_ZERO;
        return;
    }
    mc_real_t predicted_current = McMath_Add(state->rs_previous_current_pu,
        McMath_Div(state->config.rs_voltage_step_pu, estimate));
    if (McMath_Abs(predicted_current) < state->config.rs_current_target_pu)
    {
        state->rs_previous_voltage_pu = state->rs_voltage_pu;
        state->rs_previous_current_pu = state->rs_current_filtered_pu;
        state->rs_previous_pu = estimate;
        state->rs_voltage_pu = McMath_Add(
            state->rs_voltage_pu, state->config.rs_voltage_step_pu);
        output->voltage_d_pu = state->rs_voltage_pu;
    }
}

static void identification_capture(IdentificationIqState_t* state,
                                   const IdentificationIqInput_t* input,
                                   IdentificationIqOutput_t* output)
{
    bool previous_d_sign = state->injection_d_positive;
    bool previous_q_sign = state->injection_q_positive;
    if (state->injection_mode != IDENTIFICATION_IQ_INJECT_Q)
    {
        if (input->current_d_pu >= state->current_target_pu)
            state->injection_d_positive = false;
        else if (input->current_d_pu <= McMath_Neg(state->current_target_pu))
            state->injection_d_positive = true;
    }
    if (state->injection_mode != IDENTIFICATION_IQ_INJECT_D)
    {
        if (input->current_q_pu >= state->current_target_pu)
            state->injection_q_positive = false;
        else if (input->current_q_pu <= McMath_Neg(state->current_target_pu))
            state->injection_q_positive = true;
    }
    state->cycle_count++;
    if (!state->config.manual_control
        && state->cycle_count >= state->config.injection_half_period_cycles)
    {
        state->cycle_count = 0U;
        if (state->injection_mode != IDENTIFICATION_IQ_INJECT_Q)
            state->injection_d_positive = !state->injection_d_positive;
        if (state->injection_mode != IDENTIFICATION_IQ_INJECT_D)
            state->injection_q_positive = !state->injection_q_positive;
    }
    bool d_edge = previous_d_sign != state->injection_d_positive;
    bool q_edge = previous_q_sign != state->injection_q_positive;
    bool measurement_edge = state->injection_mode == IDENTIFICATION_IQ_INJECT_Q
                          ? q_edge : d_edge;
    if (measurement_edge)
    {
        state->edge_count++;
        state->cycle_count = 0U;
    }
    state->injection_positive = state->injection_d_positive;
    mc_real_t voltage_d = state->config.manual_control
                          ? state->config.injection_voltage_d_pu
                          : state->config.injection_voltage_pu;
    mc_real_t voltage_q = state->config.manual_control
                          ? state->config.injection_voltage_q_pu
                          : state->config.injection_voltage_pu;
    output->voltage_d_pu = state->injection_mode == IDENTIFICATION_IQ_INJECT_Q
        ? MC_ZERO : (state->injection_d_positive
        ? voltage_d : McMath_Neg(voltage_d));
    output->voltage_q_pu = state->injection_mode == IDENTIFICATION_IQ_INJECT_D
        ? MC_ZERO : (state->injection_q_positive
        ? voltage_q : McMath_Neg(voltage_q));

    if (state->edge_count >= (uint16_t)(state->config.wait_edges + 3U))
    {
        state->state = IDENTIFICATION_IQ_PROCESS;
        return;
    }
    if (state->edge_count < (uint16_t)(state->config.wait_edges + 1U))
        return;
    if (state->sample_count >= state->config.sample_capacity)
    {
        state->state = IDENTIFICATION_IQ_PROCESS;
        return;
    }

    uint16_t index = state->sample_count;
    state->voltage_d[index] = state->last_voltage_d_pu;
    state->voltage_q[index] = state->last_voltage_q_pu;
    state->current_d[index] = input->current_d_pu;
    state->current_q[index] = input->current_q_pu;
    if (index == 0U)
    {
        state->flux_d[0] = MC_ZERO;
        state->flux_q[0] = MC_ZERO;
    }
    else
    {
        mc_real_t emf_d = McMath_Sub(state->last_voltage_d_pu,
            McMath_Mul(state->resistance_pu, input->current_d_pu));
        mc_real_t emf_q = McMath_Sub(state->last_voltage_q_pu,
            McMath_Mul(state->resistance_pu, input->current_q_pu));
        state->flux_d[index] = McMath_Add(state->flux_d[index - 1U],
            McMath_Mul(state->config.voltage_to_flux_step, emf_d));
        state->flux_q[index] = McMath_Add(state->flux_q[index - 1U],
            McMath_Mul(state->config.voltage_to_flux_step, emf_q));
    }
    state->sample_count++;
    if (state->sample_count >= state->config.sample_capacity)
        state->state = IDENTIFICATION_IQ_PROCESS;
}

static bool identification_process_axis(IdentificationIqState_t* state,
                                        bool d_axis)
{
    const mc_real_t* flux = d_axis ? state->flux_d : state->flux_q;
    const mc_real_t* current = d_axis ? state->current_d : state->current_q;
    mc_accum_t sum_flux = 0;
    for (uint16_t index = 0U; index < state->sample_count; ++index)
    {
        sum_flux += flux[index];
    }
    mc_real_t mean_flux = identification_average(sum_flux,
                                                  state->sample_count);
#if defined(MC_NUMERIC_IQMATH)
    mc_real_t maximum_flux = (mc_real_t)INT32_MIN;
#else
    mc_real_t maximum_flux = -INFINITY;
#endif
    mc_real_t current_at_maximum = MC_ZERO;
    for (uint16_t index = 0U; index < state->sample_count; ++index)
    {
        mc_real_t centered = McMath_Sub(flux[index], mean_flux);
        if (centered > maximum_flux)
        {
            maximum_flux = centered;
            current_at_maximum = current[index];
        }
    }
    IdentificationIqPoint_t sample = {
        .current_pu = current_at_maximum,
        .maximum_flux_pu = maximum_flux,
        .valid = state->sample_count >= 2U};
    if (!sample.valid)
        return false;
    IdentificationIqPoint_t* point = d_axis
        ? &state->d_results[state->step_index]
        : &state->q_results[state->step_index];
    point->current_pu = identification_average_pair(
        point->current_pu, sample.current_pu, state->repeat_count);
    point->maximum_flux_pu = identification_average_pair(
        point->maximum_flux_pu, sample.maximum_flux_pu, state->repeat_count);
    point->valid = true;
    return true;
}

static void identification_process_dq(IdentificationIqState_t* state)
{
    mc_accum_t flux_d_sum = 0;
    mc_accum_t flux_q_sum = 0;
    for (uint16_t index = 0U; index < state->sample_count; ++index)
    {
        flux_d_sum += state->flux_d[index];
        flux_q_sum += state->flux_q[index];
    }
    mc_real_t flux_d_mean = identification_average(
        flux_d_sum, state->sample_count);
    mc_real_t flux_q_mean = identification_average(
        flux_q_sum, state->sample_count);
    for (uint16_t index = 0U; index < state->sample_count; ++index)
    {
        mc_real_t flux_d = McMath_Sub(state->flux_d[index], flux_d_mean);
        mc_real_t flux_q = McMath_Sub(state->flux_q[index], flux_q_mean);
        mc_real_t current_d = state->current_d[index];
        mc_real_t current_q = state->current_q[index];
        if (flux_d <= MC_ZERO || flux_q <= MC_ZERO
            || current_d <= MC_ZERO || current_q <= MC_ZERO)
            continue;
        mc_real_t predicted_d = McMath_Add(
            McMath_Mul(state->coefficients.ad0, flux_d),
            McMath_Mul(state->coefficients.add,
                       identification_power(flux_d, 6U)));
        mc_real_t predicted_q = McMath_Add(
            McMath_Mul(state->coefficients.aq0, flux_q),
            McMath_Mul(state->coefficients.aqq,
                       identification_power(flux_q, 2U)));
        mc_real_t residual_d = McMath_Sub(current_d, predicted_d);
        mc_real_t residual_q = McMath_Sub(current_q, predicted_q);
        mc_real_t x1 = McMath_Mul(MC_HALF, McMath_Mul(
            identification_power(flux_d, 2U),
            identification_power(flux_q, 2U)));
        mc_real_t x2 = McMath_Div(McMath_Mul(
            identification_power(flux_d, 3U), flux_q), MC_CONST(3.0));
        if (!identification_accumulate_product(&state->dq_sum_xx, x1, x1)
            || !identification_accumulate_product(&state->dq_sum_xx, x2, x2)
            || !identification_accumulate_product(&state->dq_sum_xy, x1, residual_d)
            || !identification_accumulate_product(&state->dq_sum_xy, x2, residual_q))
        {
            state->error = IDENTIFICATION_IQ_SINGULAR_LLS;
            state->state = IDENTIFICATION_IQ_FAILED;
            return;
        }
        state->dq_sum_id += current_d;
        state->dq_sum_id_squared += McMath_Mul(current_d, current_d);
        state->dq_sum_iq += current_q;
        state->dq_sum_iq_squared += McMath_Mul(current_q, current_q);
        state->dq_sum_residual_d_squared += McMath_Mul(residual_d, residual_d);
        state->dq_sum_residual_q_squared += McMath_Mul(residual_q, residual_q);
        state->dq_sample_count++;
    }
}

static bool identification_fit_axis(const IdentificationIqPoint_t* points,
                                    uint16_t count,
                                    uint8_t exponent,
                                    mc_real_t* coefficient_linear,
                                    mc_real_t* coefficient_power,
                                    mc_real_t* ssr,
                                    mc_real_t* r2)
{
    mc_accum_t sum_xx = 0;
    mc_accum_t sum_xp = 0;
    mc_accum_t sum_pp = 0;
    mc_accum_t sum_yx = 0;
    mc_accum_t sum_yp = 0;
    uint16_t valid_count = 0U;
    mc_real_t flux_scale = MC_ZERO;
    for (uint16_t index = 0U; index < count; ++index)
    {
        if (points[index].valid
            && points[index].maximum_flux_pu > flux_scale)
            flux_scale = points[index].maximum_flux_pu;
    }
    if (flux_scale <= MC_ZERO)
        return false;

    /* Normalize BOTH matrix columns. Leaving a physical x in each column
     * makes the determinant proportional to scale^4 and destroys low-flux
     * observability even when the normalized matrix is well conditioned. */
    for (uint16_t index = 0U; index < count; ++index)
    {
        if (!points[index].valid)
            continue;
        mc_real_t x = McMath_Div(points[index].maximum_flux_pu, flux_scale);
        mc_real_t p = identification_power(x, (uint8_t)(exponent + 1U));
        mc_real_t y = points[index].current_pu;
        sum_xx += McMath_Mul(x, x);
        sum_xp += McMath_Mul(x, p);
        sum_pp += McMath_Mul(p, p);
        sum_yx += McMath_Mul(y, x);
        sum_yp += McMath_Mul(y, p);
        valid_count++;
    }
    if (valid_count < 2U)
        return false;
    mc_real_t sxx = identification_average(sum_xx, valid_count);
    mc_real_t sxp = identification_average(sum_xp, valid_count);
    mc_real_t spp = identification_average(sum_pp, valid_count);
    mc_real_t syx = identification_average(sum_yx, valid_count);
    mc_real_t syp = identification_average(sum_yp, valid_count);
    mc_real_t determinant = McMath_Sub(McMath_Mul(sxx, spp),
                                       McMath_Mul(sxp, sxp));
    if (McMath_Abs(determinant) <= MC_CONST(0.0000001))
        return false;
    mc_real_t coefficient_linear_scaled = McMath_Div(
        McMath_Sub(McMath_Mul(spp, syx), McMath_Mul(sxp, syp)),
        determinant);
    mc_real_t coefficient_scaled = McMath_Div(
        McMath_Sub(McMath_Mul(sxx, syp), McMath_Mul(sxp, syx)),
        determinant);
    mc_real_t scale_power = identification_power(flux_scale, (uint8_t)(exponent + 1U));
    if (scale_power == MC_ZERO)
        return false;
    *coefficient_linear = McMath_Div(coefficient_linear_scaled, flux_scale);
    *coefficient_power = McMath_Div(coefficient_scaled, scale_power);
#if defined(MC_NUMERIC_IQMATH)
    if (*coefficient_linear == INT32_MAX || *coefficient_linear == INT32_MIN
        || *coefficient_power == INT32_MAX || *coefficient_power == INT32_MIN)
        return false; /* A clipped coefficient is not a successful fit. */
#endif

    mc_accum_t sum_y = 0;
    for (uint16_t index = 0U; index < count; ++index)
        if (points[index].valid)
            sum_y += points[index].current_pu;
    mc_real_t mean_y = identification_average(sum_y, valid_count);
    mc_accum_t residual_sum = 0;
    mc_accum_t total_sum = 0;
    for (uint16_t index = 0U; index < count; ++index)
    {
        if (!points[index].valid)
            continue;
        mc_real_t x = McMath_Div(points[index].maximum_flux_pu, flux_scale);
        mc_real_t p = identification_power(x, (uint8_t)(exponent + 1U));
        mc_real_t predicted = McMath_Add(
            McMath_Mul(coefficient_linear_scaled, x),
            McMath_Mul(coefficient_scaled, p));
        mc_real_t residual = McMath_Sub(points[index].current_pu, predicted);
        mc_real_t deviation = McMath_Sub(points[index].current_pu, mean_y);
        residual_sum += McMath_Mul(residual, residual);
        total_sum += McMath_Mul(deviation, deviation);
    }
    *ssr = identification_average(residual_sum, valid_count);
    mc_real_t residual_mean = identification_average(residual_sum,
                                                      valid_count);
    mc_real_t total_mean = identification_average(total_sum, valid_count);
    *r2 = total_mean == MC_ZERO ? MC_ZERO : McMath_Sub(
        MC_ONE, McMath_Div(residual_mean, total_mean));
    return true;
}

static bool identification_fit_single_axes(IdentificationIqState_t* state)
{
    bool d_valid = identification_fit_axis(
        state->d_results, state->step_index, 5U,
        &state->coefficients.ad0, &state->coefficients.add,
        &state->coefficients.ssr_d, &state->coefficients.r2_d);
    bool q_valid = identification_fit_axis(
        state->q_results, state->step_index, 1U,
        &state->coefficients.aq0, &state->coefficients.aqq,
        &state->coefficients.ssr_q, &state->coefficients.r2_q);

    if (!d_valid || !q_valid)
    {
        state->error = IDENTIFICATION_IQ_SINGULAR_LLS;
        return false;
    }
    state->single_axis_fit_complete = true;
    return true;
}

static void identification_finish_dq(IdentificationIqState_t* state)
{
    if (state->dq_sample_count == 0U || state->dq_sum_xx == 0)
    {
        state->error = IDENTIFICATION_IQ_SINGULAR_LLS;
        state->state = IDENTIFICATION_IQ_FAILED;
        return;
    }
    uint16_t average_count = state->dq_sample_count > UINT16_MAX
                           ? UINT16_MAX : (uint16_t)state->dq_sample_count;
    if (!identification_accum_ratio(state->dq_sum_xy, state->dq_sum_xx,
                                     &state->coefficients.adq))
    {
        state->error = IDENTIFICATION_IQ_SINGULAR_LLS;
        state->state = IDENTIFICATION_IQ_FAILED;
        return;
    }
    mc_real_t mean_id = identification_average(
        state->dq_sum_id, average_count);
    mc_real_t mean_iq = identification_average(
        state->dq_sum_iq, average_count);
    mc_real_t variance_id = McMath_Sub(
        identification_average(state->dq_sum_id_squared, average_count),
        McMath_Mul(mean_id, mean_id));
    mc_real_t variance_iq = McMath_Sub(
        identification_average(state->dq_sum_iq_squared, average_count),
        McMath_Mul(mean_iq, mean_iq));
    state->coefficients.ssr_dq_d = identification_average(
        state->dq_sum_residual_d_squared, average_count);
    state->coefficients.ssr_dq_q = identification_average(
        state->dq_sum_residual_q_squared, average_count);
    state->coefficients.r2_dq_d = variance_id <= MC_ZERO ? MC_ZERO
        : McMath_Sub(MC_ONE, McMath_Div(
            state->coefficients.ssr_dq_d, variance_id));
    state->coefficients.r2_dq_q = variance_iq <= MC_ZERO ? MC_ZERO
        : McMath_Sub(MC_ONE, McMath_Div(
            state->coefficients.ssr_dq_q, variance_iq));
    state->coefficients.valid = true;
    state->state = IDENTIFICATION_IQ_DONE;
}

static bool identification_current_series_complete(
    const IdentificationIqState_t* state)
{
    if ((uint16_t)(state->step_index + 1U) >= state->config.max_steps)
        return true;
    if (state->config.integer_current_scan)
    {
        int64_t next = (int64_t)state->current_target_a
                     + state->config.current_step_a;
        return state->config.current_step_a > 0
            ? next > state->config.current_final_a
            : next < state->config.current_final_a;
    }
    if (state->config.current_step_pu >= MC_ZERO)
        return McMath_Add(state->current_target_pu, state->config.current_step_pu)
            > state->config.current_final_pu;
    return McMath_Add(state->current_target_pu, state->config.current_step_pu)
        < state->config.current_final_pu;
}

static bool identification_fit_manual_axis(IdentificationIqState_t* state)
{
    if (state->injection_mode == IDENTIFICATION_IQ_INJECT_D)
    {
        bool valid = identification_fit_axis(
            state->d_results, state->d_result_count, 5U,
            &state->coefficients.ad0, &state->coefficients.add,
            &state->coefficients.ssr_d, &state->coefficients.r2_d);
        if (!valid)
            return false;
        state->injection_mode = IDENTIFICATION_IQ_INJECT_Q;
    }
    else if (state->injection_mode == IDENTIFICATION_IQ_INJECT_Q)
    {
        bool valid = identification_fit_axis(
            state->q_results, state->q_result_count, 1U,
            &state->coefficients.aq0, &state->coefficients.aqq,
            &state->coefficients.ssr_q, &state->coefficients.r2_q);
        if (!valid)
            return false;
        state->single_axis_fit_complete = true;
        state->injection_mode = IDENTIFICATION_IQ_INJECT_DQ;
        state->dq_current_latched = false;
    }
    else
    {
        identification_finish_dq(state);
        return state->state == IDENTIFICATION_IQ_DONE;
    }
    state->step_index = 0U;
    state->repeat_count = 0U;
    state->current_target_pu = MC_ZERO;
    state->point_started = false;
    identification_reset_capture(state);
    state->state = IDENTIFICATION_IQ_PENDING;
    return true;
}

/* Compatibility workflow for the original Experiment.* A2L operation.
 * A current point starts only after injection_enable is written. Repeats at
 * that point are automatic; the next current point waits for a new write. */
static void identification_run_manual(IdentificationIqState_t* state,
                                      const IdentificationIqInput_t* input,
                                      IdentificationIqOutput_t* output)
{
    if (state->state == IDENTIFICATION_IQ_WAIT)
    {
        if (input->start_rs)
            state->state = IDENTIFICATION_IQ_EST_RS;
        else if (state->rs_complete && input->start_axis)
        {
            state->injection_mode = IDENTIFICATION_IQ_INJECT_D;
            state->step_index = 0U;
            state->repeat_count = 0U;
            if (!identification_start_current_series(state))
                return;
            state->point_started = false;
            identification_reset_capture(state);
            state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        }
    }

    switch (state->state)
    {
    case IDENTIFICATION_IQ_EST_RS:
        identification_run_rs(state, input, output);
        if (state->state == IDENTIFICATION_IQ_INJECT_COLLECT)
        {
            state->rs_complete = true;
            state->injection_mode = IDENTIFICATION_IQ_INJECT_D;
            state->current_target_pu = MC_ZERO;
            state->point_started = false;
            state->state = IDENTIFICATION_IQ_WAIT;
            output->voltage_d_pu = MC_ZERO;
            output->voltage_q_pu = MC_ZERO;
        }
        break;
    case IDENTIFICATION_IQ_INJECT_COLLECT:
        /* The original inj.State is a live per-cycle switch, not merely a
         * one-time start pulse. Pause freezes the captured payload and last
         * injection voltage; it does not silently keep applying voltage. */
        if (!input->injection_enable)
        {
            state->point_started = false;
            break;
        }
        state->point_started = true;
        identification_capture(state, input, output);
        if (state->state == IDENTIFICATION_IQ_PROCESS)
        {
            output->voltage_d_pu = MC_ZERO;
            output->voltage_q_pu = MC_ZERO;
        }
        break;
    case IDENTIFICATION_IQ_PROCESS:
        if (state->injection_mode == IDENTIFICATION_IQ_INJECT_DQ)
            identification_process_dq(state);
        else if (!identification_process_axis(
                     state, state->injection_mode == IDENTIFICATION_IQ_INJECT_D))
        {
            identification_reset_capture(state);
            state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
            break;
        }
        if (state->state == IDENTIFICATION_IQ_FAILED)
            break;
        state->repeat_count++;
        if (state->repeat_count < state->config.repeat_times)
        {
            identification_reset_capture(state);
            state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        }
        else if (state->injection_mode == IDENTIFICATION_IQ_INJECT_DQ)
        {
            state->point_started = false;
            state->state = IDENTIFICATION_IQ_LLS;
        }
        else
        {
            state->repeat_count = 0U;
            state->point_started = false;
            if (state->injection_mode == IDENTIFICATION_IQ_INJECT_D)
                state->d_result_count = (uint16_t)(state->step_index + 1U);
            else
                state->q_result_count = (uint16_t)(state->step_index + 1U);
            state->state = IDENTIFICATION_IQ_NEXT_CURRENT;
        }
        break;
    case IDENTIFICATION_IQ_NEXT_CURRENT:
        if (identification_current_series_complete(state))
            state->state = IDENTIFICATION_IQ_LLS;
        else
        {
            state->step_index++;
            if (!identification_advance_current(state))
                break;
            identification_reset_capture(state);
            state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        }
        break;
    case IDENTIFICATION_IQ_LLS:
        if (!identification_fit_manual_axis(state)
            && state->state != IDENTIFICATION_IQ_DONE)
        {
            state->error = IDENTIFICATION_IQ_SINGULAR_LLS;
            state->state = IDENTIFICATION_IQ_FAILED;
        }
        break;
    case IDENTIFICATION_IQ_PENDING:
        if (!(state->injection_mode == IDENTIFICATION_IQ_INJECT_DQ
              ? identification_set_dq_current(state)
              : identification_start_current_series(state)))
            break;
        identification_reset_capture(state);
        state->point_started = false;
        state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        break;
    default:
        break;
    }
}

void IdentificationIq_Run(IdentificationIqState_t* state,
                           const IdentificationIqInput_t* input,
                           IdentificationIqOutput_t* output)
{
    if (state == NULL || input == NULL || output == NULL)
        return;
    *output = (IdentificationIqOutput_t){0};
    if (input->reset)
    {
        IdentificationIq_Reset(state);
        return;
    }
    if (state->state == IDENTIFICATION_IQ_INJECT_COLLECT
        && state->injection_mode == IDENTIFICATION_IQ_INJECT_DQ
        && !state->dq_current_latched)
    {
        /* Refresh even while waiting with inj.State=0, so the operator sees
         * the accepted threshold before release. Once any injection starts,
         * hold it across pauses and repeats; never mix currents in one fit. */
        if (!identification_set_dq_current(state))
            return;
        if (!state->config.manual_control || input->injection_enable)
            state->dq_current_latched = true;
    }
    if (state->config.manual_control)
    {
        identification_run_manual(state, input, output);
        output->running = state->state != IDENTIFICATION_IQ_WAIT
                       && state->state != IDENTIFICATION_IQ_DONE
                       && state->state != IDENTIFICATION_IQ_FAILED;
        output->complete = state->state == IDENTIFICATION_IQ_DONE;
        output->valid = state->coefficients.valid;
        /* The legacy early-break pause holds last_Vd/last_Vq, even though
         * its external command is zero. A paused capture should be repeated
         * as a full current point when physical continuity is required. */
        if (state->state != IDENTIFICATION_IQ_INJECT_COLLECT
            || input->injection_enable)
        {
            state->last_voltage_d_pu = output->voltage_d_pu;
            state->last_voltage_q_pu = output->voltage_q_pu;
        }
        return;
    }
    if (state->state == IDENTIFICATION_IQ_WAIT && state->start_requested)
    {
        state->start_requested = false;
        state->state = IDENTIFICATION_IQ_EST_RS;
    }

    switch (state->state)
    {
    case IDENTIFICATION_IQ_EST_RS:
        identification_run_rs(state, input, output);
        break;
    case IDENTIFICATION_IQ_INJECT_COLLECT:
        identification_capture(state, input, output);
        break;
    case IDENTIFICATION_IQ_PROCESS:
        if (state->injection_mode == IDENTIFICATION_IQ_INJECT_DQ)
            identification_process_dq(state);
        else if (!identification_process_axis(
                     state, state->injection_mode == IDENTIFICATION_IQ_INJECT_D))
        {
            identification_reset_capture(state);
            state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
            break;
        }
        if (state->state == IDENTIFICATION_IQ_FAILED)
            break;
        state->repeat_count++;
        if (state->repeat_count < state->config.repeat_times)
        {
            identification_reset_capture(state);
            state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        }
        else if (state->injection_mode == IDENTIFICATION_IQ_INJECT_D)
        {
            state->repeat_count = 0U;
            state->injection_mode = IDENTIFICATION_IQ_INJECT_Q;
            identification_reset_capture(state);
            state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        }
        else if (state->injection_mode == IDENTIFICATION_IQ_INJECT_Q)
        {
            state->repeat_count = 0U;
            state->state = IDENTIFICATION_IQ_NEXT_CURRENT;
        }
        else
            identification_finish_dq(state);
        break;
    case IDENTIFICATION_IQ_NEXT_CURRENT:
    {
        bool series_complete = identification_current_series_complete(state);
        state->step_index++;
        if (series_complete)
        {
            state->state = IDENTIFICATION_IQ_LLS;
        }
        else
        {
            if (!identification_advance_current(state))
                break;
            state->injection_mode = IDENTIFICATION_IQ_INJECT_D;
            identification_reset_capture(state);
            state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        }
        break;
    }
    case IDENTIFICATION_IQ_LLS:
        if (identification_fit_single_axes(state))
            state->state = IDENTIFICATION_IQ_PENDING;
        else
            state->state = IDENTIFICATION_IQ_FAILED;
        break;
    case IDENTIFICATION_IQ_PENDING:
        state->injection_mode = IDENTIFICATION_IQ_INJECT_DQ;
        state->repeat_count = 0U;
        state->dq_current_latched = false;
        if (!identification_set_dq_current(state))
            break;
        identification_reset_capture(state);
        state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        break;
    default:
        break;
    }
    output->running = state->state != IDENTIFICATION_IQ_WAIT
                   && state->state != IDENTIFICATION_IQ_DONE
                   && state->state != IDENTIFICATION_IQ_FAILED;
    output->complete = state->state == IDENTIFICATION_IQ_DONE;
    output->valid = state->coefficients.valid;
    state->last_voltage_d_pu = output->voltage_d_pu;
    state->last_voltage_q_pu = output->voltage_q_pu;
}

bool IdentificationIq_GetMtpaParameters(
    const IdentificationIqState_t* state,
    MtpaIqParameters_t* parameters)
{
    if (state == NULL || parameters == NULL || !state->coefficients.valid)
        return false;
    MtpaIq_DefaultParameters(parameters);
    parameters->ad0 = state->coefficients.ad0;
    parameters->add = state->coefficients.add;
    parameters->aq0 = state->coefficients.aq0;
    parameters->aqq = state->coefficients.aqq;
    parameters->adq = state->coefficients.adq;
    return true;
}
