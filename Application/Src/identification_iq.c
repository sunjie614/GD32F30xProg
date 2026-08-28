#include "identification_iq.h"

#include <limits.h>
#include <stddef.h>
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

void IdentificationIq_DefaultConfig(IdentificationIqConfig_t* config)
{
    if (config == NULL)
        return;
    *config = (IdentificationIqConfig_t){
        .sample_capacity = IDENTIFICATION_IQ_SAMPLE_CAPACITY,
        .repeat_times = 3U,
        .max_steps = 10U,
        .wait_edges = 3U,
        .injection_half_period_cycles = 250U,
        .rs_hold_cycles = 15000U,
        .current_start_pu = MC_CONST(3.0 / MC_CURRENT_BASE_A),
        .current_final_pu = MC_CONST(12.0 / MC_CURRENT_BASE_A),
        .current_step_pu = MC_CONST(1.0 / MC_CURRENT_BASE_A),
        .current_limit_pu = MC_CONST(10.0 / MC_CURRENT_BASE_A),
        .rs_current_target_pu = MC_CONST(
            10.0 * 0.8 / MC_CURRENT_BASE_A),
        .rs_threshold_pu = MC_CONST(
            0.01 * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        .rs_voltage_step_pu = MC_CONST(1.0 / MC_VOLTAGE_BASE_V),
        .injection_voltage_pu = MC_CONST(100.0 / MC_VOLTAGE_BASE_V),
        .voltage_to_flux_step = MC_CONST(
            MAIN_LOOP_TIME * MC_VOLTAGE_BASE_V / MC_FLUX_BASE_WB)};
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
    state->state = IDENTIFICATION_IQ_WAIT;
    state->current_target_pu = state->config.current_start_pu;
    state->rs_voltage_pu = state->config.rs_voltage_step_pu;
    state->rs_first_sample = true;
}

void IdentificationIq_Reset(IdentificationIqState_t* state)
{
    if (state == NULL)
        return;
    IdentificationIqConfig_t config = state->config;
    IdentificationIq_Init(state, &config);
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
    state->rs_hold_count++;
    if (state->rs_hold_count < state->config.rs_hold_cycles)
        return;
    state->rs_hold_count = 0U;

    mc_real_t absolute_current = McMath_Abs(state->rs_current_filtered_pu);
    if (absolute_current > state->config.current_limit_pu)
    {
        state->error = IDENTIFICATION_IQ_RS_NOT_CONVERGED;
        state->state = IDENTIFICATION_IQ_FAILED;
        return;
    }
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
    if (McMath_Abs(current_delta) <= MC_CONST(0.0000034))
    {
        state->rs_first_sample = true;
        return;
    }
    mc_real_t estimate = McMath_Div(
        McMath_Sub(state->rs_voltage_pu, state->rs_previous_voltage_pu),
        current_delta);
    mc_real_t change = McMath_Abs(McMath_Sub(
        estimate, state->rs_previous_pu));
    if (state->rs_estimation_step > 0U
        && change <= state->config.rs_threshold_pu
        && absolute_current >= state->config.rs_current_target_pu)
    {
        state->resistance_pu = estimate;
        state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        state->injection_mode = IDENTIFICATION_IQ_INJECT_D;
        identification_reset_capture(state);
        return;
    }
    state->rs_previous_pu = estimate;
    state->rs_estimation_step++;
    if (state->rs_estimation_step >= IDENTIFICATION_IQ_MAX_STEPS)
    {
        state->error = IDENTIFICATION_IQ_RS_NOT_CONVERGED;
        state->state = IDENTIFICATION_IQ_FAILED;
        return;
    }
    state->rs_previous_voltage_pu = state->rs_voltage_pu;
    state->rs_previous_current_pu = state->rs_current_filtered_pu;
    state->rs_voltage_pu = McMath_Add(
        state->rs_voltage_pu, state->config.rs_voltage_step_pu);
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
    if (state->cycle_count >= state->config.injection_half_period_cycles)
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
    output->voltage_d_pu = state->injection_mode == IDENTIFICATION_IQ_INJECT_Q
        ? MC_ZERO : (state->injection_d_positive
        ? state->config.injection_voltage_pu
        : McMath_Neg(state->config.injection_voltage_pu));
    output->voltage_q_pu = state->injection_mode == IDENTIFICATION_IQ_INJECT_D
        ? MC_ZERO : (state->injection_q_positive
        ? state->config.injection_voltage_pu
        : McMath_Neg(state->config.injection_voltage_pu));

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
    state->voltage_d[index] = output->voltage_d_pu;
    state->voltage_q[index] = output->voltage_q_pu;
    state->current_d[index] = input->current_d_pu;
    state->current_q[index] = input->current_q_pu;
    mc_real_t previous_d = index == 0U ? MC_ZERO : state->flux_d[index - 1U];
    mc_real_t previous_q = index == 0U ? MC_ZERO : state->flux_q[index - 1U];
    mc_real_t emf_d = McMath_Sub(output->voltage_d_pu,
        McMath_Mul(state->resistance_pu, input->current_d_pu));
    mc_real_t emf_q = McMath_Sub(output->voltage_q_pu,
        McMath_Mul(state->resistance_pu, input->current_q_pu));
    state->flux_d[index] = McMath_Add(previous_d,
        McMath_Mul(state->config.voltage_to_flux_step, emf_d));
    state->flux_q[index] = McMath_Add(previous_q,
        McMath_Mul(state->config.voltage_to_flux_step, emf_q));
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
    mc_accum_t sum_current = 0;
    for (uint16_t index = 0U; index < state->sample_count; ++index)
    {
        sum_flux += flux[index];
        sum_current += current[index];
    }
    mc_real_t mean_flux = identification_average(sum_flux,
                                                  state->sample_count);
    mc_real_t maximum_flux = MC_ZERO;
    mc_real_t current_at_maximum = MC_ZERO;
    for (uint16_t index = 0U; index < state->sample_count; ++index)
    {
        mc_real_t centered = McMath_Abs(McMath_Sub(flux[index], mean_flux));
        if (centered > maximum_flux)
        {
            maximum_flux = centered;
            current_at_maximum = McMath_Abs(current[index]);
        }
    }
    IdentificationIqPoint_t sample = {
        .current_pu = current_at_maximum,
        .maximum_flux_pu = maximum_flux,
        .valid = maximum_flux > MC_ZERO && current_at_maximum > MC_ZERO};
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
        state->dq_sum_xx += McMath_Add(
            McMath_Mul(x1, x1), McMath_Mul(x2, x2));
        state->dq_sum_xy += McMath_Add(
            McMath_Mul(x1, residual_d), McMath_Mul(x2, residual_q));
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

    /* The d-axis basis contains flux^6.  Calculating that value directly can
     * underflow in Q24 at low flux.  Fit the equivalent normalized basis
     * x*(x/scale)^exponent, then convert its coefficient back to the original
     * physical model.  The normalized basis stays observable without changing
     * the represented curve. */
    for (uint16_t index = 0U; index < count; ++index)
    {
        if (!points[index].valid)
            continue;
        mc_real_t x = points[index].maximum_flux_pu;
        mc_real_t normalized = McMath_Div(x, flux_scale);
        mc_real_t p = McMath_Mul(
            x, identification_power(normalized, exponent));
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
    *coefficient_linear = McMath_Div(
        McMath_Sub(McMath_Mul(spp, syx), McMath_Mul(sxp, syp)),
        determinant);
    mc_real_t coefficient_scaled = McMath_Div(
        McMath_Sub(McMath_Mul(sxx, syp), McMath_Mul(sxp, syx)),
        determinant);
    mc_real_t scale_power = identification_power(flux_scale, exponent);
    if (scale_power == MC_ZERO)
        return false;
    *coefficient_power = McMath_Div(coefficient_scaled, scale_power);

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
        mc_real_t x = points[index].maximum_flux_pu;
        mc_real_t normalized = McMath_Div(x, flux_scale);
        mc_real_t p = McMath_Mul(
            x, identification_power(normalized, exponent));
        mc_real_t predicted = McMath_Add(
            McMath_Mul(*coefficient_linear, x),
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
    mc_real_t mean_xx = identification_average(
        state->dq_sum_xx, average_count);
    mc_real_t mean_xy = identification_average(
        state->dq_sum_xy, average_count);
    state->coefficients.adq = McMath_Clamp(
        McMath_Div(mean_xy, mean_xx), MC_CONST(-32.0), MC_CONST(32.0));
    state->coefficients.valid = true;
    state->state = IDENTIFICATION_IQ_DONE;
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
        state->step_index++;
        if (state->step_index >= state->config.max_steps
            || state->current_target_pu >= state->config.current_final_pu)
        {
            state->state = IDENTIFICATION_IQ_LLS;
        }
        else
        {
            state->current_target_pu = McMath_Add(
                state->current_target_pu, state->config.current_step_pu);
            state->injection_mode = IDENTIFICATION_IQ_INJECT_D;
            identification_reset_capture(state);
            state->state = IDENTIFICATION_IQ_INJECT_COLLECT;
        }
        break;
    case IDENTIFICATION_IQ_LLS:
        if (identification_fit_single_axes(state))
            state->state = IDENTIFICATION_IQ_PENDING;
        else
            state->state = IDENTIFICATION_IQ_FAILED;
        break;
    case IDENTIFICATION_IQ_PENDING:
        state->injection_mode = IDENTIFICATION_IQ_INJECT_DQ;
        state->repeat_count = 0U;
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
