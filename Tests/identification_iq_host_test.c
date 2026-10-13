#include <math.h>
#include <stdio.h>
#include <string.h>
#include "identification_iq.h"
#include "fixed_numeric_config.h"
/* Directly execute the retained float reference capture/PROCESS functions. */
#include "../Application/Src/identification.c"

static int within_relative(float actual, float reference, float limit)
{
    float denominator = fmaxf(fabsf(reference), 1.0e-6F);
    return fabsf(actual - reference) / denominator <= limit;
}

static void contaminate_samples(IdentificationIqState_t* state)
{
    for (unsigned index = 0U; index < IDENTIFICATION_IQ_SAMPLE_CAPACITY; ++index)
    {
        state->voltage_d[index] = MC_CONST(0.7);
        state->voltage_q[index] = MC_CONST(-0.6);
        state->current_d[index] = MC_CONST(0.5);
        state->current_q[index] = MC_CONST(-0.4);
        state->flux_d[index] = MC_CONST(0.3);
        state->flux_q[index] = MC_CONST(-0.2);
    }
}

static int reset_matches_initialized_metadata(bool manual_control)
{
    IdentificationIqConfig_t config;
    IdentificationIqState_t state;
    IdentificationIqState_t expected;
    IdentificationIq_DefaultConfig(&config);
    config.manual_control = manual_control;
    IdentificationIq_Init(&expected, &config);
    /* Dirty every result, counter, accumulator and flag, not just fields
     * currently used by one particular injection workflow. */
    memset(&state, 0x5a, sizeof(state));
    state.config = config;
    contaminate_samples(&state);
    mc_real_t* samples[] = {state.voltage_d, state.voltage_q,
        state.current_d, state.current_q, state.flux_d, state.flux_q};
    mc_real_t saved_samples[6][IDENTIFICATION_IQ_SAMPLE_CAPACITY];
    for (unsigned array = 0U; array < 6U; ++array)
        memcpy(saved_samples[array], samples[array], sizeof(saved_samples[array]));

    IdentificationIq_Reset(&state);
    if (memcmp(&state.config, &config, sizeof(config)) != 0)
    {
        fprintf(stderr, "FAIL reset changed identification configuration\n");
        return 0;
    }
    for (unsigned array = 0U; array < 6U; ++array)
    {
        if (memcmp(saved_samples[array], samples[array],
                   sizeof(saved_samples[array])) != 0)
        {
            fprintf(stderr, "FAIL reset erased sample payload array %u\n", array);
            return 0;
        }
    }

#define EXPECT_RESET_FIELD(field) do { \
    if (state.field != expected.field) { \
        fprintf(stderr, "FAIL reset metadata: %s\n", #field); \
        return 0; \
    } \
} while (0)
    EXPECT_RESET_FIELD(state);
    EXPECT_RESET_FIELD(injection_mode);
    EXPECT_RESET_FIELD(error);
    EXPECT_RESET_FIELD(coefficients.ad0);
    EXPECT_RESET_FIELD(coefficients.add);
    EXPECT_RESET_FIELD(coefficients.aq0);
    EXPECT_RESET_FIELD(coefficients.aqq);
    EXPECT_RESET_FIELD(coefficients.adq);
    EXPECT_RESET_FIELD(coefficients.ssr_d);
    EXPECT_RESET_FIELD(coefficients.ssr_q);
    EXPECT_RESET_FIELD(coefficients.ssr_dq_d);
    EXPECT_RESET_FIELD(coefficients.ssr_dq_q);
    EXPECT_RESET_FIELD(coefficients.r2_d);
    EXPECT_RESET_FIELD(coefficients.r2_q);
    EXPECT_RESET_FIELD(coefficients.r2_dq_d);
    EXPECT_RESET_FIELD(coefficients.r2_dq_q);
    EXPECT_RESET_FIELD(coefficients.valid);
    EXPECT_RESET_FIELD(resistance_pu);
    EXPECT_RESET_FIELD(rs_voltage_pu);
    EXPECT_RESET_FIELD(rs_previous_pu);
    EXPECT_RESET_FIELD(rs_previous_voltage_pu);
    EXPECT_RESET_FIELD(rs_previous_current_pu);
    EXPECT_RESET_FIELD(rs_current_filtered_pu);
    EXPECT_RESET_FIELD(current_target_a);
    EXPECT_RESET_FIELD(current_target_pu);
    EXPECT_RESET_FIELD(accumulated_flux);
    EXPECT_RESET_FIELD(accumulated_current);
    EXPECT_RESET_FIELD(rs_hold_count);
    EXPECT_RESET_FIELD(rs_estimation_step);
    EXPECT_RESET_FIELD(cycle_count);
    EXPECT_RESET_FIELD(sample_count);
    EXPECT_RESET_FIELD(edge_count);
    EXPECT_RESET_FIELD(repeat_count);
    EXPECT_RESET_FIELD(step_index);
    EXPECT_RESET_FIELD(d_result_count);
    EXPECT_RESET_FIELD(q_result_count);
    EXPECT_RESET_FIELD(dq_sum_xx);
    EXPECT_RESET_FIELD(dq_sum_xy);
    EXPECT_RESET_FIELD(dq_sum_id);
    EXPECT_RESET_FIELD(dq_sum_id_squared);
    EXPECT_RESET_FIELD(dq_sum_iq);
    EXPECT_RESET_FIELD(dq_sum_iq_squared);
    EXPECT_RESET_FIELD(dq_sum_residual_d_squared);
    EXPECT_RESET_FIELD(dq_sum_residual_q_squared);
    EXPECT_RESET_FIELD(dq_sample_count);
    EXPECT_RESET_FIELD(last_voltage_d_pu);
    EXPECT_RESET_FIELD(last_voltage_q_pu);
    EXPECT_RESET_FIELD(single_axis_fit_complete);
    EXPECT_RESET_FIELD(rs_first_sample);
    EXPECT_RESET_FIELD(injection_positive);
    EXPECT_RESET_FIELD(injection_d_positive);
    EXPECT_RESET_FIELD(injection_q_positive);
    EXPECT_RESET_FIELD(start_requested);
    EXPECT_RESET_FIELD(rs_complete);
    EXPECT_RESET_FIELD(point_started);
    EXPECT_RESET_FIELD(dq_current_latched);
    for (unsigned index = 0U; index < IDENTIFICATION_IQ_MAX_STEPS; ++index)
    {
        EXPECT_RESET_FIELD(d_results[index].current_pu);
        EXPECT_RESET_FIELD(d_results[index].maximum_flux_pu);
        EXPECT_RESET_FIELD(d_results[index].valid);
        EXPECT_RESET_FIELD(q_results[index].current_pu);
        EXPECT_RESET_FIELD(q_results[index].maximum_flux_pu);
        EXPECT_RESET_FIELD(q_results[index].valid);
    }
#undef EXPECT_RESET_FIELD

    /* Init remains the full clear operation, distinct from bounded Reset. */
    IdentificationIq_Init(&state, &config);
    for (unsigned array = 0U; array < 6U; ++array)
        for (unsigned index = 0U; index < IDENTIFICATION_IQ_SAMPLE_CAPACITY; ++index)
            if (samples[array][index] != MC_ZERO)
                return 0;
    return 1;
}

static int first_sample_ignores_old_flux(void)
{
    IdentificationIqConfig_t config;
    IdentificationIqState_t state;
    IdentificationIqInput_t input = {
        .current_d_pu = MC_CONST(0.04),
        .current_q_pu = MC_CONST(0.05), .injection_enable = true};
    IdentificationIqOutput_t output;
    IdentificationIq_DefaultConfig(&config);
    config.manual_control = true;
    IdentificationIq_Init(&state, &config);
    contaminate_samples(&state);
    IdentificationIq_Reset(&state);
    state.state = IDENTIFICATION_IQ_INJECT_COLLECT;
    state.injection_mode = IDENTIFICATION_IQ_INJECT_DQ;
    state.current_target_pu = MC_CONST(0.1);
    state.edge_count = (uint16_t)(config.wait_edges + 1U);
    state.point_started = true;
    state.injection_d_positive = true;
    state.injection_q_positive = true;
    state.resistance_pu = MC_CONST(0.2);
    IdentificationIq_Run(&state, &input, &output);
    return state.sample_count == 1U
        && state.voltage_d[0] == MC_ZERO
        && state.voltage_q[0] == MC_ZERO
        && state.current_d[0] == input.current_d_pu
        && state.current_q[0] == input.current_q_pu
        && state.flux_d[0] == MC_ZERO && state.flux_q[0] == MC_ZERO
        && state.flux_d[1] == MC_CONST(0.3)
        && state.flux_q[1] == MC_CONST(-0.2);
}

static int legacy_capture_alignment(void)
{
    for (unsigned mode = 0U; mode < 3U; ++mode)
    {
        IdentificationIqConfig_t config;
        IdentificationIqState_t state;
        IdentificationIq_DefaultConfig(&config);
        config.manual_control = true;
        /* Preserve the old 3 A waveform fixture; DQ now has its own input. */
        config.dq_current_pu = McMath_FromFloat(3.0F / MC_CURRENT_BASE_A);
        config.injection_voltage_d_pu = McMath_FromFloat(100.0F / MC_VOLTAGE_BASE_V);
        config.injection_voltage_q_pu = McMath_FromFloat(70.0F / MC_VOLTAGE_BASE_V);
        IdentificationIq_Init(&state, &config);
        state.state = IDENTIFICATION_IQ_INJECT_COLLECT;
        state.injection_mode = (IdentificationIqInjectionMode_e)mode;
        state.current_target_pu = McMath_FromFloat(3.0F / MC_CURRENT_BASE_A);
        state.resistance_pu = McMath_FromFloat(0.65F * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V);
        state.injection_d_positive = state.injection_q_positive = true;
        FluxExperiment_t legacy;
        Experiment_Init(&legacy, 0.0002F, 512, 1, 20, 3, 12, 1, 100.0F);
        legacy.state = INJECT_COLLECT;
        legacy.inj.State = true;
        legacy.inj.mode = (Inj_Mode_e)mode;
        legacy.inj.Imax = 3.0F;
        legacy.inj.Ud_amp = 100.0F;
        legacy.inj.Uq_amp = 70.0F;
        legacy.Rs_est = 0.65F;
        bool saw_flip = false;
        unsigned cycle;
        for (cycle = 0U; cycle < 1000U; ++cycle)
        {
            float id = 3.2F * sinf(cycle * (6.28318530718F / 20.0F));
            float iq = 3.2F * sinf(cycle * (6.28318530718F / 28.0F));
            IdentificationIqInput_t input = {
                .current_d_pu = McMath_FromFloat(id / MC_CURRENT_BASE_A),
                .current_q_pu = McMath_FromFloat(iq / MC_CURRENT_BASE_A),
                .injection_enable = true};
            float old_d = 0.0F, old_q = 0.0F;
            IdentificationIqOutput_t out;
            Experiment_Step(&legacy, id, iq, &old_d, &old_q);
            IdentificationIq_Run(&state, &input, &out);
            if (fabsf(old_d - McMath_ToFloat(out.voltage_d_pu) * MC_VOLTAGE_BASE_V) > 0.001F
                || fabsf(old_q - McMath_ToFloat(out.voltage_q_pu) * MC_VOLTAGE_BASE_V) > 0.001F)
                return 0;
            for (unsigned k = 0U; k < state.sample_count; ++k)
            {
                if (fabsf(legacy.Ud_buf[k] - McMath_ToFloat(state.voltage_d[k]) * MC_VOLTAGE_BASE_V) > 0.001F
                    || fabsf(legacy.Uq_buf[k] - McMath_ToFloat(state.voltage_q[k]) * MC_VOLTAGE_BASE_V) > 0.001F
                    || fabsf(legacy.psi_d_buf[k] - McMath_ToFloat(state.flux_d[k]) * MC_FLUX_BASE_WB) > 0.0001F
                    || fabsf(legacy.psi_q_buf[k] - McMath_ToFloat(state.flux_q[k]) * MC_FLUX_BASE_WB) > 0.0001F)
                    return 0;
            }
            if (state.sample_count > 0U && state.state != IDENTIFICATION_IQ_PROCESS)
            {
                unsigned k = state.sample_count - 1U;
                if (state.voltage_d[k] != out.voltage_d_pu
                    || state.voltage_q[k] != out.voltage_q_pu)
                    saw_flip = true;
            }
            if (state.state == IDENTIFICATION_IQ_PROCESS)
                break;
        }
        if (cycle == 1000U || legacy.state != PROCESS
            || legacy.edge_idx[1] != state.sample_count || !saw_flip)
            return 0;
    }
    return 1;
}

static int legacy_positive_peak_keeps_current_sign(void)
{
    /* Negative peak is larger in absolute value; the original selects the
     * positive centered peak and keeps the current's sign at that sample. */
    const float flux[] = {-0.4F, 0.05F, 0.05F, 0.2F};
    const float current[] = {9.0F, 2.0F, 3.0F, -2.0F};
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        IdentificationIqConfig_t config;
        IdentificationIqState_t state;
        IdentificationIq_DefaultConfig(&config);
        config.manual_control = true;
        config.repeat_times = 1U;
        IdentificationIq_Init(&state, &config);
        state.state = IDENTIFICATION_IQ_PROCESS;
        state.injection_mode = (IdentificationIqInjectionMode_e)axis;
        state.sample_count = 4U;
        FluxExperiment_t legacy;
        Experiment_Init(&legacy, 0.0002F, 512, 1, 20, 3, 12, 1, 100.0F);
        legacy.state = PROCESS;
        legacy.inj.mode = (Inj_Mode_e)axis;
        legacy.edge_count = 6;
        legacy.edge_idx[0] = 0;
        legacy.edge_idx[1] = 4;
        for (unsigned k = 0U; k < 4U; ++k)
        {
            state.flux_d[k] = state.flux_q[k] = McMath_FromFloat(flux[k] / MC_FLUX_BASE_WB);
            state.current_d[k] = state.current_q[k] = McMath_FromFloat(current[k] / MC_CURRENT_BASE_A);
            legacy.psi_d_buf[k] = legacy.psi_q_buf[k] = flux[k];
            legacy.Id_buf[k] = legacy.Iq_buf[k] = current[k];
        }
        float old_d = 0.0F, old_q = 0.0F;
        IdentificationIqInput_t input = {0};
        IdentificationIqOutput_t output;
        Experiment_Step(&legacy, 0.0F, 0.0F, &old_d, &old_q);
        IdentificationIq_Run(&state, &input, &output);
        const IdentificationIqPoint_t* point = axis ? state.q_results : state.d_results;
        if (!point->valid || point->current_pu >= MC_ZERO
            || fabsf(legacy.results[0].Imax_value - McMath_ToFloat(point->current_pu) * MC_CURRENT_BASE_A) > 0.00001F
            || fabsf(legacy.results[0].avg_max_psi - McMath_ToFloat(point->maximum_flux_pu) * MC_FLUX_BASE_WB) > 0.00001F)
            return 0;
    }
    return 1;
}

static int manual_pause_holds_original_capture(void)
{
    IdentificationIqConfig_t config;
    IdentificationIqState_t state;
    IdentificationIq_DefaultConfig(&config);
    config.manual_control = true;
    IdentificationIq_Init(&state, &config);
    state.state = IDENTIFICATION_IQ_INJECT_COLLECT;
    state.injection_mode = IDENTIFICATION_IQ_INJECT_D;
    state.current_target_pu = McMath_FromFloat(3.0F / MC_CURRENT_BASE_A);
    state.edge_count = (uint16_t)(config.wait_edges + 1U);
    state.injection_d_positive = true;
    state.point_started = true;
    state.last_voltage_d_pu = config.injection_voltage_d_pu;
    state.sample_count = 1U;
    state.flux_d[0] = MC_ZERO;
    FluxExperiment_t legacy;
    Experiment_Init(&legacy, 0.0002F, 512, 1, 20, 3, 12, 1, 100.0F);
    legacy.state = INJECT_COLLECT;
    legacy.inj.mode = INJECT_D;
    legacy.inj.State = false;
    legacy.inj.Imax = 3.0F;
    legacy.inj.Ud_amp = 100.0F;
    legacy.last_Vd = 100.0F;
    legacy.edge_count = config.wait_edges + 1U;
    legacy.pos = 1;
    IdentificationIqInput_t input = {0};
    IdentificationIqOutput_t out;
    float old_d = 0.0F, old_q = 0.0F;
    for (unsigned n = 0U; n < 3U; ++n)
    {
        Experiment_Step(&legacy, 0.0F, 0.0F, &old_d, &old_q);
        IdentificationIq_Run(&state, &input, &out);
        if (out.voltage_d_pu != MC_ZERO || out.voltage_q_pu != MC_ZERO
            || state.sample_count != 1U || state.point_started
            || fabsf(legacy.last_Vd - McMath_ToFloat(state.last_voltage_d_pu) * MC_VOLTAGE_BASE_V) > 0.001F)
            return 0;
    }
    legacy.inj.State = input.injection_enable = true;
    Experiment_Step(&legacy, 0.0F, 0.0F, &old_d, &old_q);
    IdentificationIq_Run(&state, &input, &out);
    return state.point_started && state.sample_count == 2U
        && fabsf(legacy.psi_d_buf[1] - McMath_ToFloat(state.flux_d[1]) * MC_FLUX_BASE_WB) < 0.0001F;
}

static int reset_rejects_invalid_config(void)
{
    IdentificationIqConfig_t config;
    IdentificationIqState_t state;
    IdentificationIqInput_t input = {0};
    IdentificationIqOutput_t output;
    for (unsigned invalid = 0U; invalid < 5U; ++invalid)
    {
        IdentificationIq_DefaultConfig(&config);
        if (invalid == 0U) config.sample_capacity = 0U;
        if (invalid == 1U)
            config.sample_capacity = IDENTIFICATION_IQ_SAMPLE_CAPACITY + 1U;
        if (invalid == 2U) config.max_steps = 0U;
        if (invalid == 3U) config.max_steps = IDENTIFICATION_IQ_MAX_STEPS + 1U;
        if (invalid == 4U) config.repeat_times = 0U;
        IdentificationIq_Init(&state, &config);
        contaminate_samples(&state);
        state.coefficients.valid = true;
        state.sample_count = 17U;
        IdentificationIq_Reset(&state);
        IdentificationIq_Start(&state);
        IdentificationIq_Run(&state, &input, &output);
        if (state.state != IDENTIFICATION_IQ_FAILED
            || state.error != IDENTIFICATION_IQ_INVALID_CONFIG
            || state.coefficients.valid || state.sample_count != 0U
            || state.start_requested || output.running || output.valid
            || output.voltage_d_pu != MC_ZERO
            || output.voltage_q_pu != MC_ZERO)
            return 0;
    }
    return 1;
}

static int manual_workflow_completes(void)
{
    IdentificationIqConfig_t config;
    IdentificationIqState_t state;
    IdentificationIqInput_t input = {0};
    IdentificationIqOutput_t output = {0};
    IdentificationIq_DefaultConfig(&config);
    config.manual_control = true;
    config.sample_capacity = 128U;
    config.repeat_times = 2U;
    config.max_steps = 6U;
    config.wait_edges = 1U;
    config.rs_hold_cycles = 1000U;
    config.current_start_pu = MC_CONST(0.03);
    config.current_final_pu = MC_CONST(0.18);
    config.current_step_pu = MC_CONST(0.03);
    config.dq_current_pu = config.current_start_pu;
    config.current_limit_pu = MC_CONST(0.8);
    /* The legacy Rs predictor needs a below-target estimate before the
     * convergence target is crossed. 0.15 with a 0.02 V / 0.2 R fixture
     * crosses on its very first estimate and correctly times out. */
    config.rs_current_target_pu = MC_CONST(0.25);
    config.rs_threshold_pu = MC_CONST(0.01);
    config.rs_voltage_step_pu = MC_CONST(0.02);
    config.injection_voltage_d_pu = MC_CONST(0.10);
    config.injection_voltage_q_pu = MC_CONST(0.10);
    config.voltage_to_flux_step = MC_CONST(0.16);
    IdentificationIq_Init(&state, &config);
    /* The old payload must not influence a complete new D -> Q -> DQ run. */
    contaminate_samples(&state);
    state.coefficients.valid = true;
    state.sample_count = config.sample_capacity;
    state.dq_sum_xx = 123;
    IdentificationIq_Reset(&state);

    const float plant_resistance = 0.2F;
    const float plant_ad0 = 0.1F;
    const float plant_add = 20.0F;
    const float plant_aq0 = 0.2F;
    const float plant_aqq = 0.5F;
    const float plant_adq = 0.2F;
    float plant_flux_d = 0.0F;
    float plant_flux_q = 0.0F;
    float current_d = 0.0F;
    float current_q = 0.0F;
    unsigned enable_count = 0U;
    for (unsigned iteration = 0U; iteration < 1000000U; ++iteration)
    {
        input.current_d_pu = McMath_FromFloat(current_d);
        input.current_q_pu = McMath_FromFloat(current_q);
        input.start_rs = state.state == IDENTIFICATION_IQ_WAIT
                      && !state.rs_complete;
        input.start_axis = state.state == IDENTIFICATION_IQ_WAIT
                        && state.rs_complete;
        input.injection_enable = state.state == IDENTIFICATION_IQ_INJECT_COLLECT;
        if (input.injection_enable && !state.point_started)
            enable_count++;
        IdentificationIq_Run(&state, &input, &output);
        float voltage_d = McMath_ToFloat(output.voltage_d_pu);
        float voltage_q = McMath_ToFloat(output.voltage_q_pu);
        plant_flux_d += 0.16F * (voltage_d
                      - plant_resistance * current_d);
        plant_flux_q += 0.16F * (voltage_q
                      - plant_resistance * current_q);
        float absolute_flux_d = fabsf(plant_flux_d);
        float absolute_flux_q = fabsf(plant_flux_q);
        float d_term = plant_ad0
                     + plant_add * powf(absolute_flux_d, 5.0F)
                     + 0.5F * plant_adq * absolute_flux_d
                     * absolute_flux_q * absolute_flux_q;
        float q_term = plant_aq0 + plant_aqq * absolute_flux_q
                     + (plant_adq / 3.0F)
                     * absolute_flux_d * absolute_flux_d * absolute_flux_d;
        current_d = d_term * plant_flux_d;
        current_q = q_term * plant_flux_q;
        if (state.state == IDENTIFICATION_IQ_DONE)
            return state.coefficients.valid
                && state.d_result_count == config.max_steps
                && state.q_result_count == config.max_steps
                && enable_count == (unsigned)(config.max_steps * 2U + 1U);
        if (state.state == IDENTIFICATION_IQ_FAILED)
            return 0;
    }
    return 0;
}

static int independent_dq_current_latching(void)
{
    for (unsigned manual = 0U; manual < 2U; ++manual)
    {
        IdentificationIqConfig_t config;
        IdentificationIqState_t state;
        IdentificationIqInput_t input = {0};
        IdentificationIqOutput_t output;
        IdentificationIq_DefaultConfig(&config);
        if (fabsf(McMath_ToFloat(config.dq_current_pu) * MC_CURRENT_BASE_A
                  - 9.0F) > 0.00002F)
            return 0;
        config.manual_control = manual != 0U;
        IdentificationIq_Init(&state, &config);
        state.injection_mode = IDENTIFICATION_IQ_INJECT_DQ;
        state.state = IDENTIFICATION_IQ_PENDING;
        IdentificationIq_Run(&state, &input, &output);
        if (state.state != IDENTIFICATION_IQ_INJECT_COLLECT
            || state.current_target_pu != config.dq_current_pu
            || state.dq_current_latched)
            return 0;

        const mc_real_t selected = McMath_FromFloat(9.5F / MC_CURRENT_BASE_A);
        state.config.dq_current_pu = selected;
        if (manual)
        {
            IdentificationIq_Run(&state, &input, &output);
            if (state.current_target_pu != selected || state.dq_current_latched
                || state.point_started || output.voltage_d_pu != MC_ZERO
                || output.voltage_q_pu != MC_ZERO)
                return 0;
        }
        input.injection_enable = true;
        IdentificationIq_Run(&state, &input, &output);
        if (!state.dq_current_latched || state.current_target_pu != selected)
            return 0;

        state.config.dq_current_pu = McMath_FromFloat(10.0F / MC_CURRENT_BASE_A);
        input.injection_enable = false; /* Manual pause must not unlock. */
        IdentificationIq_Run(&state, &input, &output);
        if (!state.dq_current_latched || state.current_target_pu != selected)
            return 0;
        input.injection_enable = true;
        IdentificationIq_Run(&state, &input, &output);
        if (state.current_target_pu != selected) return 0;

        /* Simulate a completed capture, exercise the real repeat transition,
         * and ensure reset_capture does not unlock the batch threshold. */
        state.state = IDENTIFICATION_IQ_PROCESS;
        state.sample_count = 2U;
        for (unsigned sample = 0U; sample < 2U; ++sample)
        {
            state.flux_d[sample] = state.flux_q[sample] = MC_ZERO;
            state.current_d[sample] = state.current_q[sample] = MC_ZERO;
        }
        IdentificationIq_Run(&state, &input, &output);
        if (state.state != IDENTIFICATION_IQ_INJECT_COLLECT
            || state.repeat_count != 1U || !state.dq_current_latched
            || state.current_target_pu != selected)
            return 0;
        IdentificationIq_Run(&state, &input, &output);
        if (state.current_target_pu != selected) return 0;

        IdentificationIq_Reset(&state);
        if (state.dq_current_latched) return 0;
        state.injection_mode = IDENTIFICATION_IQ_INJECT_DQ;
        state.state = IDENTIFICATION_IQ_PENDING;
        IdentificationIq_Run(&state, &input, &output);
        if (state.current_target_pu != state.config.dq_current_pu) return 0;

        /* Single-axis targets remain owned by the existing scan, regardless
         * of the new DQ setting or whether the scan runs forward/reverse. */
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            IdentificationIq_Reset(&state);
            state.injection_mode = (IdentificationIqInjectionMode_e)axis;
            state.state = IDENTIFICATION_IQ_INJECT_COLLECT;
            state.current_target_pu = MC_CONST(0.1);
            IdentificationIq_Run(&state, &input, &output);
            if (state.current_target_pu != MC_CONST(0.1)
                || state.dq_current_latched)
                return 0;
        }
    }
    puts("PASS: independent 9 A DQ default, fractional preview, batch latch, pause/repeat and reset");
    return 1;
}

static int legacy_integer_scan_endpoints(void)
{
    IdentificationIqConfig_t defaults;
    IdentificationIq_DefaultConfig(&defaults);
    defaults.manual_control = true;
    IdentificationIqState_t default_state;
    IdentificationIq_Init(&default_state, &defaults);
    default_state.rs_complete = true;
    IdentificationIqInput_t default_input = {.start_axis = true};
    IdentificationIqOutput_t default_output;
    IdentificationIq_Run(&default_state, &default_input, &default_output);
    unsigned default_count = 0U;
    while (default_state.state == IDENTIFICATION_IQ_INJECT_COLLECT)
    {
        if (++default_count > 20U) return 0;
        default_state.state = IDENTIFICATION_IQ_NEXT_CURRENT;
        IdentificationIq_Run(&default_state, &default_input, &default_output);
    }
    if (default_count != 10U || default_state.state != IDENTIFICATION_IQ_LLS)
        return 0;
    static const struct { int start, final, step, maximum, count, last; } cases[] = {
        {3, 12, 1, 20, 10, 12}, {12, 3, -1, 20, 10, 3},
        {3, 22, 1, 20, 20, 22}, {22, 3, -1, 20, 20, 3},
        {3, 12, 2, 20, 5, 11}, {12, 3, -2, 20, 5, 4},
        {3, 12, 1, 4, 4, 6}, {3, 3, 1, 20, 1, 3},
        {3, 12, INT32_MAX, 20, 1, 3}, {12, 3, INT32_MIN, 20, 1, 12}
    };
    for (unsigned test = 0U; test < sizeof(cases) / sizeof(cases[0]); ++test)
    {
        IdentificationIqConfig_t config;
        IdentificationIq_DefaultConfig(&config);
        config.manual_control = true;
        config.integer_current_scan = true;
        config.current_start_a = cases[test].start;
        config.current_final_a = cases[test].final;
        config.current_step_a = cases[test].step;
        config.max_steps = (uint16_t)cases[test].maximum;
        /* Prove the physical-integer mode does not accumulate these rounded
         * three quantities (the old bug omitted 12 A by four raw Q24 LSBs). */
        config.current_start_pu = McMath_FromFloat((float)config.current_start_a / 30.0F);
        config.current_final_pu = McMath_FromFloat((float)config.current_final_a / 30.0F);
        IdentificationIqState_t state;
        IdentificationIq_Init(&state, &config);
        state.rs_complete = true;
        IdentificationIqInput_t input = {.start_axis = true};
        IdentificationIqOutput_t output;
        IdentificationIq_Run(&state, &input, &output);
        unsigned count = 0U;
        int last = 0;
        while (state.state == IDENTIFICATION_IQ_INJECT_COLLECT)
        {
            last = state.current_target_a;
            if (fabsf(McMath_ToFloat(state.current_target_pu) * 30.0F
                      - (float)last) > 0.00001F)
                return 0;
            ++count;
            if (count > 20U) return 0;
            state.state = IDENTIFICATION_IQ_NEXT_CURRENT;
            IdentificationIq_Run(&state, &input, &output);
        }
        if (state.state != IDENTIFICATION_IQ_LLS
            || count != (unsigned)cases[test].count || last != cases[test].last)
        {
            fprintf(stderr, "FAIL integer scan %d->%d step%d: count=%u last=%d\n",
                cases[test].start, cases[test].final, cases[test].step, count, last);
            return 0;
        }
        /* Execute the retained FLOAT_REF state machine too. Its int addition
         * has no overflow guard, so exclude the two deliberate int extremes. */
        if (test < 8U)
        {
            FluxExperiment_t legacy;
            Experiment_Init(&legacy, 0.0002F, 512, 1, cases[test].maximum,
                cases[test].start, cases[test].final, cases[test].step, 100.0F);
            legacy.step_dir = cases[test].step; /* Live A2L write after Init. */
            float ud = 0.0F, uq = 0.0F;
            unsigned old_count = 0U;
            for (;;)
            {
                legacy.state = NEXT_I;
                Experiment_Step(&legacy, 0.0F, 0.0F, &ud, &uq);
                if (legacy.state == LLS) break;
                ++old_count;
                ++legacy.step_index; /* A completed, accepted current point. */
                if (old_count > 20U) return 0;
            }
            if (old_count != count || (int)legacy.inj.Imax != last) return 0;
        }
    }
    for (unsigned invalid = 0U; invalid < 5U; ++invalid)
    {
        IdentificationIqConfig_t config;
        IdentificationIq_DefaultConfig(&config);
        config.integer_current_scan = true;
        if (invalid == 0U) config.current_step_a = 0;
        if (invalid == 1U) config.current_step_a = -1;
        if (invalid == 2U) { config.current_start_a = 12; config.current_final_a = 3; }
        if (invalid == 3U) config.current_final_a = INT32_MAX;
        if (invalid == 4U) config.current_start_a = INT32_MIN;
        IdentificationIqState_t state;
        IdentificationIq_Init(&state, &config);
        IdentificationIqInput_t invalid_input = {
            .start_axis = true, .injection_enable = true};
        IdentificationIqOutput_t invalid_output = {
            .voltage_d_pu = MC_ONE, .voltage_q_pu = MC_ONE};
        IdentificationIq_Run(&state, &invalid_input, &invalid_output);
#if defined(MC_NUMERIC_IQMATH)
        if (state.state != IDENTIFICATION_IQ_FAILED
            || invalid_output.voltage_d_pu != MC_ZERO
            || invalid_output.voltage_q_pu != MC_ZERO) return 0;
#else
        if (invalid < 3U && (state.state != IDENTIFICATION_IQ_FAILED
            || invalid_output.voltage_d_pu != MC_ZERO
            || invalid_output.voltage_q_pu != MC_ZERO)) return 0;
#endif
    }

    /* PU-only callers intentionally get exact represented-value semantics,
     * not an arbitrary epsilon that adds a physically unrequested endpoint. */
    IdentificationIqConfig_t config;
    IdentificationIq_DefaultConfig(&config);
    config.manual_control = true;
    config.current_start_pu = MC_CONST(0.125);
    config.current_step_pu = MC_CONST(0.125);
    config.current_final_pu = MC_CONST(0.374);
    IdentificationIqState_t state;
    IdentificationIq_Init(&state, &config);
    state.rs_complete = true;
    IdentificationIqInput_t input = {.start_axis = true};
    IdentificationIqOutput_t output;
    IdentificationIq_Run(&state, &input, &output);
    state.state = IDENTIFICATION_IQ_NEXT_CURRENT;
    IdentificationIq_Run(&state, &input, &output);
    if (state.current_target_pu != MC_CONST(0.25)) return 0;
    state.state = IDENTIFICATION_IQ_NEXT_CURRENT;
    IdentificationIq_Run(&state, &input, &output);
    if (state.state != IDENTIFICATION_IQ_LLS) return 0;
    config.integer_current_scan = true;
    config.current_step_a = 0;
    state.config = config; /* Invalid live write before the next axis. */
    state.state = IDENTIFICATION_IQ_PENDING;
    output.voltage_d_pu = output.voltage_q_pu = MC_ONE;
    IdentificationIq_Run(&state, &input, &output);
    if (state.state != IDENTIFICATION_IQ_FAILED
        || output.voltage_d_pu != MC_ZERO || output.voltage_q_pu != MC_ZERO)
        return 0;
    return 1;
}

static int legacy_rs_voltage_and_hold_parity(void)
{
    /* Estimate_Rs is the ACTUAL retained float function, with its own static
     * state. Run it only once from its cold state, against identical samples.
     * Include a blocked predictor at 8.2 A, a tiny-delta restart, then success. */
    static const float currents[] = {1.4F, 2.8F, 4.2F, 5.6F, 7.0F,
                                    8.2F, 7.0F, 7.0F, 8.4F, 8.4F};
    IdentificationIqState_t state;
    IdentificationIq_Init(&state, NULL);
    state.state = IDENTIFICATION_IQ_EST_RS;
    IdentificationIqInput_t input = {0};
    IdentificationIqOutput_t output;
    float old_voltage = 0.0F, old_resistance = 0.0F;
    bool old_done = false;
    for (unsigned block = 0U; block < sizeof(currents) / sizeof(currents[0]); ++block)
    {
        input.current_d_pu = McMath_FromFloat(currents[block] / MC_CURRENT_BASE_A);
        for (unsigned cycle = 0U; cycle <= HOLD_CYCLES; ++cycle)
        {
            old_done = Estimate_Rs(currents[block], &old_voltage, &old_resistance);
            IdentificationIq_Run(&state, &input, &output);
            float voltage = McMath_ToFloat(output.voltage_d_pu) * MC_VOLTAGE_BASE_V;
            if (fabsf(voltage - old_voltage) > 0.001F
                || old_done != (state.state == IDENTIFICATION_IQ_INJECT_COLLECT))
            {
                fprintf(stderr, "FAIL actual old Rs parity block=%u cycle=%u V=%g/%g done=%d state=%d\n",
                    block, cycle, voltage, old_voltage, old_done, state.state);
                return 0;
            }
        }
        if (block == 5U && fabsf(old_voltage - 6.0F) > 0.0001F) return 0;
        if (block == 6U && (!state.rs_first_sample || state.rs_estimation_step != 0U))
            return 0;
    }
    return old_done && fabsf(McMath_ToFloat(state.resistance_pu)
        * MC_VOLTAGE_BASE_V / MC_CURRENT_BASE_A - old_resistance) < 0.001F;
}

static int rs_failure_and_delta_boundaries(void)
{
    IdentificationIqState_t state;
    IdentificationIqInput_t input = {0};
    IdentificationIqOutput_t output;
    IdentificationIq_Init(&state, NULL);
    state.state = IDENTIFICATION_IQ_EST_RS;
    state.rs_first_sample = false;
    state.rs_estimation_step = 19U;
    state.rs_previous_voltage_pu = McMath_FromFloat(5.0F / MC_VOLTAGE_BASE_V);
    state.rs_voltage_pu = McMath_FromFloat(6.0F / MC_VOLTAGE_BASE_V);
    state.rs_previous_current_pu = McMath_FromFloat(7.0F / MC_CURRENT_BASE_A);
    state.rs_previous_pu = McMath_FromFloat((1.0F / 1.4F)
        * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V);
    state.rs_current_filtered_pu = input.current_d_pu =
        McMath_FromFloat(8.2F / MC_CURRENT_BASE_A);
    state.rs_hold_count = state.config.rs_hold_cycles;
    IdentificationIq_Run(&state, &input, &output);
    if (state.state != IDENTIFICATION_IQ_EST_RS || state.rs_estimation_step != 20U)
        return 0; /* Old ++est_step > 20, not >= 20. */
    state.rs_hold_count = state.config.rs_hold_cycles;
    IdentificationIq_Run(&state, &input, &output);
    if (state.state != IDENTIFICATION_IQ_FAILED || output.valid
        || output.voltage_d_pu != MC_ZERO || output.voltage_q_pu != MC_ZERO)
        return 0;

    IdentificationIq_Init(&state, NULL);
    state.state = IDENTIFICATION_IQ_EST_RS;
    state.rs_first_sample = false;
    state.rs_hold_count = state.config.rs_hold_cycles;
    state.rs_current_filtered_pu = input.current_d_pu =
        McMath_FromFloat(11.0F / MC_CURRENT_BASE_A);
    IdentificationIq_Run(&state, &input, &output);
    if (state.state != IDENTIFICATION_IQ_FAILED || output.valid
        || output.voltage_d_pu != MC_ZERO || output.voltage_q_pu != MC_ZERO)
        return 0; /* Preserve explicit failure, not legacy done_flag=2 success. */
#if defined(MC_NUMERIC_IQMATH)
    /* Raw 55 is below 1e-4 A; raw 56 is above. Use integer expectations,
     * independent of the host mock's round-to-nearest _IQ24 constant macro. */
    for (int32_t delta = 55; delta <= 56; ++delta)
    {
        IdentificationIq_Init(&state, NULL);
        state.state = IDENTIFICATION_IQ_EST_RS;
        state.rs_first_sample = false;
        state.rs_hold_count = state.config.rs_hold_cycles;
        state.rs_estimation_step = 0U;
        state.rs_voltage_pu = state.config.rs_voltage_step_pu = 1;
        input.current_d_pu = delta * 10; /* 0.1 IIR from zero gives delta. */
        IdentificationIq_Run(&state, &input, &output);
        if (state.rs_first_sample != (delta == 55)
            || state.state != IDENTIFICATION_IQ_EST_RS)
            return 0;
    }
#endif
    return 1;
}

int main(void)
{
    if (!reset_matches_initialized_metadata(false)
        || !reset_matches_initialized_metadata(true)
        || !first_sample_ignores_old_flux()
        || !reset_rejects_invalid_config()
        || !legacy_capture_alignment()
        || !legacy_positive_peak_keeps_current_sign()
        || !manual_pause_holds_original_capture()
        || !independent_dq_current_latching()
        || !legacy_integer_scan_endpoints()
        || !legacy_rs_voltage_and_hold_parity()
        || !rs_failure_and_delta_boundaries())
    {
        fprintf(stderr, "FAIL bounded identification reset regression\n");
        return 1;
    }
    IdentificationIqConfig_t config;
    IdentificationIqState_t state;
    IdentificationIqInput_t input = {0};
    IdentificationIqOutput_t output = {0};
    IdentificationIq_DefaultConfig(&config);
    config.sample_capacity = 128U;
    config.repeat_times = 2U;
    config.max_steps = 6U;
    config.wait_edges = 1U;
    config.injection_half_period_cycles = 80U;
    config.rs_hold_cycles = 1000U;
    config.current_start_pu = MC_CONST(0.03);
    config.current_final_pu = MC_CONST(0.18);
    config.current_step_pu = MC_CONST(0.03);
    /* Automatic legacy fixture used the last scan point for joint injection. */
    config.dq_current_pu = config.current_final_pu;
    config.current_limit_pu = MC_CONST(0.8);
    config.rs_current_target_pu = MC_CONST(0.25);
    config.rs_threshold_pu = MC_CONST(0.01);
    config.rs_voltage_step_pu = MC_CONST(0.02);
    config.injection_voltage_pu = MC_CONST(0.10);
    /* Match the firmware's 5 kHz / 800 V / 1 Wb flux integration scale. */
    config.voltage_to_flux_step = MC_CONST(0.16);
    IdentificationIq_Init(&state, &config);
    IdentificationIq_Start(&state);

    const float plant_resistance = 0.2F;
    const float plant_ad0 = 0.1F;
    const float plant_add = 20.0F;
    const float plant_aq0 = 0.2F;
    const float plant_aqq = 0.5F;
    const float plant_adq = 0.2F;
    float plant_flux_d = 0.0F;
    float plant_flux_q = 0.0F;
    float current_d = 0.0F;
    float current_q = 0.0F;
    unsigned iteration;
    for (iteration = 0U; iteration < 1000000U; ++iteration)
    {
        input.current_d_pu = McMath_FromFloat(current_d);
        input.current_q_pu = McMath_FromFloat(current_q);
        IdentificationIq_Run(&state, &input, &output);
        float voltage_d = McMath_ToFloat(output.voltage_d_pu);
        float voltage_q = McMath_ToFloat(output.voltage_q_pu);
        plant_flux_d += 0.16F * (voltage_d
                      - plant_resistance * current_d);
        plant_flux_q += 0.16F * (voltage_q
                      - plant_resistance * current_q);
        float absolute_flux_d = fabsf(plant_flux_d);
        float absolute_flux_q = fabsf(plant_flux_q);
        float d_term = plant_ad0
                     + plant_add * powf(absolute_flux_d, 5.0F)
                     + 0.5F * plant_adq * absolute_flux_d
                     * absolute_flux_q * absolute_flux_q;
        float q_term = plant_aq0 + plant_aqq * absolute_flux_q
                     + (plant_adq / 3.0F)
                     * absolute_flux_d * absolute_flux_d * absolute_flux_d;
        current_d = d_term * plant_flux_d;
        current_q = q_term * plant_flux_q;
        if (state.state == IDENTIFICATION_IQ_DONE
            || state.state == IDENTIFICATION_IQ_FAILED)
            break;
    }
    if (state.state != IDENTIFICATION_IQ_DONE || !state.coefficients.valid)
    {
        fprintf(stderr,
                "FAIL identification: iteration=%u state=%u error=%u steps=%u samples=%u\n",
                iteration, (unsigned)state.state, (unsigned)state.error,
                state.step_index, state.sample_count);
        for (unsigned index = 0U; index < state.step_index; ++index)
        {
            fprintf(stderr,
                    "  step=%u d=(%.8f, %.8f, %u) q=(%.8f, %.8f, %u)\n",
                    index,
                    McMath_ToFloat(state.d_results[index].current_pu),
                    McMath_ToFloat(state.d_results[index].maximum_flux_pu),
                    state.d_results[index].valid ? 1U : 0U,
                    McMath_ToFloat(state.q_results[index].current_pu),
                    McMath_ToFloat(state.q_results[index].maximum_flux_pu),
                    state.q_results[index].valid ? 1U : 0U);
        }
        return 1;
    }
    float result_ad0 = McMath_ToFloat(state.coefficients.ad0);
    float result_add = McMath_ToFloat(state.coefficients.add);
    float result_aq0 = McMath_ToFloat(state.coefficients.aq0);
    float result_aqq = McMath_ToFloat(state.coefficients.aqq);
    float result_adq = McMath_ToFloat(state.coefficients.adq);
    /* Golden values are generated by this deterministic plant with the
     * FLOAT_REF backend.  The same 1% bound is then applied to Q24. */
    if (state.step_index != config.max_steps
        || !isfinite(result_ad0) || !isfinite(result_adq)
        || !within_relative(result_ad0, 0.09577211F, 0.01F)
        || !within_relative(result_add, 21.94590569F, 0.01F)
        || !within_relative(result_aq0, 0.20137627F, 0.01F)
        || !within_relative(result_aqq, 0.51704043F, 0.01F)
        || !within_relative(result_adq, 0.22158611F, 0.01F)
        || McMath_Diagnostics.saturation_count != 0U
        || McMath_Diagnostics.divide_by_zero_count != 0U
        || McMath_Diagnostics.invalid_input_count != 0U)
    {
        fprintf(stderr, "FAIL identification result bounds: cycles=%u ad0=%.8f add=%.8f aq0=%.8f aqq=%.8f adq=%.8f diagnostics=%u/%u/%u\n",
            iteration + 1U, result_ad0, result_add, result_aq0, result_aqq, result_adq,
            (unsigned)McMath_Diagnostics.saturation_count,
            (unsigned)McMath_Diagnostics.divide_by_zero_count,
            (unsigned)McMath_Diagnostics.invalid_input_count);
        return 1;
    }
    if (!manual_workflow_completes())
    {
        fprintf(stderr, "FAIL legacy manual identification workflow\n");
        return 1;
    }
    printf("PASS: identification IQ state regression in %u cycles "
           "ad0=%.8f add=%.8f aq0=%.8f aqq=%.8f adq=%.8f\n",
           iteration + 1U,
           result_ad0, result_add, result_aq0, result_aqq, result_adq);
    return 0;
}
