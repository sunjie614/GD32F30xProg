#include <stdio.h>
#include <math.h>
#include <string.h>
#include "foc_iq.h"
#include "fixed_numeric_config.h"
#include "pid.h" /* Actual legacy inline Pid_Update, not an IQ reimplementation. */

static unsigned Failures;

static void expect_true(const char* name, bool condition)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL %s\n", name);
        Failures++;
    }
}

static FocIqInput_t zero_input(void)
{
    return (FocIqInput_t){
        .current_abc_pu = {MC_ZERO, MC_ZERO, MC_ZERO},
        .measured_angle_pu = MC_CONST(0.125),
        .measured_speed_pu = MC_ZERO,
        .bus_voltage_pu = MC_CONST(0.5)};
}

static bool neutral_pwm(const FocIqOutput_t* output)
{
    return output->pwm_duty.a == MC_HALF
        && output->pwm_duty.b == MC_HALF
        && output->pwm_duty.c == MC_HALF;
}

static void contaminate_control_state(FocIqState_t* state)
{
    PidIqState_t* pids[] = {&state->speed_pid, &state->current_d_pid,
                            &state->current_q_pid};
    for (unsigned index = 0U; index < sizeof(pids) / sizeof(pids[0]); ++index)
    {
        pids[index]->integral = MC_CONST(0.3);
        pids[index]->last_error = MC_CONST(0.2);
        pids[index]->output = MC_CONST(0.1);
    }
    state->speed_ramp.value = MC_CONST(0.4);
    state->vf_phase.phase_pu = MC_CONST(0.7);
    state->vf_phase.held_phase_pu = MC_CONST(0.6);
    state->if_phase.phase_pu = MC_CONST(0.7);
    state->if_phase.held_phase_pu = MC_CONST(0.6);
    state->startup_count = 29U;
    state->startup_active = true;
    state->speed_counter = 17U;
}

static void expect_active_mode_reset(const FocIqState_t* state, FocMode_t mode)
{
    if (mode == IF_MODE || mode == SPEED)
        expect_true("active current mode resets current PI",
        state->current_d_pid.integral == MC_ZERO
        && state->current_d_pid.last_error == MC_ZERO
        && state->current_d_pid.output == MC_ZERO
        && state->current_q_pid.integral == MC_ZERO
        && state->current_q_pid.last_error == MC_ZERO
        && state->current_q_pid.output == MC_ZERO);
    else
        expect_true("inactive current PI memory retained like float",
            state->current_d_pid.integral == MC_CONST(0.3)
            && state->current_q_pid.integral == MC_CONST(0.3));
    if (mode == SPEED)
        expect_true("speed reset clears speed PI/ramp/preparation",
        state->speed_pid.integral == MC_ZERO
        && state->speed_pid.last_error == MC_ZERO
        && state->speed_pid.output == MC_ZERO
        && state->speed_counter == 0U
        && state->speed_ramp.value == MC_ZERO
        && state->startup_count == 0U && !state->startup_active);
    else
        expect_true("inactive speed PI/ramp memory retained like float",
            state->speed_pid.integral == MC_CONST(0.3)
            && state->speed_ramp.value == MC_CONST(0.4));
}

static void held_reset_regression(void)
{
    FocIqState_t state;
    FocIqConfig_t config;
    FocIqParameters_t parameters;
    FocIqInput_t input = zero_input();
    FocIqOutput_t output;
    FocIq_DefaultConfig(&config);
    FocIq_DefaultParameters(&parameters);
    FocIq_Init(&state, &config);

    /* IDLE is a reset condition even when the caller's Stop input is false.
     * Metadata sentinels detect an unwanted experiment reset every cycle. */
    parameters.mode = IDLE;
    parameters.reset = false;
    for (unsigned cycle = 0U; cycle < 64U; ++cycle)
    {
        contaminate_control_state(&state);
        state.identification.sample_count = 17U;
        state.identification.dq_sample_count = 19U;
        state.identification.dq_sum_xx = 23;
        input.measured_angle_pu = MC_CONST(0.125)
                                + (mc_real_t)(cycle * MC_CONST(0.001));
        FocIq_Run(&state, &parameters, &input, &output);
        expect_true("continuous idle neutral pwm", neutral_pwm(&output));
        expect_true("continuous idle zero voltage",
            output.voltage_reference_dq_pu.d == MC_ZERO
            && output.voltage_reference_dq_pu.q == MC_ZERO
            && output.voltage_reference_ab_pu.a == MC_ZERO
            && output.voltage_reference_ab_pu.b == MC_ZERO);
        expect_active_mode_reset(&state, IDLE);
        expect_true("idle experiment resets only on entry",
            state.identification.sample_count == (cycle == 0U ? 0U : 17U)
            && state.identification.dq_sample_count
                == (cycle == 0U ? 0U : 19U)
            && state.identification.dq_sum_xx == (cycle == 0U ? 0 : 23));
    }

    const FocMode_t stopped_modes[] = {VF_MODE, IF_MODE, SPEED, IDENTIFY};
    parameters.reset = true;
    parameters.vf_voltage_reference_pu.d = MC_CONST(0.05);
    parameters.vf_frequency_step_pu = MC_CONST(0.001);
    parameters.if_frequency_step_pu = MC_CONST(0.002);
    parameters.identification_start_rs = true;
    parameters.identification_start_axis = true;
    parameters.identification_injection_enable = true;
    for (unsigned mode = 0U;
         mode < sizeof(stopped_modes) / sizeof(stopped_modes[0]); ++mode)
    {
        parameters.mode = stopped_modes[mode];
        for (unsigned cycle = 0U; cycle < 8U; ++cycle)
        {
            contaminate_control_state(&state);
            state.identification.sample_count = 17U;
            state.identification.dq_sample_count = 19U;
            state.identification.dq_sum_xx = 23;
            input.measured_angle_pu = MC_CONST(0.25)
                                    + (mc_real_t)(cycle * MC_CONST(0.003));
            FocIq_Run(&state, &parameters, &input, &output);
            expect_true("all stopped modes neutral pwm", neutral_pwm(&output));
            expect_active_mode_reset(&state, parameters.mode);
            expect_true("stopped mode transition resets experiment once",
                state.identification.sample_count == (cycle == 0U ? 0U : 17U)
                && state.identification.dq_sample_count
                    == (cycle == 0U ? 0U : 19U)
                && state.identification.dq_sum_xx == (cycle == 0U ? 0 : 23));
            if (parameters.mode == VF_MODE || parameters.mode == IF_MODE)
            {
                mc_real_t step = parameters.mode == VF_MODE
                               ? parameters.vf_frequency_step_pu
                               : parameters.if_frequency_step_pu;
                expect_true("legacy stopped phase starts at zero plus step",
                    output.control_angle_pu == step);
            }
            if (parameters.mode == VF_MODE)
                expect_true("stopped vf retains legacy voltage mirror",
                    output.voltage_reference_dq_pu.d
                        == parameters.vf_voltage_reference_pu.d
                    && output.voltage_reference_dq_pu.q
                        == parameters.vf_voltage_reference_pu.q);
            if (parameters.mode == IDENTIFY)
                expect_true("held identify cannot start injection",
                    output.identification_state == IDENTIFICATION_IQ_WAIT
                    && output.voltage_reference_dq_pu.d == MC_ZERO
                    && output.voltage_reference_dq_pu.q == MC_ZERO
                    && !state.identification.start_requested);
        }
    }

    parameters.identification_start_rs = false;
    parameters.identification_start_axis = false;
    parameters.identification_injection_enable = false;
    parameters.reset = false;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("identify resumes normally after held reset",
        output.identification_state == IDENTIFICATION_IQ_EST_RS);

    parameters.mode = SPEED;
    parameters.reset = true;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("speed mode change while stopped remains neutral",
        neutral_pwm(&output) && !output.startup_active);
    parameters.reset = false;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("speed startup retained after held mode change",
        output.startup_active
        && output.voltage_reference_dq_pu.d == config.startup_voltage_d_pu);

    /* Stop can also rise without any mode change; this must invalidate the
     * experiment once and still neutralize every subsequent PWM output. */
    parameters.reset = true;
    state.identification.sample_count = 17U;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("same mode stop entry resets experiment",
        neutral_pwm(&output) && !output.startup_active
        && state.identification.sample_count == 0U);
    state.identification.sample_count = 17U;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("same mode sustained stop does not repeat experiment reset",
        neutral_pwm(&output) && !output.startup_active
        && state.identification.sample_count == 17U);
}

static void expect_near(const char* name, mc_real_t actual, float expected,
                        float tolerance)
{
    float value = McMath_ToFloat(actual);
    if (!isfinite(value) || fabsf(value - expected) > tolerance)
    {
        fprintf(stderr, "FAIL %s: IQ=%g old=%g\n", name, value, expected);
        Failures++;
    }
}

typedef struct
{
    float counter;
    float held[2];
    bool sweep[2];
    bool reset_previous[2];
} LegacyPhaseReference_t;

static float legacy_phase_step(LegacyPhaseReference_t* old, unsigned mode,
                               float step, float offset, bool sweep, bool reset)
{
    /* Direct PU scaling of signal.h SawtoothWaveGenerator and foc.c VF/IF:
     * one shared counter, per-mode reset_prev/phase_prev/sweep_flag. */
    if ((old->reset_previous[mode] && !reset) || reset)
        old->counter = 0.0F;
    old->counter += step;
    if (step >= 0.0F)
    {
        if (old->counter > 1.0F) old->counter -= 1.0F;
        else if (old->counter < 0.0F) old->counter = 0.0F;
    }
    else
    {
        if (old->counter < 0.0F) old->counter += 1.0F;
        else if (old->counter > 1.0F) old->counter = 1.0F;
    }
    if (fabsf(old->counter - old->held[mode]) > 0.25F)
        old->sweep[mode] = false;
    if (sweep) old->sweep[mode] = true;
    if (old->sweep[mode]) old->held[mode] = old->counter;
    old->reset_previous[mode] = reset;
    float result = old->held[mode] + offset;
    while (result >= 1.0F) result -= 1.0F;
    while (result < 0.0F) result += 1.0F;
    return result;
}

static void legacy_phase_parity(void)
{
    FocIqState_t state;
    FocIqConfig_t config;
    FocIqParameters_t parameters;
    FocIqInput_t input = zero_input();
    FocIqOutput_t output;
    LegacyPhaseReference_t old = {.reset_previous = {true, true}};
    FocIq_DefaultConfig(&config);
    FocIq_DefaultParameters(&parameters);
    FocIq_Init(&state, &config);
    parameters.current_d_kp = MC_ZERO;
    parameters.current_q_kp = MC_ZERO;
    parameters.current_d_ki_step = MC_ZERO;
    parameters.current_q_ki_step = MC_ZERO;
    for (unsigned cycle = 0U; cycle < 112U; ++cycle)
    {
        unsigned mode = cycle >= 48U && cycle < 72U ? 1U : 0U;
        bool reset = cycle < 3U || cycle == 100U || cycle == 101U;
        bool sweep = cycle < 10U || (cycle >= 38U && cycle < 56U)
                   || cycle >= 92U;
        float step = cycle >= 80U ? -0.0625F : 0.0625F;
        if (cycle >= 38U && cycle < 42U) step = 0.3125F;
        float offset = mode == 0U ? 0.125F : -0.0625F;
        parameters.mode = mode == 0U ? VF_MODE : IF_MODE;
        parameters.reset = reset;
        parameters.sweep_request = sweep;
        parameters.if_use_sensor = false;
        parameters.vf_frequency_step_pu = McMath_FromFloat(step);
        parameters.if_frequency_step_pu = McMath_FromFloat(step);
        parameters.vf_offset_pu = McMath_FromFloat(offset);
        parameters.if_offset_pu = McMath_FromFloat(offset);
        input.measured_angle_pu = MC_CONST(0.37);
        float expected = legacy_phase_step(&old, mode, step, offset, sweep, reset);
        FocIq_Run(&state, &parameters, &input, &output);
        expect_near("VF/IF shared phase/reset/freeze old parity",
                    output.control_angle_pu, expected, 0.000003F);
        expect_near("VF/IF sawtooth shared counter old parity",
                    state.shared_phase_pu, old.counter, 0.000003F);
        expect_true("held reset still neutralizes hardware duty",
            !reset || neutral_pwm(&output));
    }
}

static void legacy_pid_parity(void)
{
    FocIqState_t state;
    FocIqConfig_t config;
    FocIqParameters_t parameters;
    FocIqInput_t input = zero_input();
    FocIqOutput_t output;
    FocIq_DefaultConfig(&config);
    FocIq_DefaultParameters(&parameters);
    FocIq_Init(&state, &config);
    PID_Handler_t old = {.Kp = 0.2F, .Ki = 0.5F, .Kd = 0.2F,
        .Ts = 0.1F, .MinOutput = -1.0F, .MaxOutput = 1.0F,
        .IntegralLimit = 0.5F};
    parameters.mode = IF_MODE;
    parameters.if_use_sensor = true;
    parameters.current_d_kp = MC_CONST(0.2);
    parameters.current_d_ki_step = MC_CONST(0.05);
    parameters.current_d_kd_step = MC_CONST(2.0);
    parameters.current_d_minimum_pu = MC_CONST(-1.0);
    parameters.current_d_maximum_pu = MC_ONE;
    parameters.current_d_integral_limit_pu = MC_HALF;
    const float errors[] = {0.75F, 0.0F, -0.5F, 0.9F, 0.2F, -0.1F};
    for (unsigned cycle = 0U; cycle < 72U; ++cycle)
    {
        parameters.reset = cycle == 21U || cycle == 22U;
        if (cycle == 36U)
        {
            old.Kd = 0.0F;
            parameters.current_d_kd_step = MC_ZERO;
        }
        float error = errors[cycle % 6U];
        parameters.if_current_reference_pu.d = McMath_FromFloat(error);
        float expected = Pid_Update(error, parameters.reset, &old);
        FocIq_Run(&state, &parameters, &input, &output);
        expect_near("current PI old actual Pid_Update output",
            output.voltage_reference_dq_pu.d, expected, 0.000008F);
        expect_near("current PI old actual conditional integral",
            state.current_d_pid.integral, old.integral, 0.000008F);
        expect_near("current PI old Kd=0 previous_error",
            state.current_d_pid.last_error, old.previous_error, 0.000008F);
    }
}

static void legacy_speed_timing_parity(void)
{
    FocIqState_t state;
    FocIqConfig_t config;
    FocIqParameters_t parameters;
    FocIqInput_t input = zero_input();
    FocIqOutput_t output;
    FocIq_DefaultConfig(&config);
    FocIq_DefaultParameters(&parameters);
    config.startup_hold_cycles = 3U;
    config.speed_prescaler = 4U;
    config.speed_ramp_step_pu = MC_CONST(0.03125);
    config.speed_ramp_minimum_pu = MC_CONST(-0.125);
    config.speed_ramp_maximum_pu = MC_CONST(0.125);
    FocIq_Init(&state, &config);
    parameters.mode = SPEED;
    parameters.reset = false;
    parameters.speed_reference_pu = MC_CONST(0.5);
    for (unsigned cycle = 0U; cycle < 3U; ++cycle)
    {
        state.current_d_pid.integral = MC_CONST(0.2);
        FocIq_Run(&state, &parameters, &input, &output);
        expect_true("old SPEED preparation fixed d voltage",
            output.voltage_reference_dq_pu.d == config.startup_voltage_d_pu
            && output.voltage_reference_dq_pu.q == MC_ZERO);
        expect_true("old preparation bypasses current PI without clearing it",
            state.current_d_pid.integral == MC_CONST(0.2));
        expect_true("old preparation resets speed ramp and counter",
            state.speed_ramp.value == MC_ZERO && state.speed_counter == 0U);
    }
    float old_ramp = 0.0F;
    unsigned old_counter = 0U;
    for (unsigned cycle = 0U; cycle < 24U; ++cycle)
    {
        if (++old_counter >= 4U)
        {
            old_counter = 0U;
            float delta = 0.5F - old_ramp;
            if (delta > 0.03125F) old_ramp += 0.03125F;
            else if (delta < -0.03125F) old_ramp -= 0.03125F;
            else old_ramp = 0.5F;
            if (old_ramp > 0.125F) old_ramp = 0.125F;
            else if (old_ramp < -0.125F) old_ramp = -0.125F;
        }
        FocIq_Run(&state, &parameters, &input, &output);
        expect_near("speed ramp old prescaler/limit parity",
                    state.speed_ramp.value, old_ramp, 0.000003F);
        expect_true("old SPEED counter timing",
                    state.speed_counter == old_counter && !output.startup_active);
    }
    parameters.mode = STARTUP;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("legacy STARTUP remains STARTUP, no fabricated auto transition",
        output.requested_mode == STARTUP
        && output.voltage_reference_dq_pu.d == config.startup_voltage_d_pu);
    parameters.mode = SPEED;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("mode enum change alone does not retrigger preparation",
        !output.startup_active);

    state.speed_counter = config.speed_prescaler - 1U;
    state.speed_ramp.value = MC_CONST(0.05);
    state.speed_reset_previous = false;
    SpeedPidScheduleIq_Reset(&state.speed_schedule);
    input.measured_speed_pu = MC_ZERO;
    FocIq_Run(&state, &parameters, &input, &output);
    /* Old speed_pi_update_gain sees 90rpm before this speed tick advances
     * the ramp: 1+(90-80)/(130-80)*(5-1)=1.8, not the new-ramp gain 5. */
    expect_near("old segmented Kp uses pre-ramp error",
        state.speed_schedule.kp_multiplier, 1.8F, 0.0001F);

    /* The original scheduler uses the pre-update ramp error every current
     * cycle, not the new ramp only at the speed prescaler boundary. */
    state.speed_counter = 0U;
    state.speed_ramp.value = MC_ZERO;
    state.speed_reset_previous = false;
    SpeedPidScheduleIq_Reset(&state.speed_schedule);
    input.measured_speed_pu = MC_CONST(-0.2);
    for (unsigned cycle = 0U; cycle < 8U; ++cycle)
    {
        FocIq_Run(&state, &parameters, &input, &output);
        expect_true("legacy Ki confirmation counts every current cycle",
            state.speed_schedule.ki_high_count == cycle + 1U);
        expect_true("legacy Ki confirm threshold equals cycles times prescaler",
            state.speed_schedule.ki_boosted == (cycle == 7U));
    }
    parameters.reset = true;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("speed stop resets active PI/ramp/preparation",
        neutral_pwm(&output) && state.speed_ramp.value == MC_ZERO
        && state.speed_pid.integral == MC_ZERO && !state.startup_active);
}

static void make_small_mtpa_table(FocIqState_t* state)
{
    for (unsigned i = 0U; i < MTPA_IQ_TABLE_POINTS; ++i)
        state->mtpa.table[i] = (MtpaIqPoint_t){
            .iq_pu = McMath_FromFloat((float)i * 0.005F),
            .id_pu = MC_CONST(0.05), .ld_pu = MC_CONST(3.0),
            .lq_pu = MC_CONST(0.48), .valid = true};
    state->mtpa.table_valid = true;
}

static FocIqOutput_t run_speed_current_reference(FocIqState_t* state,
                                                mc_real_t signed_iq)
{
    FocIqParameters_t parameters;
    FocIq_DefaultParameters(&parameters);
    parameters.mode = SPEED;
    parameters.reset = false;
    parameters.speed_kp = parameters.speed_ki_step = parameters.speed_kd_step = MC_ZERO;
    FocIqInput_t input = zero_input();
    FocIqOutput_t output;
    state->mode = state->previous_mode = SPEED;
    state->reset_previous = state->speed_reset_previous = false;
    state->startup_active = state->startup_request_latched = false;
    state->config.speed_prescaler = 1U;
    state->speed_counter = 0U;
    state->speed_pid.integral = signed_iq;
    FocIq_Run(state, &parameters, &input, &output);
    return output;
}

static void mtpa_linear_fallback_regression(void)
{
    FocIqState_t state;
    FocIq_Init(&state, NULL);
    make_small_mtpa_table(&state);
    const mc_real_t requests[] = {MC_CONST(0.1), MC_CONST(-0.1),
        MC_CONST(0.25), MC_CONST(-0.25), MC_CONST(0.4), MC_CONST(-0.4)};
    for (unsigned i = 0U; i < sizeof(requests) / sizeof(requests[0]); ++i)
    {
        FocIqOutput_t output = run_speed_current_reference(&state, requests[i]);
        bool outside = McMath_Abs(requests[i]) > state.mtpa.table[50].iq_pu;
        expect_true("Iq keeps both direction and PI-limited request",
            output.current_reference_dq_pu.q == requests[i]);
        expect_true("range fallback explicitly reported", state.mtpa_linear_fallback == outside);
        expect_true("in range uses table, outside uses abs Iq",
            output.current_reference_dq_pu.d == (outside ? McMath_Abs(requests[i]) : MC_CONST(0.05)));
        expect_true("inductance always remains positive",
            output.inductance_dq_pu.d > MC_ZERO && output.inductance_dq_pu.q > MC_ZERO);
    }
    state.mtpa.table_valid = false;
    state.config.mtpa_fallback_inductance_dq_pu = (ParkIq_t){MC_CONST(2.7), MC_CONST(0.45)};
    for (int direction = -1; direction <= 1; direction += 2)
    {
        FocIqOutput_t output = run_speed_current_reference(&state,
            direction * MC_CONST(0.2));
        expect_true("invalid table enters symmetric linear fallback",
            state.mtpa_linear_fallback && output.current_reference_dq_pu.d == MC_CONST(0.2)
            && output.current_reference_dq_pu.q == direction * MC_CONST(0.2));
        expect_true("fallback uses explicit configured nominal inductances",
            output.inductance_dq_pu.d == MC_CONST(2.7)
            && output.inductance_dq_pu.q == MC_CONST(0.45));
        output = run_speed_current_reference(&state, direction * MC_CONST(1.25));
        expect_near("original q PI 24 A limit retained",
            output.current_reference_dq_pu.q, direction * 0.8F, 0.000001F);
        expect_near("fallback tracks magnitude after q limiting",
            output.current_reference_dq_pu.d, 0.8F, 0.000001F);
    }
    state.config.mtpa_fallback_inductance_dq_pu = (ParkIq_t){0};
    FocIqOutput_t output = run_speed_current_reference(&state, MC_ZERO);
    expect_true("zero Iq linear fallback is zero current",
        state.mtpa_linear_fallback && output.current_reference_dq_pu.d == MC_ZERO
        && output.current_reference_dq_pu.q == MC_ZERO);
    expect_near("missing nominal Ld falls back to normal 0.1 H", output.inductance_dq_pu.d, 3.0F, 0.000001F);
    expect_near("missing nominal Lq falls back to normal 0.016 H", output.inductance_dq_pu.q, 0.48F, 0.000001F);
    make_small_mtpa_table(&state);
    output = run_speed_current_reference(&state, MC_CONST(-0.1));
    expect_true("valid table recovery exits fallback without losing direction",
        !state.mtpa_linear_fallback && output.current_reference_dq_pu.d == MC_CONST(0.05)
        && output.current_reference_dq_pu.q == MC_CONST(-0.1));
}

static void failed_mtpa_candidate_is_not_old_model_regression(void)
{
    FocIqState_t state;
    FocIq_Init(&state, NULL);
    MtpaIqPoint_t old_point = state.mtpa.table[20];
    MtpaIqParameters_t old_parameters = state.mtpa.parameters;
    MtpaIq_DefaultParameters(&state.pending_mtpa_parameters);
    state.pending_mtpa_parameters.ad0 = state.pending_mtpa_parameters.aq0 = MC_CONST(100.0);
    state.pending_mtpa_parameters.add = state.pending_mtpa_parameters.aqq
        = state.pending_mtpa_parameters.adq = MC_ZERO;
    state.mode = IDLE;
    state.mtpa_rebuild_pending = true;
    expect_true("saturating pending model rejected", !FocIq_BackgroundBuildMtpa(&state));
    expect_true("failed candidate invalidates active lookup and records failure",
        !state.mtpa.table_valid && state.mtpa_rebuild_failed
        && !state.mtpa_staging_ready && !state.mtpa_rebuild_in_progress
        && (state.mtpa_staging.build_errors & MTPA_IQ_BUILD_SATURATION) != 0U);
    expect_true("old model bytes retained only for diagnosis",
        memcmp(&old_point, &state.mtpa.table[20], sizeof(old_point)) == 0
        && memcmp(&old_parameters, &state.mtpa.parameters, sizeof(old_parameters)) == 0);
    FocIq_CommitBackgroundMtpa(&state);
    expect_true("invalid candidate cannot be committed as valid", !state.mtpa.table_valid);
    FocIqOutput_t output = run_speed_current_reference(&state, MC_CONST(-0.2));
    expect_true("new model failure cannot silently use old MTPA dispatch",
        state.mtpa_linear_fallback && output.current_reference_dq_pu.d == MC_CONST(0.2)
        && output.current_reference_dq_pu.q == MC_CONST(-0.2));
    FocIq_Reinitialize(&state);
    expect_true("warm INIT does not silently resurrect rejected old table",
        !state.mtpa.table_valid && !state.mtpa_linear_fallback);
    MtpaIq_DefaultParameters(&state.pending_mtpa_parameters);
    state.mode = IDLE;
    state.mtpa_rebuild_pending = true;
    expect_true("valid next candidate builds", FocIq_BackgroundBuildMtpa(&state));
    expect_true("candidate not active until commit", !state.mtpa.table_valid);
    FocIq_CommitBackgroundMtpa(&state);
    expect_true("valid next model committed and failure cleared",
        state.mtpa.table_valid && !state.mtpa_rebuild_failed && !state.mtpa_rebuild_in_progress);
    output = run_speed_current_reference(&state, MC_CONST(-0.2));
    expect_true("successful model restores lookup", !state.mtpa_linear_fallback
        && output.current_reference_dq_pu.q == MC_CONST(-0.2));
}

int main(void)
{
    FocIqState_t state;
    FocIqConfig_t config;
    FocIqParameters_t parameters;
    FocIqOutput_t output;
    FocIqInput_t input = zero_input();
    FocIq_DefaultConfig(&config);
    FocIq_DefaultParameters(&parameters);
    FocIq_Init(&state, &config);
    expect_true("init", state.initialized && state.mtpa.table_valid);

    parameters.mode = SPEED;
    parameters.reset = true;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("speed held in reset", !output.startup_active);
    parameters.reset = false;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("startup on reset release", output.startup_active);
    expect_true("startup d voltage",
                output.voltage_reference_dq_pu.d
                    == config.startup_voltage_d_pu);

    parameters.mode = VF_MODE;
    parameters.vf_voltage_reference_pu.d = MC_CONST(0.05);
    parameters.vf_frequency_step_pu = MC_CONST(0.001);
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("vf duty a", output.pwm_duty.a >= MC_ZERO
                              && output.pwm_duty.a <= MC_ONE);
    expect_true("vf duty b", output.pwm_duty.b >= MC_ZERO
                              && output.pwm_duty.b <= MC_ONE);
    expect_true("vf duty c", output.pwm_duty.c >= MC_ZERO
                              && output.pwm_duty.c <= MC_ONE);
    input.bus_voltage_pu = MC_CONST(80.0F / MC_VOLTAGE_BASE_V);
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("legacy low-bus neutral pwm",
                output.pwm_duty.a == MC_HALF
                && output.pwm_duty.b == MC_HALF
                && output.pwm_duty.c == MC_HALF);
    input.bus_voltage_pu = MC_CONST(120.0F / MC_VOLTAGE_BASE_V);
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("legacy normal-bus modulation",
                output.pwm_duty.a != MC_HALF
                || output.pwm_duty.b != MC_HALF
                || output.pwm_duty.c != MC_HALF);
    input.bus_voltage_pu = MC_CONST(0.5);

    parameters.mode = IDENTIFY;
    parameters.reset = true;
    FocIq_Run(&state, &parameters, &input, &output);
    parameters.reset = false;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("identification on reset release",
                output.identification_state == IDENTIFICATION_IQ_EST_RS);

    parameters.mode = (FocMode_t)99;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("invalid mode fallback", output.requested_mode == IDLE);
    expect_true("invalid mode neutral pwm",
                output.pwm_duty.a == MC_HALF
                && output.pwm_duty.b == MC_HALF
                && output.pwm_duty.c == MC_HALF);

    MtpaIq_DefaultParameters(&state.pending_mtpa_parameters);
    state.mtpa_rebuild_pending = true;
    state.mode = IDLE;
    expect_true("background build", FocIq_BackgroundBuildMtpa(&state));
    expect_true("background forces idle", state.mtpa_rebuild_in_progress);
    FocIq_CommitBackgroundMtpa(&state);
    expect_true("background commit", state.mtpa.table_valid
                                     && !state.mtpa_rebuild_in_progress);

    held_reset_regression();
    legacy_phase_parity();
    legacy_pid_parity();
    legacy_speed_timing_parity();
    mtpa_linear_fallback_regression();
    failed_mtpa_candidate_is_not_old_model_regression();

    if (Failures != 0U)
        return 1;
    puts("PASS: FOC IQ state regression");
    return 0;
}
