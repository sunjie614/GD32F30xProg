#include "foc_iq.h"

#include <stddef.h>
#include "fixed_numeric_config.h"
#include "parameters.h"

static bool foc_iq_mode_is_valid(FocMode_t mode)
{
    return (uint32_t)mode <= (uint32_t)IDENTIFY;
}

static PhaseIq_t foc_iq_neutral_pwm(void)
{
    return (PhaseIq_t){MC_HALF, MC_HALF, MC_HALF};
}

static PhaseIq_t foc_iq_svpwm(ClarkIq_t voltage, mc_real_t bus_voltage_pu)
{
    PhaseIq_t duty = foc_iq_neutral_pwm();
    /* Legacy Adc_Get_VoltageBusInv() suppresses modulation below 100 V.
     * Preserve that hardware-side boundary after per-unit conversion. */
    if (bus_voltage_pu < MC_CONST(100.0F / MC_VOLTAGE_BASE_V))
        return duty;

    mc_real_t inverse_bus = McMath_Div(MC_ONE, bus_voltage_pu);
    mc_real_t x = McMath_Mul(McMath_Mul(MC_CONST(1.7320508075689), voltage.b),
                             inverse_bus);
    mc_real_t y = McMath_Mul(McMath_Add(
        McMath_Mul(MC_CONST(1.5), voltage.a),
        McMath_Mul(MC_CONST(0.8660254037844), voltage.b)), inverse_bus);
    mc_real_t z = McMath_Mul(McMath_Add(
        McMath_Mul(MC_CONST(-1.5), voltage.a),
        McMath_Mul(MC_CONST(0.8660254037844), voltage.b)), inverse_bus);
    mc_real_t v1 = voltage.b;
    mc_real_t v2 = McMath_Mul(McMath_Sub(
        McMath_Mul(MC_CONST(1.7320508075689), voltage.a), voltage.b), MC_HALF);
    mc_real_t v3 = McMath_Mul(McMath_Sub(
        McMath_Mul(MC_CONST(-1.7320508075689), voltage.a), voltage.b), MC_HALF);
    uint8_t sector = (v1 > MC_ZERO ? 1U : 0U)
                   + (v2 > MC_ZERO ? 2U : 0U)
                   + (v3 > MC_ZERO ? 4U : 0U);
    mc_real_t t1 = MC_ZERO;
    mc_real_t t2 = MC_ZERO;
    switch (sector)
    {
    case 1U: t1 = z; t2 = y; break;
    case 2U: t1 = y; t2 = McMath_Neg(x); break;
    case 3U: t1 = McMath_Neg(z); t2 = x; break;
    case 4U: t1 = McMath_Neg(x); t2 = z; break;
    case 5U: t1 = x; t2 = McMath_Neg(y); break;
    case 6U: t1 = McMath_Neg(y); t2 = McMath_Neg(z); break;
    default: return duty;
    }
    mc_real_t sum = McMath_Add(t1, t2);
    if (sum > MC_ONE)
    {
        t1 = McMath_Div(t1, sum);
        t2 = McMath_Div(t2, sum);
    }
    mc_real_t ta = McMath_Mul(
        McMath_Sub(MC_ONE, McMath_Add(t1, t2)), MC_HALF);
    mc_real_t tb = McMath_Add(ta, t1);
    mc_real_t tc = McMath_Add(tb, t2);
    switch (sector)
    {
    case 1U: duty = (PhaseIq_t){tb, ta, tc}; break;
    case 2U: duty = (PhaseIq_t){ta, tc, tb}; break;
    case 3U: duty = (PhaseIq_t){ta, tb, tc}; break;
    case 4U: duty = (PhaseIq_t){tc, tb, ta}; break;
    case 5U: duty = (PhaseIq_t){tc, ta, tb}; break;
    case 6U: duty = (PhaseIq_t){tb, tc, ta}; break;
    default: break;
    }
    duty.a = McMath_Clamp(duty.a, MC_ZERO, MC_ONE);
    duty.b = McMath_Clamp(duty.b, MC_ZERO, MC_ONE);
    duty.c = McMath_Clamp(duty.c, MC_ZERO, MC_ONE);
    return duty;
}

void FocIq_DefaultConfig(FocIqConfig_t* config)
{
    if (config == NULL)
        return;
    *config = (FocIqConfig_t){
        .speed_prescaler = (uint16_t)SPEED_LOOP_PRESCALER,
        .startup_hold_cycles = (uint16_t)(
            SENSORLESS_STARTUP_HOLD_TIME * MAIN_LOOP_FREQ + 0.5),
        .flying_startup_delay = 8U,
        .speed_ramp_step_pu = MC_CONST(
            RAMP_SPEED_SLOPE * SPEED_LOOP_TIME / MC_SPEED_BASE_RPM),
        .speed_ramp_minimum_pu = MC_CONST(
            RAMP_SPEED_LIMIT_MIN / MC_SPEED_BASE_RPM),
        .speed_ramp_maximum_pu = MC_CONST(
            RAMP_SPEED_LIMIT_MAX / MC_SPEED_BASE_RPM),
        .speed_q_voltage_limit_pu = MC_CONST(
            PID_CURRENT_Q_LOOP_MAX_OUTPUT / MC_VOLTAGE_BASE_V),
        .sensorless_switch_speed_pu = MC_CONST(
            SENSORLESS_SWITCH_SPEED / MC_SPEED_BASE_RPM),
        .sensorless_hysteresis_pu = MC_CONST(
            SENSORLESS_HYSTERESIS / MC_SPEED_BASE_RPM),
        .startup_voltage_d_pu = MC_CONST(
            SENSORLESS_STARTUP_UD / MC_VOLTAGE_BASE_V),
        .mtpa_fallback_inductance_dq_pu = {
            MC_CONST(MOTOR_LD * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB),
            MC_CONST(MOTOR_LQ * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB)}};
    config->speed_schedule = (SpeedPidScheduleIqConfig_t){
        .kp_base = MC_CONST(
            PID_SPEED_LOOP_KP * MC_SPEED_BASE_RPM / MC_CURRENT_BASE_A),
        .ki_step_base = MC_CONST(
            PID_SPEED_LOOP_KI * SPEED_LOOP_TIME
            * MC_SPEED_BASE_RPM / MC_CURRENT_BASE_A),
        .kp_multiplier_max = MC_CONST(SPEED_PI_KP_MUL_MAX),
        .ki_multiplier_max = MC_CONST(SPEED_PI_KI_MUL_MAX),
        .kp_up_start_error = MC_CONST(
            SPEED_PI_KP_UP_START_ERR / MC_SPEED_BASE_RPM),
        .kp_up_end_error = MC_CONST(
            SPEED_PI_KP_UP_END_ERR / MC_SPEED_BASE_RPM),
        .kp_down_start_error = MC_CONST(
            SPEED_PI_KP_DOWN_START_ERR / MC_SPEED_BASE_RPM),
        .kp_down_end_error = MC_CONST(
            SPEED_PI_KP_DOWN_END_ERR / MC_SPEED_BASE_RPM),
        .ki_switch_error = MC_CONST(
            SPEED_PI_KI_SWITCH_ERR / MC_SPEED_BASE_RPM),
        .enable_reference_minimum = MC_CONST(
            SPEED_PI_ENABLE_REF_MIN / MC_SPEED_BASE_RPM),
        .ki_confirm_cycles = SPEED_PI_KI_CONFIRM_CYCLES};
    LesoIq_DefaultConfig(&config->leso);
    HfiIq_DefaultConfig(&config->hfi);
    SensorlessIq_DefaultConfig(&config->sensorless);
}

void FocIq_DefaultParameters(FocIqParameters_t* parameters)
{
    if (parameters == NULL)
        return;
    *parameters = (FocIqParameters_t){
        .mode = IDLE,
        .reset = true,
        .sweep_request = true,
        .use_real_angle = true,
        .speed_kp = MC_CONST(
            PID_SPEED_LOOP_KP * MC_SPEED_BASE_RPM / MC_CURRENT_BASE_A),
        .speed_ki_step = MC_CONST(
            PID_SPEED_LOOP_KI * SPEED_LOOP_TIME
            * MC_SPEED_BASE_RPM / MC_CURRENT_BASE_A),
        .speed_kd_step = MC_ZERO,
        .speed_minimum_pu = MC_CONST(
            PID_SPEED_LOOP_MIN_OUTPUT / MC_CURRENT_BASE_A),
        .speed_maximum_pu = MC_CONST(
            PID_SPEED_LOOP_MAX_OUTPUT / MC_CURRENT_BASE_A),
        .speed_integral_limit_pu = MC_CONST(
            PID_SPEED_LOOP_INTEGRAL_LIMIT / MC_CURRENT_BASE_A),
        .current_d_kp = MC_CONST(
            PID_CURRENT_D_LOOP_KP * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        .current_d_ki_step = MC_CONST(
            PID_CURRENT_D_LOOP_KI * MAIN_LOOP_TIME
            * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        .current_d_minimum_pu = MC_CONST(
            PID_CURRENT_D_LOOP_MIN_OUTPUT / MC_VOLTAGE_BASE_V),
        .current_d_maximum_pu = MC_CONST(
            PID_CURRENT_D_LOOP_MAX_OUTPUT / MC_VOLTAGE_BASE_V),
        .current_d_integral_limit_pu = MC_CONST(
            PID_CURRENT_D_LOOP_INTEGRAL_LIMIT / MC_VOLTAGE_BASE_V),
        .current_q_kp = MC_CONST(
            PID_CURRENT_Q_LOOP_KP * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        .current_q_ki_step = MC_CONST(
            PID_CURRENT_Q_LOOP_KI * MAIN_LOOP_TIME
            * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        .current_q_minimum_pu = MC_CONST(
            PID_CURRENT_Q_LOOP_MIN_OUTPUT / MC_VOLTAGE_BASE_V),
        .current_q_maximum_pu = MC_CONST(
            PID_CURRENT_Q_LOOP_MAX_OUTPUT / MC_VOLTAGE_BASE_V),
        .current_q_integral_limit_pu = MC_CONST(
            PID_CURRENT_Q_LOOP_INTEGRAL_LIMIT / MC_VOLTAGE_BASE_V)};
    IdentificationIq_DefaultConfig(&parameters->identification_config);
}

static void foc_iq_apply_pid_parameters(FocIqState_t* state,
                                        const FocIqParameters_t* parameters)
{
    state->config.speed_schedule.kp_base = parameters->speed_kp;
    state->config.speed_schedule.ki_step_base = parameters->speed_ki_step;
    state->speed_pid.kd_step = parameters->speed_kd_step;
    state->speed_pid.minimum = parameters->speed_minimum_pu;
    state->speed_pid.maximum = parameters->speed_maximum_pu;
    state->speed_pid.integral_limit = parameters->speed_integral_limit_pu;
    state->current_d_pid.kp = parameters->current_d_kp;
    state->current_d_pid.ki_step = parameters->current_d_ki_step;
    state->current_d_pid.kd_step = parameters->current_d_kd_step;
    state->current_d_pid.minimum = parameters->current_d_minimum_pu;
    state->current_d_pid.maximum = parameters->current_d_maximum_pu;
    state->current_d_pid.integral_limit = parameters->current_d_integral_limit_pu;
    state->current_q_pid.kp = parameters->current_q_kp;
    state->current_q_pid.ki_step = parameters->current_q_ki_step;
    state->current_q_pid.kd_step = parameters->current_q_kd_step;
    state->current_q_pid.minimum = parameters->current_q_minimum_pu;
    state->current_q_pid.maximum = parameters->current_q_maximum_pu;
    state->current_q_pid.integral_limit = parameters->current_q_integral_limit_pu;
}

void FocIq_Init(FocIqState_t* state, const FocIqConfig_t* config)
{
    if (state == NULL)
        return;
    *state = (FocIqState_t){0};
    if (config == NULL)
        FocIq_DefaultConfig(&state->config);
    else
        state->config = *config;
    state->mode = IDLE;
    state->previous_mode = IDLE;
    state->vf_reset_previous = true;
    state->if_reset_previous = true;
    state->speed_reset_previous = true;
    RampIq_Init(&state->speed_ramp, MC_ZERO);
    PhaseGeneratorIq_Init(&state->vf_phase, MC_ZERO);
    PhaseGeneratorIq_Init(&state->if_phase, MC_ZERO);
    SpeedPidScheduleIq_Init(&state->speed_schedule);
    PidIq_Init(&state->speed_pid,
               state->config.speed_schedule.kp_base,
               state->config.speed_schedule.ki_step_base,
               MC_ZERO, MC_CONST(-1.0), MC_CONST(1.0), MC_CONST(1.0));
    PidIq_Init(&state->current_d_pid, MC_ZERO, MC_ZERO, MC_ZERO,
               MC_CONST(-1.0), MC_CONST(1.0), MC_CONST(1.0));
    PidIq_Init(&state->current_q_pid, MC_ZERO, MC_ZERO, MC_ZERO,
               MC_CONST(-1.0), MC_CONST(1.0), MC_CONST(1.0));
    MtpaIq_Init(&state->mtpa, NULL);
    (void)MtpaIq_BuildTable(&state->mtpa, MC_ZERO, MC_ONE);
    IdentificationIq_Init(&state->identification, NULL);
    LesoIq_Init(&state->leso, &state->config.leso);
    HfiIq_Init(&state->hfi, &state->config.hfi);
    FlyingIq_Init(&state->flying, state->config.flying_startup_delay);
    SensorlessIq_Init(&state->sensorless, &state->config.sensorless);
    state->initialized = true;
}

void FocIq_Reinitialize(FocIqState_t* state)
{
    if (state == NULL || !state->initialized)
        return;
    /* Initialization_Modules in the float firmware resets parameters/PI and
     * the common speed filter, not the observer angle/current histories or
     * the accepted MTPA model/table. Work in place: those arrays do not fit on
     * the target's 2 KB stack. These three settings were not reset by it. */
    mc_real_t adjustment = state->config.sensorless.theta_adjustment_pu;
    mc_real_t kp_maximum = state->config.sensorless.kp_maximum;
    mc_real_t ki_maximum = state->config.sensorless.ki_step_maximum;
    FocIq_DefaultConfig(&state->config);
    state->config.sensorless.theta_adjustment_pu = adjustment;
    state->config.sensorless.kp_maximum = kp_maximum;
    state->config.sensorless.ki_step_maximum = ki_maximum;
    RampIq_Init(&state->speed_ramp, MC_ZERO);
    PidIq_Init(&state->speed_pid,
        state->config.speed_schedule.kp_base,
        state->config.speed_schedule.ki_step_base, MC_ZERO,
        MC_CONST(PID_SPEED_LOOP_MIN_OUTPUT / MC_CURRENT_BASE_A),
        MC_CONST(PID_SPEED_LOOP_MAX_OUTPUT / MC_CURRENT_BASE_A),
        MC_CONST(PID_SPEED_LOOP_INTEGRAL_LIMIT / MC_CURRENT_BASE_A));
    PidIq_Init(&state->current_d_pid,
        MC_CONST(PID_CURRENT_D_LOOP_KP * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        MC_CONST(PID_CURRENT_D_LOOP_KI * MAIN_LOOP_TIME * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        MC_ZERO, MC_CONST(PID_CURRENT_D_LOOP_MIN_OUTPUT / MC_VOLTAGE_BASE_V),
        MC_CONST(PID_CURRENT_D_LOOP_MAX_OUTPUT / MC_VOLTAGE_BASE_V),
        MC_CONST(PID_CURRENT_D_LOOP_INTEGRAL_LIMIT / MC_VOLTAGE_BASE_V));
    PidIq_Init(&state->current_q_pid,
        MC_CONST(PID_CURRENT_Q_LOOP_KP * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        MC_CONST(PID_CURRENT_Q_LOOP_KI * MAIN_LOOP_TIME * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        MC_ZERO, MC_CONST(PID_CURRENT_Q_LOOP_MIN_OUTPUT / MC_VOLTAGE_BASE_V),
        MC_CONST(PID_CURRENT_Q_LOOP_MAX_OUTPUT / MC_VOLTAGE_BASE_V),
        MC_CONST(PID_CURRENT_Q_LOOP_INTEGRAL_LIMIT / MC_VOLTAGE_BASE_V));
    SensorlessIqConfig_t* config = &state->config.sensorless;
    SensorlessIqState_t* common = &state->sensorless;
    PidIq_Init(&common->common_pid, config->kp, config->ki_step,
        config->kd_step, config->minimum, config->maximum, config->integral_limit);
    BiquadIq_Init(&common->speed_filter, config->filter_b0, config->filter_b1,
        config->filter_b2, config->filter_a1, config->filter_a2);
    common->omega_pu = MC_ZERO;
    common->integral_product_residual = 0;
    common->previous_integral_output = MC_ZERO;
    common->pll_wc_pu = config->pll_wc_pu;
    /* Local PLLs remain inactive compatibility views. The original INIT did
     * reset their PID handlers without clearing their theta/speed memories. */
    PidIq_Init(&state->leso.pll.pid, state->config.leso.pll_kp,
        state->config.leso.pll_ki_step, MC_ZERO,
        MC_CONST(-1.5), MC_CONST(1.5), MC_CONST(1.5));
    PidIq_Init(&state->hfi.pll.pid, state->config.hfi.pll_kp,
        state->config.hfi.pll_ki_step, MC_ZERO,
        McMath_Neg(MC_ONE), MC_ONE, MC_ONE);
    state->leso.pll.speed_to_angle_step = state->config.leso.speed_to_angle_step;
    state->hfi.pll.speed_to_angle_step = state->config.hfi.speed_to_angle_step;
    state->leso.last_inductance_q_pu = state->config.leso.inductance_q_pu;
    /* The old synchronous table builder had no pending transaction. INIT
     * cancels the new background candidate; the last committed table/model,
     * last dynamic inductance and identification results stay untouched. */
    state->mtpa_rebuild_pending = false;
    state->mtpa_rebuild_in_progress = false;
    state->mtpa_staging_ready = false;
    state->mtpa_rebuild_failed = false;
    state->mtpa_staging.table_valid = false;
    state->mtpa_linear_fallback = false;
    state->mode = IDLE;
    state->previous_mode = IDLE;
    state->reset_previous = true; /* No synthetic first-IDLE experiment reset. */
}

static void foc_iq_reset_control(FocIqState_t* state,
                                 mc_real_t initial_angle_pu)
{
    if (state == NULL)
        return;
    state->speed_counter = 0U;
    state->startup_count = 0U;
    state->startup_active = false;
    state->startup_request_latched = false;
    state->current_reference_dq_pu = (ParkIq_t){0};
    state->voltage_reference_dq_pu = (ParkIq_t){0};
    state->voltage_reference_ab_pu = (ClarkIq_t){0};
    state->control_angle_pu = AngleIq_WrapPu(initial_angle_pu);
    RampIq_Reset(&state->speed_ramp, MC_ZERO);
    state->shared_phase_pu = MC_ZERO;
    state->vf_reset_previous = true;
    state->if_reset_previous = true;
    state->speed_reset_previous = true;
    PhaseGeneratorIq_Reset(&state->vf_phase, MC_ZERO);
    PhaseGeneratorIq_Reset(&state->if_phase, MC_ZERO);
    PidIq_Reset(&state->speed_pid);
    PidIq_Reset(&state->current_d_pid);
    PidIq_Reset(&state->current_q_pid);
    SpeedPidScheduleIq_Reset(&state->speed_schedule);
    LesoIq_Reset(&state->leso, state->control_angle_pu);
    HfiIq_Reset(&state->hfi, state->control_angle_pu);
    FlyingIq_Reset(&state->flying);
    SensorlessIq_Reset(&state->sensorless, state->control_angle_pu);
}

void FocIq_Reset(FocIqState_t* state, mc_real_t initial_angle_pu)
{
    if (state == NULL)
        return;
    foc_iq_reset_control(state, initial_angle_pu);
    IdentificationIq_Reset(&state->identification);
}

void FocIq_Set_Mode(FocIqState_t* state, FocMode_t mode)
{
    if (state != NULL)
        state->mode = foc_iq_mode_is_valid(mode) ? mode : IDLE;
}

FocMode_t FocIq_Get_Mode(const FocIqState_t* state)
{
    return state == NULL ? IDLE : state->mode;
}

void FocIq_Request_StartupPrepare(FocIqState_t* state)
{
    if (state != NULL)
        state->startup_request_latched = true;
}

void FocIq_Set_Angle(FocIqState_t* state, mc_real_t angle_pu)
{
    if (state != NULL)
        state->control_angle_pu = AngleIq_WrapPu(angle_pu);
}

void FocIq_Set_Speed(FocIqState_t* state, mc_real_t speed_pu)
{
    if (state != NULL)
        state->speed_feedback_pu = speed_pu;
}

ParkIq_t FocIq_Get_Inductor(const FocIqState_t* state)
{
    return state == NULL ? (ParkIq_t){0} : state->inductance_dq_pu;
}

static void foc_iq_handle_mode_change(FocIqState_t* state,
                                      FocMode_t requested_mode,
                                      mc_real_t measured_angle_pu)
{
    FocMode_t mode = foc_iq_mode_is_valid(requested_mode)
                   ? requested_mode : IDLE;
    if (mode == state->previous_mode)
    {
        state->mode = mode;
        return;
    }
    state->mode = mode;
    state->previous_mode = mode;
    /* Legacy Foc_Set_Mode writes the enum only: mode switching is not a
     * controller reset or a request to repeat SPEED startup preparation. */
    (void)measured_angle_pu;
    if (mode == IDENTIFY)
    {
        IdentificationIq_Reset(&state->identification);
        if (!state->identification.config.manual_control)
            IdentificationIq_Start(&state->identification);
    }
}

static void foc_iq_handle_reset_release(FocIqState_t* state)
{
    if (state->mode == IDENTIFY
             && state->identification.state == IDENTIFICATION_IQ_WAIT
             && !state->identification.config.manual_control)
        IdentificationIq_Start(&state->identification);
}

bool FocIq_BackgroundBuildMtpa(FocIqState_t* state)
{
    if (state == NULL || !state->mtpa_rebuild_pending
        || state->mtpa_rebuild_in_progress || state->mode != IDLE)
        return false;
    state->mtpa_rebuild_in_progress = true;
    MtpaIq_Init(&state->mtpa_staging, &state->pending_mtpa_parameters);
    bool valid = MtpaIq_BuildTable(&state->mtpa_staging, MC_ZERO, MC_ONE);
    state->mtpa_rebuild_pending = false;
    state->mtpa_staging_ready = valid;
    state->mtpa_rebuild_failed = !valid;
    if (!valid)
    {
        /* The new model was requested but cannot be used. Retain old table
         * bytes for diagnosis, never silently run them as the new model. */
        state->mtpa.table_valid = false;
        state->mtpa_rebuild_in_progress = false;
    }
    return valid;
}

void FocIq_CommitBackgroundMtpa(FocIqState_t* state)
{
    if (state == NULL || !state->mtpa_staging_ready)
        return;
    state->mtpa = state->mtpa_staging;
    state->mtpa_staging_ready = false;
    state->mtpa_rebuild_in_progress = false;
    state->mtpa_rebuild_failed = false;
}

static SensorlessIqInput_t foc_iq_update_position(
    FocIqState_t* state, const FocIqParameters_t* parameters,
    const FocIqInput_t* input, bool reset, SensorlessIqOutput_t* estimated)
{
    SensorlessIqInput_t snapshot = {
        .enabled = parameters->sensorless_enabled, .reset = reset,
        .requested_method = (SensorlessIqMethod_t)parameters->sensorless_method,
        .real_angle_pu = input->measured_angle_pu,
        .measured_speed_pu = input->measured_speed_pu,
        .speed_reference_pu = state->sensorless.speed_reference_pu,
        .speed_target_pu = state->sensorless.speed_target_pu,
        .speed_feedback_pu = state->sensorless.speed_feedback_pu,
        .current_ab_pu = state->current_ab_pu,
        /* Calculate refreshes this to this cycle's base FOC voltage below. */
        .voltage_ab_pu = state->voltage_reference_ab_pu};
    SensorlessIq_UpdatePosition(&state->sensorless, &state->config.sensorless,
        &snapshot, &state->leso, &state->hfi, estimated);
    state->using_hfi = estimated->using_hfi;
    state->control_angle_pu = parameters->use_real_angle
        ? input->measured_angle_pu : estimated->angle_pu;
    state->speed_feedback_pu = parameters->use_real_angle
        ? input->measured_speed_pu : estimated->speed_pu;
    /* Match the old MainInt setters after Update_Position and before FOC. */
    snapshot.speed_reference_pu = state->sensorless.speed_reference_pu
        = state->speed_ramp.value;
    snapshot.speed_target_pu = state->sensorless.speed_target_pu
        = parameters->speed_reference_pu;
    snapshot.speed_feedback_pu = state->sensorless.speed_feedback_pu
        = state->speed_feedback_pu;
    return snapshot;
}

static mc_real_t foc_iq_pid_update(PidIqState_t* pid, mc_real_t error,
                                   bool reset)
{
    /* Match the active Pid_Update in pid.h: the anti-windup decision uses
     * P+I before the derivative, and previous_error changes only with Kd. */
    if (reset)
    {
        PidIq_Reset(pid);
        return MC_ZERO;
    }
    mc_real_t proportional = McMath_Mul(pid->kp, error);
    mc_real_t unclamped = McMath_Add(proportional, pid->integral);
    if (unclamped >= pid->minimum && unclamped <= pid->maximum
        && pid->ki_step != MC_ZERO)
        pid->integral = McMath_Clamp(
            McMath_Add(pid->integral, McMath_Mul(pid->ki_step, error)),
            McMath_Neg(pid->integral_limit), pid->integral_limit);
    mc_real_t derivative = MC_ZERO;
    if (pid->kd_step != MC_ZERO)
    {
        derivative = McMath_Mul(
            pid->kd_step, McMath_Sub(error, pid->last_error));
        pid->last_error = error;
    }
    pid->output = McMath_Clamp(
        McMath_Add(McMath_Add(proportional, pid->integral), derivative),
        pid->minimum, pid->maximum);
    return pid->output;
}

static mc_real_t foc_iq_update_phase(FocIqState_t* state,
                                     PhaseGeneratorIqState_t* phase,
                                     bool* reset_previous,
                                     mc_real_t frequency_step_pu,
                                     mc_real_t offset_pu,
                                     bool sweep_request, bool reset)
{
    /* SawtoothWaveGenerator resets its zero-based counter then still adds
     * one step. VF and IF share that counter; held/sweep memories are local. */
    if (reset || (*reset_previous && !reset))
        state->shared_phase_pu = MC_ZERO;
    mc_real_t value = McMath_Add(
        state->shared_phase_pu, frequency_step_pu);
    if (frequency_step_pu >= MC_ZERO)
    {
        if (value > MC_ONE)
            value = McMath_Sub(value, MC_ONE);
        else if (value < MC_ZERO)
            value = MC_ZERO;
    }
    else
    {
        if (value < MC_ZERO)
            value = McMath_Add(value, MC_ONE);
        else if (value > MC_ONE)
            value = MC_ONE;
    }
    state->shared_phase_pu = value;
    phase->phase_pu = value;
    /* Original freeze checks the generated phase against the held phase,
     * not against the previous free-running counter, at pi/2 = 0.25 pu. */
    if (McMath_Abs(McMath_Sub(value, phase->held_phase_pu)) > MC_CONST(0.25))
        phase->sweep_active = false;
    if (sweep_request)
        phase->sweep_active = true;
    if (phase->sweep_active)
        phase->held_phase_pu = value;
    *reset_previous = reset;
    return AngleIq_WrapPu(McMath_Add(phase->held_phase_pu, offset_pu));
}

static ParkIq_t foc_iq_run_current_loop(FocIqState_t* state,
                                        ParkIq_t reference, bool reset)
{
    return (ParkIq_t){
        foc_iq_pid_update(&state->current_d_pid,
            McMath_Sub(reference.d, state->current_dq_pu.d), reset),
        foc_iq_pid_update(&state->current_q_pid,
            McMath_Sub(reference.q, state->current_dq_pu.q), reset)};
}

static ParkIq_t foc_iq_mtpa_fallback_inductance(const FocIqConfig_t* config)
{
    ParkIq_t fallback = config->mtpa_fallback_inductance_dq_pu;
    /* Invalid MTPA results must not inject zero L into the observer. These
     * are nominal motor parameters, not a claim that identification passed. */
    if (fallback.d <= MC_ZERO)
        fallback.d = MC_CONST(MOTOR_LD * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB);
    if (fallback.q <= MC_ZERO)
        fallback.q = MC_CONST(MOTOR_LQ * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB);
#if !defined(MC_NUMERIC_IQMATH)
    if (!isfinite(fallback.d))
        fallback.d = MC_CONST(MOTOR_LD * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB);
    if (!isfinite(fallback.q))
        fallback.q = MC_CONST(MOTOR_LQ * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB);
#endif
    return fallback;
}

static ParkIq_t foc_iq_run_speed_mode(FocIqState_t* state,
                                      const FocIqParameters_t* parameters,
                                      bool reset)
{
    if (parameters->startup_prepare_request)
        state->startup_request_latched = true;
    if (state->startup_request_latched)
    {
        state->startup_request_latched = false;
        state->startup_active = true;
        state->startup_count = 0U;
    }
    if (state->speed_reset_previous && !reset)
    {
        state->startup_active = true;
        state->startup_count = 0U;
    }
    if (reset)
    {
        state->startup_active = false;
        state->startup_count = 0U;
        state->startup_request_latched = false;
    }
    if (state->startup_active)
    {
        state->mtpa_linear_fallback = false;
        /* The legacy preparation bypasses current PI and resets only the
         * speed scheduler, speed PI/ramp/counter and current reference. */
        SpeedPidScheduleIq_Reset(&state->speed_schedule);
        state->speed_pid.kp = state->config.speed_schedule.kp_base;
        state->speed_pid.ki_step = state->config.speed_schedule.ki_step_base;
        PidIq_Reset(&state->speed_pid);
        RampIq_Reset(&state->speed_ramp, MC_ZERO);
        state->speed_counter = 0U;
        state->current_reference_dq_pu = (ParkIq_t){0};
        state->startup_count++;
        if (state->startup_count >= state->config.startup_hold_cycles)
            state->startup_active = false;
        state->speed_reset_previous = reset;
        return (ParkIq_t){state->config.startup_voltage_d_pu, MC_ZERO};
    }

    if (reset)
    {
        SpeedPidScheduleIq_Reset(&state->speed_schedule);
        state->mtpa_linear_fallback = false;
        state->speed_pid.kp = state->config.speed_schedule.kp_base;
        state->speed_pid.ki_step = state->config.speed_schedule.ki_step_base;
        PidIq_Reset(&state->speed_pid);
        RampIq_Reset(&state->speed_ramp, MC_ZERO);
        state->speed_counter = 0U;
        state->current_reference_dq_pu = (ParkIq_t){0};
    }
    else
    {
        /* speed_pi_update_gain ran every current-loop cycle, before the
         * possibly prescaled ramp update. Confirm counts are current ticks. */
        SpeedPidScheduleIqConfig_t schedule = state->config.speed_schedule;
        uint32_t confirm = (uint32_t)schedule.ki_confirm_cycles
                         * state->config.speed_prescaler;
        if (confirm == 0U)
            confirm = 1U;
        if (confirm > UINT16_MAX)
            confirm = UINT16_MAX;
        schedule.ki_confirm_cycles = (uint16_t)confirm;
        SpeedPidScheduleIq_Run(&state->speed_schedule,
            &schedule, parameters->speed_reference_pu,
            McMath_Abs(McMath_Sub(state->speed_ramp.value,
                                 state->speed_feedback_pu)),
            &state->speed_pid.kp,
            &state->speed_pid.ki_step);
        state->speed_counter++;
        if (state->speed_counter >= state->config.speed_prescaler)
        {
            state->speed_counter = 0U;
            mc_real_t ramp = RampIq_Run(&state->speed_ramp,
                parameters->speed_reference_pu,
                state->config.speed_ramp_step_pu);
            state->speed_ramp.value = McMath_Clamp(
                ramp, state->config.speed_ramp_minimum_pu,
                state->config.speed_ramp_maximum_pu);
            mc_real_t error = McMath_Sub(state->speed_ramp.value,
                                         state->speed_feedback_pu);
            state->current_reference_dq_pu.q = foc_iq_pid_update(
                &state->speed_pid, error, false);
            MtpaIqLookupResult_t mtpa = MtpaIq_InterpolateByIq(
                &state->mtpa, state->current_reference_dq_pu.q);
            state->mtpa_linear_fallback = !mtpa.valid;
            if (mtpa.valid)
            {
                state->current_reference_dq_pu.d = mtpa.id_pu;
                state->inductance_dq_pu = (ParkIq_t){mtpa.ld_pu, mtpa.lq_pu};
            }
            else
            {
                /* Preserve the signed, already PI-limited Iq. No additional
                 * vector-current limiter existed in the original dispatch. */
                state->current_reference_dq_pu.d = McMath_Abs(
                    state->current_reference_dq_pu.q);
                state->inductance_dq_pu = foc_iq_mtpa_fallback_inductance(
                    &state->config);
            }
        }
    }
    ParkIq_t voltage = foc_iq_run_current_loop(
        state, state->current_reference_dq_pu, reset);
    voltage.q = McMath_Clamp(voltage.q,
        McMath_Neg(state->config.speed_q_voltage_limit_pu),
        state->config.speed_q_voltage_limit_pu);
    state->speed_reset_previous = reset;
    return voltage;
}

void FocIq_Run(FocIqState_t* state,
               const FocIqParameters_t* parameters,
               const FocIqInput_t* input,
               FocIqOutput_t* output)
{
    if (state == NULL || parameters == NULL || input == NULL || output == NULL)
        return;
    if (!state->initialized)
    {
        McMath_Diagnostics.invalid_input_count++;
        *output = (FocIqOutput_t){.pwm_duty = {
            MC_HALF, MC_HALF, MC_HALF}, .requested_mode = IDLE};
        return;
    }
    foc_iq_apply_pid_parameters(state, parameters);
    state->identification.config = parameters->identification_config;
    FocMode_t requested_mode = state->mtpa_rebuild_in_progress
                             ? IDLE : parameters->mode;
    bool mode_changed = (foc_iq_mode_is_valid(requested_mode)
                       ? requested_mode : IDLE) != state->previous_mode;
    foc_iq_handle_mode_change(state, requested_mode,
                              input->measured_angle_pu);
    state->current_ab_pu = TransformIq_Clarke(input->current_abc_pu);
    bool reset = parameters->reset || state->mode == IDLE;
    if (state->mode != SPEED || reset)
        state->mtpa_linear_fallback = false;
    if (reset)
    {
        /* The legacy active mode controls its own Reset semantics. Do not
         * reset dormant PI/phase memories merely for IDLE or a mode change.
         * The large experiment resets only on entry, never every 5kHz tick. */
        if ((!state->reset_previous || mode_changed)
            && !(mode_changed && state->mode == IDENTIFY))
            IdentificationIq_Reset(&state->identification);
        state->identification.start_requested = false;
    }
    else if (state->reset_previous)
        foc_iq_handle_reset_release(state);
    state->reset_previous = reset;

    /* Same order as legacy MainInt: Clarke / optional HFI current extraction,
     * previous-angle Park, common PLL position, mode FOC, observer, injection.
     * VF/IF/SPEED/IDENTIFY re-Park below with the updated control angle. */
    state->current_ab_pu = SensorlessIq_FilterCurrent(
        &state->sensorless, &state->hfi, state->current_ab_pu,
        parameters->sensorless_enabled, reset);
    state->current_dq_pu = TransformIq_Park(
        state->current_ab_pu, state->control_angle_pu);
    SensorlessIqOutput_t estimated = {0};
    SensorlessIqInput_t sensorless_input = foc_iq_update_position(
        state, parameters, input, reset, &estimated);

    if (state->mode == VF_MODE)
        state->control_angle_pu = foc_iq_update_phase(
            state, &state->vf_phase, &state->vf_reset_previous,
            parameters->vf_frequency_step_pu, parameters->vf_offset_pu,
            parameters->sweep_request, reset);
    else if (state->mode == IF_MODE && !parameters->if_use_sensor)
        state->control_angle_pu = foc_iq_update_phase(
            state, &state->if_phase, &state->if_reset_previous,
            parameters->if_frequency_step_pu, parameters->if_offset_pu,
            parameters->sweep_request, reset);
    else if (state->mode == IF_MODE)
        state->if_reset_previous = reset;
    if (state->mode == VF_MODE || state->mode == IF_MODE
        || state->mode == SPEED || state->mode == IDENTIFY)
        state->current_dq_pu = TransformIq_Park(
            state->current_ab_pu, state->control_angle_pu);

    ParkIq_t voltage = {0};
    switch (state->mode)
    {
    case VF_MODE:
        voltage = parameters->vf_voltage_reference_pu;
        break;
    case IF_MODE:
        state->current_reference_dq_pu = parameters->if_current_reference_pu;
        voltage = foc_iq_run_current_loop(
            state, state->current_reference_dq_pu, reset);
        break;
    case STARTUP:
        voltage.d = state->config.startup_voltage_d_pu;
        break;
    case SPEED:
        voltage = foc_iq_run_speed_mode(state, parameters, reset);
        break;
    case IDENTIFY:
    {
        /* Reset entry already invalidated the experiment. Keep injection
         * stopped without repeatedly resetting its metadata. */
        if (reset)
            break;
        IdentificationIqInput_t identification_input = {
            .current_d_pu = state->current_dq_pu.d,
            .current_q_pu = state->current_dq_pu.q,
            .start_rs = parameters->identification_start_rs,
            .start_axis = parameters->identification_start_axis,
            .injection_enable = parameters->identification_injection_enable,
            .reset = reset};
        IdentificationIqOutput_t identification_output = {0};
        IdentificationIq_Run(&state->identification,
            &identification_input, &identification_output);
        voltage.d = identification_output.voltage_d_pu;
        voltage.q = identification_output.voltage_q_pu;
        if (identification_output.complete && identification_output.valid
            && !state->mtpa_rebuild_pending
            && !state->mtpa_rebuild_in_progress)
        {
            if (IdentificationIq_GetMtpaParameters(
                    &state->identification,
                    &state->pending_mtpa_parameters))
            {
                state->mtpa_rebuild_pending = true;
            }
        }
        break;
    }
    case IDLE:
    default:
        break;
    }

    ParkIq_t inductance = FocIq_Get_Inductor(state);
    sensorless_input.inductance_q_pu = inductance.q;
    /* Legacy Foc_Update_Main already publishes the current non-HFI voltage
     * before MainInt_Update_Sensorless; do not model last cycle's injection. */
    sensorless_input.voltage_ab_pu = TransformIq_InversePark(
        voltage, state->control_angle_pu);
    SensorlessIq_Calculate(&state->sensorless, &state->config.sensorless,
        &sensorless_input, &state->config.leso, &state->config.hfi,
        &state->leso, &state->hfi, &state->flying);
    voltage = SensorlessIq_InjectVoltage(&state->sensorless,
        &state->config.hfi, &state->hfi, voltage, reset);
    state->voltage_reference_dq_pu = voltage;
    state->voltage_reference_ab_pu = TransformIq_InversePark(
        voltage, state->control_angle_pu);
    PhaseIq_t pwm = reset ? foc_iq_neutral_pwm()
                          : foc_iq_svpwm(state->voltage_reference_ab_pu,
                                        input->bus_voltage_pu);
    *output = (FocIqOutput_t){
        .pwm_duty = pwm,
        .current_ab_pu = state->current_ab_pu,
        .current_dq_pu = state->current_dq_pu,
        .current_reference_dq_pu = state->current_reference_dq_pu,
        .voltage_reference_dq_pu = state->voltage_reference_dq_pu,
        .voltage_reference_ab_pu = state->voltage_reference_ab_pu,
        .inductance_dq_pu = inductance,
        .control_angle_pu = state->control_angle_pu,
        .speed_ramp_pu = state->speed_ramp.value,
        .speed_feedback_pu = state->speed_feedback_pu,
        .estimated_angle_pu = estimated.angle_pu,
        .estimated_speed_pu = estimated.speed_pu,
        .requested_mode = (state->mode == IDENTIFY
                         && (state->identification.state == IDENTIFICATION_IQ_DONE
                         || state->identification.state == IDENTIFICATION_IQ_FAILED))
                        ? IDLE : state->mode,
        .identification_state = state->identification.state,
        .identification_error = state->identification.error,
        .startup_active = state->startup_active,
        .using_hfi = state->using_hfi};
}
