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
    if (bus_voltage_pu <= MC_CONST(0.025))
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
        .flying_startup_delay = 100U,
        .speed_ramp_step_pu = MC_CONST(
            RAMP_SPEED_SLOPE * SPEED_LOOP_TIME / MC_SPEED_BASE_RPM),
        .sensorless_switch_speed_pu = MC_CONST(
            SENSORLESS_SWITCH_SPEED / MC_SPEED_BASE_RPM),
        .sensorless_hysteresis_pu = MC_CONST(
            SENSORLESS_HYSTERESIS / MC_SPEED_BASE_RPM),
        .startup_voltage_d_pu = MC_CONST(
            SENSORLESS_STARTUP_UD / MC_VOLTAGE_BASE_V)};
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
    state->initialized = true;
}

void FocIq_Reset(FocIqState_t* state, mc_real_t initial_angle_pu)
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
    PhaseGeneratorIq_Reset(&state->vf_phase, state->control_angle_pu);
    PhaseGeneratorIq_Reset(&state->if_phase, state->control_angle_pu);
    PidIq_Reset(&state->speed_pid);
    PidIq_Reset(&state->current_d_pid);
    PidIq_Reset(&state->current_q_pid);
    SpeedPidScheduleIq_Reset(&state->speed_schedule);
    IdentificationIq_Reset(&state->identification);
    LesoIq_Reset(&state->leso, state->control_angle_pu);
    HfiIq_Reset(&state->hfi, state->control_angle_pu);
    FlyingIq_Reset(&state->flying);
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
    ParkIq_t result = {0};
    if (state == NULL || !state->mtpa.table_valid)
        return result;
    mc_real_t iq = McMath_Abs(state->current_reference_dq_pu.q);
    uint16_t nearest = 0U;
    mc_real_t distance = INT32_MAX;
    for (uint16_t index = 0U; index < MTPA_IQ_TABLE_POINTS; ++index)
    {
        if (!state->mtpa.table[index].valid)
            continue;
        mc_real_t current_distance = McMath_Abs(McMath_Sub(
            state->mtpa.table[index].iq_pu, iq));
        if (current_distance < distance)
        {
            distance = current_distance;
            nearest = index;
        }
    }
    result.d = state->mtpa.table[nearest].ld_pu;
    result.q = state->mtpa.table[nearest].lq_pu;
    return result;
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
    state->control_angle_pu = measured_angle_pu;
    state->speed_counter = 0U;
    state->current_reference_dq_pu = (ParkIq_t){0};
    PidIq_Reset(&state->speed_pid);
    PidIq_Reset(&state->current_d_pid);
    PidIq_Reset(&state->current_q_pid);
    SpeedPidScheduleIq_Reset(&state->speed_schedule);
    PhaseGeneratorIq_Reset(&state->vf_phase, measured_angle_pu);
    PhaseGeneratorIq_Reset(&state->if_phase, measured_angle_pu);
    if (mode == SPEED)
    {
        state->startup_active = true;
        state->startup_count = 0U;
    }
    if (mode == IDENTIFY)
    {
        IdentificationIq_Reset(&state->identification);
        IdentificationIq_Start(&state->identification);
    }
}

static void foc_iq_handle_reset_release(FocIqState_t* state)
{
    if (state->mode == SPEED)
    {
        state->startup_active = true;
        state->startup_count = 0U;
    }
    else if (state->mode == IDENTIFY
             && state->identification.state == IDENTIFICATION_IQ_WAIT)
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
        state->mtpa_rebuild_in_progress = false;
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

static void foc_iq_run_sensorless(FocIqState_t* state,
                                  const FocIqParameters_t* parameters,
                                  const FocIqInput_t* input,
                                  bool reset,
                                  LesoIqOutput_t* leso_output,
                                  HfiIqOutput_t* hfi_output)
{
    bool leso_requested =
        (parameters->sensorless_method & FOC_IQ_SENSORLESS_LESO) != 0U;
    bool hfi_requested =
        (parameters->sensorless_method & FOC_IQ_SENSORLESS_HFI) != 0U;
    if (leso_requested != state->leso.enabled)
    {
        state->leso.enabled = leso_requested;
        LesoIq_Reset(&state->leso, input->measured_angle_pu);
    }
    if (hfi_requested != state->hfi.enabled)
    {
        state->hfi.enabled = hfi_requested;
        HfiIq_Reset(&state->hfi, input->measured_angle_pu);
    }
    FlyingIq_SetEnabled(&state->flying,
        (parameters->sensorless_method & FOC_IQ_SENSORLESS_FLYING) != 0U);
    FlyingIq_Run(&state->flying, reset);
    LesoIqInput_t leso_input = {
        .voltage_ab_pu = state->voltage_reference_ab_pu,
        .current_ab_pu = state->current_ab_pu,
        .reset = reset};
    LesoIq_Run(&state->leso, &state->config.leso,
               &leso_input, leso_output);
    HfiIqInput_t hfi_input = {
        .current_ab_pu = state->current_ab_pu,
        .reset = reset};
    HfiIq_Run(&state->hfi, &state->config.hfi,
              &hfi_input, hfi_output);

    bool hfi_available = state->hfi.enabled;
    bool leso_available = state->leso.enabled;
    mc_real_t speed_abs = McMath_Abs(leso_output->speed_pu);
    mc_real_t lower = McMath_Sub(state->config.sensorless_switch_speed_pu,
                                 state->config.sensorless_hysteresis_pu);
    mc_real_t upper = McMath_Add(state->config.sensorless_switch_speed_pu,
                                 state->config.sensorless_hysteresis_pu);
    if (hfi_available && leso_available)
    {
        if (state->using_hfi && speed_abs >= upper)
            state->using_hfi = false;
        else if (!state->using_hfi && speed_abs <= lower)
            state->using_hfi = true;
    }
    else
        state->using_hfi = hfi_available;
    if (!parameters->use_real_angle && (hfi_available || leso_available))
    {
        state->control_angle_pu = state->using_hfi
                                ? hfi_output->angle_pu
                                : leso_output->angle_pu;
        state->speed_feedback_pu = state->using_hfi
                                 ? hfi_output->speed_pu
                                 : leso_output->speed_pu;
    }
    else
    {
        state->control_angle_pu = input->measured_angle_pu;
        state->speed_feedback_pu = input->measured_speed_pu;
    }
}

static ParkIq_t foc_iq_run_current_loop(FocIqState_t* state,
                                        ParkIq_t reference)
{
    return (ParkIq_t){
        PidIq_Run(&state->current_d_pid,
            McMath_Sub(reference.d, state->current_dq_pu.d)),
        PidIq_Run(&state->current_q_pid,
            McMath_Sub(reference.q, state->current_dq_pu.q))};
}

static ParkIq_t foc_iq_run_speed_mode(FocIqState_t* state,
                                      const FocIqParameters_t* parameters)
{
    if (parameters->startup_prepare_request)
        state->startup_request_latched = true;
    if (state->startup_request_latched)
    {
        state->startup_request_latched = false;
        state->startup_active = true;
        state->startup_count = 0U;
    }
    if (state->startup_active)
    {
        PidIq_Reset(&state->speed_pid);
        PidIq_Reset(&state->current_d_pid);
        PidIq_Reset(&state->current_q_pid);
        state->current_reference_dq_pu = (ParkIq_t){0};
        state->startup_count++;
        if (state->startup_count >= state->config.startup_hold_cycles)
            state->startup_active = false;
        return (ParkIq_t){state->config.startup_voltage_d_pu, MC_ZERO};
    }

    state->speed_counter++;
    if (state->speed_counter >= state->config.speed_prescaler)
    {
        state->speed_counter = 0U;
        mc_real_t ramp = RampIq_Run(&state->speed_ramp,
            parameters->speed_reference_pu, state->config.speed_ramp_step_pu);
        mc_real_t error = McMath_Sub(ramp, state->speed_feedback_pu);
        SpeedPidScheduleIq_Run(&state->speed_schedule,
            &state->config.speed_schedule, parameters->speed_reference_pu,
            McMath_Abs(error), &state->speed_pid.kp,
            &state->speed_pid.ki_step);
        state->current_reference_dq_pu.q = PidIq_Run(
            &state->speed_pid, error);
        state->current_reference_dq_pu.d = MtpaIq_InterpolateIdByIq(
            &state->mtpa, state->current_reference_dq_pu.q);
    }
    return foc_iq_run_current_loop(state, state->current_reference_dq_pu);
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
    FocMode_t requested_mode = state->mtpa_rebuild_in_progress
                             ? IDLE : parameters->mode;
    foc_iq_handle_mode_change(state, requested_mode,
                              input->measured_angle_pu);
    state->current_ab_pu = TransformIq_Clarke(input->current_abc_pu);
    bool reset = parameters->reset || state->mode == IDLE;
    if (reset)
        FocIq_Reset(state, input->measured_angle_pu);
    else if (state->reset_previous)
        foc_iq_handle_reset_release(state);
    state->reset_previous = reset;

    LesoIqOutput_t leso_output = {0};
    HfiIqOutput_t hfi_output = {0};
    foc_iq_run_sensorless(state, parameters, input, reset,
                          &leso_output, &hfi_output);

    if (state->mode == VF_MODE)
        state->control_angle_pu = PhaseGeneratorIq_Run(&state->vf_phase,
            parameters->vf_frequency_step_pu, parameters->vf_offset_pu,
            parameters->sweep_request);
    else if (state->mode == IF_MODE && !parameters->if_use_sensor)
        state->control_angle_pu = PhaseGeneratorIq_Run(&state->if_phase,
            parameters->if_frequency_step_pu, parameters->if_offset_pu,
            parameters->sweep_request);
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
            state, state->current_reference_dq_pu);
        break;
    case STARTUP:
        voltage.d = state->config.startup_voltage_d_pu;
        break;
    case SPEED:
        voltage = foc_iq_run_speed_mode(state, parameters);
        break;
    case IDENTIFY:
    {
        IdentificationIqInput_t identification_input = {
            .current_d_pu = state->current_dq_pu.d,
            .current_q_pu = state->current_dq_pu.q,
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

    if (state->hfi.enabled && !reset)
    {
        voltage.d = McMath_Add(voltage.d,
            hfi_output.injection_voltage_dq_pu.d);
        voltage.q = McMath_Add(voltage.q,
            hfi_output.injection_voltage_dq_pu.q);
    }
    state->voltage_reference_dq_pu = voltage;
    state->voltage_reference_ab_pu = TransformIq_InversePark(
        voltage, state->control_angle_pu);
    PhaseIq_t pwm = reset ? foc_iq_neutral_pwm()
                          : foc_iq_svpwm(state->voltage_reference_ab_pu,
                                        input->bus_voltage_pu);
    ParkIq_t inductance = FocIq_Get_Inductor(state);
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
        .estimated_angle_pu = state->using_hfi
                            ? hfi_output.angle_pu : leso_output.angle_pu,
        .estimated_speed_pu = state->using_hfi
                            ? hfi_output.speed_pu : leso_output.speed_pu,
        .requested_mode = state->identification.state == IDENTIFICATION_IQ_DONE
                        ? IDLE : state->mode,
        .identification_state = state->identification.state,
        .identification_error = state->identification.error,
        .startup_active = state->startup_active,
        .using_hfi = state->using_hfi};
}
