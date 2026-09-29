#include "fixed_control.h"

#include <stddef.h>
#include "fixed_numeric_config.h"
#include "identification.h"
#include "parameters.h"

/* These floating-point objects are CAN/CCP/A2L compatibility registers only.
 * They are sampled into a fixed-point snapshot at the ISR boundary and are
 * never read by the algorithm core. */
typedef struct
{
    volatile float Kp;
    volatile float Ki;
    volatile float Kd;
    volatile float integral;
    volatile float previous_error;
    volatile float MaxOutput;
    volatile float MinOutput;
    volatile float output;
    volatile float IntegralLimit;
    volatile float Ts;
    volatile bool Reset;
} A2lPidMirror_t;

typedef struct
{
    volatile Park_t vol_ref;
    volatile float freq;
    volatile float offset;
} A2lVfMirror_t;

typedef struct
{
    volatile Park_t cur_ref;
    volatile float freq;
    volatile float offset;
    volatile bool use_sensor;
} A2lIfMirror_t;

volatile FocMode_t Foc_Mode = IDLE;
volatile bool Foc_Reset = true;
volatile bool Foc_Sweep = true;
volatile bool Foc_StartupPrepareRequest = false;
volatile float Foc_Speed_Ref = 0.0F;
volatile float Foc_Speed_Ramp = 0.0F;
volatile float Foc_Speed_Fdbk = 0.0F;
volatile float Foc_Theta = 0.0F;
volatile float Foc_BusVoltage = 0.0F;
volatile float Foc_BusVoltage_Inv = 0.0F;
volatile float Foc_Id_Ref = 0.0F;
volatile float Foc_Iq_Ref = 0.0F;
volatile float Foc_Id_Fdbk = 0.0F;
volatile float Foc_Iq_Fdbk = 0.0F;
volatile float Foc_Ud_Ref = 0.0F;
volatile float Foc_Uq_Ref = 0.0F;
volatile float Foc_Current_Ts = MAIN_LOOP_TIME;
volatile float Foc_Current_Freq = MAIN_LOOP_FREQ;
volatile float Foc_Speed_Ts = SPEED_LOOP_TIME;
volatile float Foc_Speed_Freq = SPEED_LOOP_FREQ;
volatile bool MainInt_UseRealTheta = true;
volatile uint16_t Sensorless_Method = 0U;
volatile uint32_t FixedControl_CycleCount = 0U;
volatile FixedControlSelfTest_t FixedControl_SelfTest = {0};
volatile Park_t Foc_Idq_Ref = {0};
volatile Park_t Foc_Idq_Fdbk = {0};
volatile Park_t Foc_Udq_Ref = {0};
volatile Park_t Foc_Inductor = {0};
volatile Clark_t Foc_Iclark_Fdbk = {0};
volatile Clark_t Foc_Uclark_Ref = {0};
volatile Phase_t Foc_Tcm = {0.5F, 0.5F, 0.5F};
volatile A2lPidMirror_t Foc_Pid_Speed_Handler = {
    .Kp = PID_SPEED_LOOP_KP, .Ki = PID_SPEED_LOOP_KI,
    .Kd = PID_SPEED_LOOP_KD, .MaxOutput = PID_SPEED_LOOP_MAX_OUTPUT,
    .MinOutput = PID_SPEED_LOOP_MIN_OUTPUT,
    .IntegralLimit = PID_SPEED_LOOP_INTEGRAL_LIMIT,
    .Ts = SPEED_LOOP_TIME, .Reset = true};
volatile A2lPidMirror_t Foc_Pid_CurD_Handler = {
    .Kp = PID_CURRENT_D_LOOP_KP, .Ki = PID_CURRENT_D_LOOP_KI,
    .Kd = PID_CURRENT_D_LOOP_KD, .MaxOutput = PID_CURRENT_D_LOOP_MAX_OUTPUT,
    .MinOutput = PID_CURRENT_D_LOOP_MIN_OUTPUT,
    .IntegralLimit = PID_CURRENT_D_LOOP_INTEGRAL_LIMIT,
    .Ts = MAIN_LOOP_TIME, .Reset = true};
volatile A2lPidMirror_t Foc_Pid_CurQ_Handler = {
    .Kp = PID_CURRENT_Q_LOOP_KP, .Ki = PID_CURRENT_Q_LOOP_KI,
    .Kd = PID_CURRENT_Q_LOOP_KD, .MaxOutput = PID_CURRENT_Q_LOOP_MAX_OUTPUT,
    .MinOutput = PID_CURRENT_Q_LOOP_MIN_OUTPUT,
    .IntegralLimit = PID_CURRENT_Q_LOOP_INTEGRAL_LIMIT,
    .Ts = MAIN_LOOP_TIME, .Reset = true};
volatile A2lVfMirror_t Foc_VfParam = {0};
volatile A2lIfMirror_t Foc_IfParam = {0};
/* Legacy flat aliases remain searchable; the structured fields are writable
 * inputs and these aliases report the effective values. */
volatile float Foc_Vf_Vd = 0.0F;
volatile float Foc_Vf_Vq = 0.0F;
volatile float Foc_Vf_Frequency = 0.0F;
volatile float Foc_Vf_Offset = 0.0F;
volatile float Foc_If_Id = 0.0F;
volatile float Foc_If_Iq = 0.0F;
volatile float Foc_If_Frequency = 0.0F;
volatile float Foc_If_Offset = 0.0F;
volatile bool Foc_If_UseSensor = false;
FluxExperiment_t Experiment = {0};
volatile float Identification_Voltage = 100.0F;
volatile float Identification_Rs = 0.0F;
volatile uint16_t Identification_State = IDENTIFICATION_IQ_WAIT;
volatile uint16_t Identification_Error = IDENTIFICATION_IQ_NO_ERROR;
volatile bool Foc_StartupActive = false;
volatile bool Sensorless_UsingHfi = false;
volatile bool Mtpa_TableValid = false;
volatile bool Mtpa_RebuildPending = false;
volatile bool Mtpa_RebuildFailed = false;

static mc_real_t gateway_from_physical(float value, float base)
{
    return base == 0.0F ? MC_ZERO : McMath_FromFloat(value / base);
}

static float gateway_to_physical(mc_real_t value, float base)
{
    return McMath_ToFloat(value) * base;
}

static FocMode_t gateway_validate_mode(FocMode_t mode)
{
    if ((uint32_t)mode > (uint32_t)IDENTIFY)
        return IDLE;
    return mode;
}

static uint16_t gateway_clamp_count(int value, uint16_t minimum,
                                    uint16_t maximum)
{
    if (value < (int)minimum)
        return minimum;
    if (value > (int)maximum)
        return maximum;
    return (uint16_t)value;
}

static ExpState_e gateway_identification_state(
    IdentificationIqState_e state)
{
    switch (state)
    {
    case IDENTIFICATION_IQ_EST_RS: return EST_RS;
    case IDENTIFICATION_IQ_PROCESS: return PROCESS;
    case IDENTIFICATION_IQ_INJECT_COLLECT: return INJECT_COLLECT;
    case IDENTIFICATION_IQ_NEXT_CURRENT: return NEXT_I;
    case IDENTIFICATION_IQ_LLS: return LLS;
    case IDENTIFICATION_IQ_PENDING: return PENDING;
    case IDENTIFICATION_IQ_DONE: return DONE;
    case IDENTIFICATION_IQ_FAILED:
    case IDENTIFICATION_IQ_WAIT:
    default: return WAIT;
    }
}

static float gateway_coefficient_to_physical(mc_real_t value,
                                             uint8_t flux_power)
{
    float scale = MC_CURRENT_BASE_A;
    for (uint8_t power = 0U; power < flux_power; ++power)
        scale /= MC_FLUX_BASE_WB;
    return McMath_ToFloat(value) * scale;
}

static float gateway_accum_to_physical(mc_accum_t value, float scale)
{
#if defined(MC_NUMERIC_IQMATH)
    return ((float)value / (float)(1UL << MC_Q_FRACTIONAL_BITS)) * scale;
#else
    return (float)value * scale;
#endif
}

static void gateway_initialize_identification_mirror(void)
{
    Experiment.Ts = MAIN_LOOP_TIME;
    Experiment.Rs_est = 0.0F;
    Experiment.sample_capacity = IDENTIFICATION_IQ_SAMPLE_CAPACITY;
    Experiment.max_steps = IDENTIFICATION_IQ_MAX_STEPS;
    Experiment.repeat_times = 3;
    Experiment.repeat_count = 0;
    Experiment.sum_max_psi = 0.0F;
    Experiment.sum_max_I = 0.0F;
    Experiment.wait_edges = 3;
    Experiment.start_I = 3;
    Experiment.final_I = 12;
    Experiment.step_dir = 1;
    Experiment.pos = 0;
    Experiment.edge_count = 0;
    Experiment.step_index = 0;
    Experiment.state = WAIT;
    Experiment.inj = (VoltageInjector_t){0};
    Experiment.inj.mode = INJECT_D;
    Experiment.inject_amp = 100.0F;
    Experiment.Complete = false;
    Experiment.Initialized = true;
    Experiment.LLS = (LLS_Result_t){0};
    for (uint16_t index = 0U; index < IDENTIFICATION_IQ_MAX_STEPS; ++index)
        Experiment.results[index] = (ImaxResult_t){0};
    Identification_Voltage = 100.0F;
    Identification_Rs = 0.0F;
    Identification_State = IDENTIFICATION_IQ_WAIT;
    Identification_Error = IDENTIFICATION_IQ_NO_ERROR;
}

bool FixedControl_ModeRequestsStop(void)
{
    /* Match FLOAT_REF's Foc_Get_ResetFlag(): IDLE must keep hardware Stop
     * asserted, not merely reset the algorithm and emit neutral duty. */
    return gateway_validate_mode(Foc_Mode) == IDLE;
}

static void gateway_read_parameters(FocIqParameters_t* parameters)
{
    FocIq_DefaultParameters(parameters);
    FocMode_t validated_mode = gateway_validate_mode(Foc_Mode);
    if (validated_mode != Foc_Mode)
        Foc_Mode = IDLE;
    parameters->mode = validated_mode;
    parameters->reset = Foc_Reset;
    parameters->sweep_request = Foc_Sweep;
    parameters->startup_prepare_request = Foc_StartupPrepareRequest;
    parameters->use_real_angle = MainInt_UseRealTheta;
    parameters->sensorless_method = Sensorless_Method;
    parameters->speed_reference_pu = gateway_from_physical(
        Foc_Speed_Ref, MC_SPEED_BASE_RPM);
    parameters->vf_voltage_reference_pu = (ParkIq_t){
        gateway_from_physical(Foc_VfParam.vol_ref.d, MC_VOLTAGE_BASE_V),
        gateway_from_physical(Foc_VfParam.vol_ref.q, MC_VOLTAGE_BASE_V)};
    parameters->vf_frequency_step_pu = McMath_FromFloat(
        Foc_VfParam.freq * MAIN_LOOP_TIME);
    parameters->vf_offset_pu = McMath_FromFloat(
        Foc_VfParam.offset * MC_INV_TWO_PI_F);
    parameters->if_current_reference_pu = (ParkIq_t){
        gateway_from_physical(Foc_IfParam.cur_ref.d, MC_CURRENT_BASE_A),
        gateway_from_physical(Foc_IfParam.cur_ref.q, MC_CURRENT_BASE_A)};
    parameters->if_frequency_step_pu = McMath_FromFloat(
        Foc_IfParam.freq * MAIN_LOOP_TIME);
    parameters->if_offset_pu = McMath_FromFloat(
        Foc_IfParam.offset * MC_INV_TWO_PI_F);
    parameters->if_use_sensor = Foc_IfParam.use_sensor;
    Foc_Vf_Vd = Foc_VfParam.vol_ref.d;
    Foc_Vf_Vq = Foc_VfParam.vol_ref.q;
    Foc_Vf_Frequency = Foc_VfParam.freq;
    Foc_Vf_Offset = Foc_VfParam.offset;
    Foc_If_Id = Foc_IfParam.cur_ref.d;
    Foc_If_Iq = Foc_IfParam.cur_ref.q;
    Foc_If_Frequency = Foc_IfParam.freq;
    Foc_If_Offset = Foc_IfParam.offset;
    Foc_If_UseSensor = Foc_IfParam.use_sensor;

    parameters->speed_kp = McMath_FromFloat(
        Foc_Pid_Speed_Handler.Kp * MC_SPEED_BASE_RPM / MC_CURRENT_BASE_A);
    parameters->speed_ki_step = McMath_FromFloat(
        Foc_Pid_Speed_Handler.Ki * SPEED_LOOP_TIME
        * MC_SPEED_BASE_RPM / MC_CURRENT_BASE_A);
    parameters->speed_kd_step = McMath_FromFloat(
        Foc_Pid_Speed_Handler.Kd / SPEED_LOOP_TIME
        * MC_SPEED_BASE_RPM / MC_CURRENT_BASE_A);
    parameters->speed_minimum_pu = gateway_from_physical(
        Foc_Pid_Speed_Handler.MinOutput, MC_CURRENT_BASE_A);
    parameters->speed_maximum_pu = gateway_from_physical(
        Foc_Pid_Speed_Handler.MaxOutput, MC_CURRENT_BASE_A);
    parameters->speed_integral_limit_pu = gateway_from_physical(
        Foc_Pid_Speed_Handler.IntegralLimit, MC_CURRENT_BASE_A);

    parameters->current_d_kp = McMath_FromFloat(
        Foc_Pid_CurD_Handler.Kp * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V);
    parameters->current_d_ki_step = McMath_FromFloat(
        Foc_Pid_CurD_Handler.Ki * MAIN_LOOP_TIME
        * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V);
    parameters->current_d_kd_step = McMath_FromFloat(
        Foc_Pid_CurD_Handler.Kd / MAIN_LOOP_TIME
        * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V);
    parameters->current_d_minimum_pu = gateway_from_physical(
        Foc_Pid_CurD_Handler.MinOutput, MC_VOLTAGE_BASE_V);
    parameters->current_d_maximum_pu = gateway_from_physical(
        Foc_Pid_CurD_Handler.MaxOutput, MC_VOLTAGE_BASE_V);
    parameters->current_d_integral_limit_pu = gateway_from_physical(
        Foc_Pid_CurD_Handler.IntegralLimit, MC_VOLTAGE_BASE_V);

    parameters->current_q_kp = McMath_FromFloat(
        Foc_Pid_CurQ_Handler.Kp * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V);
    parameters->current_q_ki_step = McMath_FromFloat(
        Foc_Pid_CurQ_Handler.Ki * MAIN_LOOP_TIME
        * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V);
    parameters->current_q_kd_step = McMath_FromFloat(
        Foc_Pid_CurQ_Handler.Kd / MAIN_LOOP_TIME
        * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V);
    parameters->current_q_minimum_pu = gateway_from_physical(
        Foc_Pid_CurQ_Handler.MinOutput, MC_VOLTAGE_BASE_V);
    parameters->current_q_maximum_pu = gateway_from_physical(
        Foc_Pid_CurQ_Handler.MaxOutput, MC_VOLTAGE_BASE_V);
    parameters->current_q_integral_limit_pu = gateway_from_physical(
        Foc_Pid_CurQ_Handler.IntegralLimit, MC_VOLTAGE_BASE_V);

    IdentificationIqConfig_t* identification =
        &parameters->identification_config;
    identification->manual_control = true;
    identification->sample_capacity = gateway_clamp_count(
        Experiment.sample_capacity, 2U, IDENTIFICATION_IQ_SAMPLE_CAPACITY);
    identification->repeat_times = gateway_clamp_count(
        Experiment.repeat_times, 1U, 3U);
    identification->max_steps = gateway_clamp_count(
        Experiment.max_steps, 1U, IDENTIFICATION_IQ_MAX_STEPS);
    identification->wait_edges = gateway_clamp_count(
        Experiment.wait_edges, 0U, 100U);
    Experiment.sample_capacity = (int)identification->sample_capacity;
    Experiment.repeat_times = (int)identification->repeat_times;
    Experiment.max_steps = (int)identification->max_steps;
    Experiment.wait_edges = (int)identification->wait_edges;
    identification->current_start_pu = gateway_from_physical(
        (float)Experiment.start_I, MC_CURRENT_BASE_A);
    identification->current_final_pu = gateway_from_physical(
        (float)Experiment.final_I, MC_CURRENT_BASE_A);
    identification->current_step_pu = gateway_from_physical(
        Experiment.step_dir < 0 ? -1.0F : 1.0F, MC_CURRENT_BASE_A);
    Experiment.step_dir = Experiment.step_dir < 0 ? -1 : 1;
    identification->injection_voltage_d_pu = gateway_from_physical(
        Experiment.inj.Ud_amp, MC_VOLTAGE_BASE_V);
    identification->injection_voltage_q_pu = gateway_from_physical(
        Experiment.inj.Uq_amp, MC_VOLTAGE_BASE_V);
    parameters->identification_start_rs = Experiment.state == EST_RS;
    parameters->identification_start_axis = Experiment.state == NEXT_I;
    parameters->identification_injection_enable = Experiment.inj.State;
    float ud_absolute = Experiment.inj.Ud_amp >= 0.0F
                      ? Experiment.inj.Ud_amp : -Experiment.inj.Ud_amp;
    float uq_absolute = Experiment.inj.Uq_amp >= 0.0F
                      ? Experiment.inj.Uq_amp : -Experiment.inj.Uq_amp;
    Identification_Voltage = ud_absolute > uq_absolute
                           ? ud_absolute : uq_absolute;
    Foc_StartupPrepareRequest = false;
}

static void gateway_write_telemetry(const FocIqState_t* state,
                                    const FocIqOutput_t* value,
                                    float bus_voltage)
{
    Foc_Mode = value->requested_mode;
    Foc_Speed_Ramp = gateway_to_physical(
        value->speed_ramp_pu, MC_SPEED_BASE_RPM);
    Foc_Speed_Fdbk = gateway_to_physical(
        value->speed_feedback_pu, MC_SPEED_BASE_RPM);
    Foc_Theta = gateway_to_physical(value->control_angle_pu, MC_TWO_PI_F);
    Foc_BusVoltage = bus_voltage;
    Foc_BusVoltage_Inv = bus_voltage > 0.0F ? 1.0F / bus_voltage : 0.0F;
    Foc_Id_Ref = gateway_to_physical(
        value->current_reference_dq_pu.d, MC_CURRENT_BASE_A);
    Foc_Iq_Ref = gateway_to_physical(
        value->current_reference_dq_pu.q, MC_CURRENT_BASE_A);
    Foc_Id_Fdbk = gateway_to_physical(
        value->current_dq_pu.d, MC_CURRENT_BASE_A);
    Foc_Iq_Fdbk = gateway_to_physical(
        value->current_dq_pu.q, MC_CURRENT_BASE_A);
    Foc_Ud_Ref = gateway_to_physical(
        value->voltage_reference_dq_pu.d, MC_VOLTAGE_BASE_V);
    Foc_Uq_Ref = gateway_to_physical(
        value->voltage_reference_dq_pu.q, MC_VOLTAGE_BASE_V);
    Foc_Idq_Ref = (Park_t){Foc_Id_Ref, Foc_Iq_Ref};
    Foc_Idq_Fdbk = (Park_t){Foc_Id_Fdbk, Foc_Iq_Fdbk};
    Foc_Udq_Ref = (Park_t){Foc_Ud_Ref, Foc_Uq_Ref};
    Foc_Inductor = (Park_t){
        gateway_to_physical(value->inductance_dq_pu.d,
            MC_FLUX_BASE_WB / MC_CURRENT_BASE_A),
        gateway_to_physical(value->inductance_dq_pu.q,
            MC_FLUX_BASE_WB / MC_CURRENT_BASE_A)};
    Foc_Iclark_Fdbk = (Clark_t){
        gateway_to_physical(value->current_ab_pu.a, MC_CURRENT_BASE_A),
        gateway_to_physical(value->current_ab_pu.b, MC_CURRENT_BASE_A)};
    Foc_Uclark_Ref = (Clark_t){
        gateway_to_physical(value->voltage_reference_ab_pu.a,
                            MC_VOLTAGE_BASE_V),
        gateway_to_physical(value->voltage_reference_ab_pu.b,
                            MC_VOLTAGE_BASE_V)};
    Foc_Pid_Speed_Handler.integral = gateway_to_physical(
        state->speed_pid.integral, MC_CURRENT_BASE_A);
    Foc_Pid_Speed_Handler.previous_error = gateway_to_physical(
        state->speed_pid.last_error, MC_SPEED_BASE_RPM);
    Foc_Pid_Speed_Handler.output = Foc_Iq_Ref;
    Foc_Pid_CurD_Handler.integral = gateway_to_physical(
        state->current_d_pid.integral, MC_VOLTAGE_BASE_V);
    Foc_Pid_CurD_Handler.previous_error = gateway_to_physical(
        state->current_d_pid.last_error, MC_CURRENT_BASE_A);
    Foc_Pid_CurD_Handler.output = Foc_Ud_Ref;
    Foc_Pid_CurQ_Handler.integral = gateway_to_physical(
        state->current_q_pid.integral, MC_VOLTAGE_BASE_V);
    Foc_Pid_CurQ_Handler.previous_error = gateway_to_physical(
        state->current_q_pid.last_error, MC_CURRENT_BASE_A);
    Foc_Pid_CurQ_Handler.output = Foc_Uq_Ref;
    if (state->mode == IDENTIFY)
    {
        Identification_State = (uint16_t)value->identification_state;
        Identification_Error = (uint16_t)value->identification_error;
        Identification_Rs = gateway_to_physical(
            state->identification.resistance_pu,
            MC_VOLTAGE_BASE_V / MC_CURRENT_BASE_A);
    }
    Foc_StartupActive = value->startup_active;
    Sensorless_UsingHfi = value->using_hfi;
    Mtpa_TableValid = state->mtpa.table_valid;
    Mtpa_RebuildPending = state->mtpa_rebuild_pending
                       || state->mtpa_rebuild_in_progress;
    Mtpa_RebuildFailed = state->mtpa_rebuild_failed;
    /* DONE/FAILED requests IDLE.  Preserve the last experiment mirrors after
     * that transition instead of overwriting them with Reset's zero state. */
    if (state->mode != IDENTIFY)
        return;
    bool injection_mode_changed = (uint32_t)Experiment.inj.mode
                               != (uint32_t)state->identification.injection_mode;
    if (injection_mode_changed)
    {
        /* The original PENDING state clears both amplitudes before Q and DQ.
         * Keep that manual safety gate: the operator must enter the next
         * stage's voltage explicitly. */
        Experiment.inj.Ud_amp = 0.0F;
        Experiment.inj.Uq_amp = 0.0F;
    }
    Experiment.Initialized = true;
    Experiment.Complete = value->identification_state == IDENTIFICATION_IQ_DONE;
    Experiment.Rs_est = Identification_Rs;
    Experiment.state = gateway_identification_state(
        value->identification_state);
    Experiment.inj.mode = (Inj_Mode_e)state->identification.injection_mode;
    Experiment.inj.State = state->identification.point_started
                        && value->identification_state
                           == IDENTIFICATION_IQ_INJECT_COLLECT;
    Experiment.inj.inj_state_d = state->identification.injection_d_positive
                               ? 1 : -1;
    Experiment.inj.inj_state_q = state->identification.injection_q_positive
                               ? 1 : -1;
    Experiment.inj.Vd = Foc_Ud_Ref;
    Experiment.inj.Vq = Foc_Uq_Ref;
    Experiment.inj.Imax = gateway_to_physical(
        state->identification.current_target_pu, MC_CURRENT_BASE_A);
    Experiment.inj.IDmax = Experiment.inj.Imax;
    Experiment.inj.IQmax = Experiment.inj.Imax;
    Experiment.pos = (int)state->identification.sample_count;
    Experiment.edge_count = (int)state->identification.edge_count;
    Experiment.repeat_count = (int)state->identification.repeat_count;
    Experiment.step_index = (int)state->identification.step_index;
    if (value->identification_state == IDENTIFICATION_IQ_PENDING
        || value->identification_state == IDENTIFICATION_IQ_DONE
        || value->identification_state == IDENTIFICATION_IQ_FAILED)
    {
        Experiment.LLS.ad0 = gateway_coefficient_to_physical(
            state->identification.coefficients.ad0, 1U);
        Experiment.LLS.add = gateway_coefficient_to_physical(
            state->identification.coefficients.add, 6U);
        Experiment.LLS.aq0 = gateway_coefficient_to_physical(
            state->identification.coefficients.aq0, 1U);
        Experiment.LLS.aqq = gateway_coefficient_to_physical(
            state->identification.coefficients.aqq, 2U);
        Experiment.LLS.adq = gateway_coefficient_to_physical(
            state->identification.coefficients.adq, 4U);
        Experiment.LLS.D.J = gateway_accum_to_physical(
            state->identification.coefficients.ssr_d,
            MC_CURRENT_BASE_A * MC_CURRENT_BASE_A);
        Experiment.LLS.Q.J = gateway_accum_to_physical(
            state->identification.coefficients.ssr_q,
            MC_CURRENT_BASE_A * MC_CURRENT_BASE_A);
        Experiment.LLS.D.R2 = McMath_ToFloat(
            state->identification.coefficients.r2_d);
        Experiment.LLS.Q.R2 = McMath_ToFloat(
            state->identification.coefficients.r2_q);
        Experiment.LLS.DQ.J[0] = gateway_accum_to_physical(
            state->identification.dq_sum_residual_d_squared,
            MC_CURRENT_BASE_A * MC_CURRENT_BASE_A);
        Experiment.LLS.DQ.J[1] = gateway_accum_to_physical(
            state->identification.dq_sum_residual_q_squared,
            MC_CURRENT_BASE_A * MC_CURRENT_BASE_A);
        Experiment.LLS.DQ.R2[0] = McMath_ToFloat(
            state->identification.coefficients.r2_dq_d);
        Experiment.LLS.DQ.R2[1] = McMath_ToFloat(
            state->identification.coefficients.r2_dq_q);
    }

    if (state->identification.step_index < IDENTIFICATION_IQ_MAX_STEPS)
    {
        uint16_t index = state->identification.step_index;
        bool show_d_results = state->identification.injection_mode
                           == IDENTIFICATION_IQ_INJECT_D;
        const IdentificationIqPoint_t* point = show_d_results
            ? &state->identification.d_results[index]
            : &state->identification.q_results[index];
        if (point->valid)
        {
            Experiment.results[index].Imax_value = gateway_to_physical(
                point->current_pu, MC_CURRENT_BASE_A);
            Experiment.results[index].avg_max_psi = gateway_to_physical(
                point->maximum_flux_pu, MC_FLUX_BASE_WB);
            Experiment.results[index].cycles_used = Experiment.repeat_times;
        }
    }
    if (state->identification.sample_count > 0U)
    {
        uint16_t index = (uint16_t)(state->identification.sample_count - 1U);
        Experiment.Ud_buf[index] = gateway_to_physical(
            state->identification.voltage_d[index], MC_VOLTAGE_BASE_V);
        Experiment.Uq_buf[index] = gateway_to_physical(
            state->identification.voltage_q[index], MC_VOLTAGE_BASE_V);
        Experiment.Id_buf[index] = gateway_to_physical(
            state->identification.current_d[index], MC_CURRENT_BASE_A);
        Experiment.Iq_buf[index] = gateway_to_physical(
            state->identification.current_q[index], MC_CURRENT_BASE_A);
        Experiment.psi_d_buf[index] = gateway_to_physical(
            state->identification.flux_d[index], MC_FLUX_BASE_WB);
        Experiment.psi_q_buf[index] = gateway_to_physical(
            state->identification.flux_q[index], MC_FLUX_BASE_WB);
    }
}

void FixedControl_Init(FixedControlState_t* state)
{
    if (state == NULL)
        return;
    *state = (FixedControlState_t){0};
    FocIq_DefaultParameters(&state->parameters);
    FocIq_Init(&state->foc, NULL);
    McMath_ResetDiagnostics();
    state->initialized = true;
    FixedControl_SelfTest = FixedControl_RunSelfTest();
}

void FixedControl_Reset(FixedControlState_t* state)
{
    if (state == NULL)
        return;
    FocIq_Reset(&state->foc, state->foc.control_angle_pu);
}

FixedControlOutput_t FixedControl_Step(FixedControlState_t* state,
                                      const FixedControlInput_t* input)
{
    FixedControlOutput_t output = {
        .pwm_duty = {0.5F, 0.5F, 0.5F}};
    if (state == NULL || input == NULL)
    {
        McMath_Diagnostics.invalid_input_count++;
        return output;
    }
    if (!state->initialized)
    {
        McMath_Diagnostics.invalid_input_count++;
        return output;
    }
    FixedControl_CycleCount++;
    FocMode_t requested_mode = gateway_validate_mode(Foc_Mode);
    if (requested_mode == IDENTIFY
        && state->gateway_previous_mode != IDENTIFY)
        gateway_initialize_identification_mirror();
    state->gateway_previous_mode = requested_mode;
    Foc_Reset = input->reset || gateway_validate_mode(Foc_Mode) == IDLE;
    gateway_read_parameters(&state->parameters);
    FocIqInput_t fixed_input = {
        .current_abc_pu = {
            gateway_from_physical(input->current_abc.a, MC_CURRENT_BASE_A),
            gateway_from_physical(input->current_abc.b, MC_CURRENT_BASE_A),
            gateway_from_physical(input->current_abc.c, MC_CURRENT_BASE_A)},
        .measured_angle_pu = McMath_FromFloat(
            input->theta_rad * MC_INV_TWO_PI_F),
        .measured_speed_pu = gateway_from_physical(
            input->speed_rpm, MC_SPEED_BASE_RPM),
        .bus_voltage_pu = gateway_from_physical(
            input->bus_voltage, MC_VOLTAGE_BASE_V)};
    output.bus_voltage_q24_raw = fixed_input.bus_voltage_pu;
    output.phase_a_current_q24_raw = fixed_input.current_abc_pu.a;
    output.bus_voltage_q24_v = gateway_to_physical(
        fixed_input.bus_voltage_pu, MC_VOLTAGE_BASE_V);
    output.phase_a_current_q24_a = gateway_to_physical(
        fixed_input.current_abc_pu.a, MC_CURRENT_BASE_A);
    FocIqOutput_t fixed_output = {0};
    FocIq_Run(&state->foc, &state->parameters,
              &fixed_input, &fixed_output);
    gateway_write_telemetry(&state->foc, &fixed_output, input->bus_voltage);
    output.pwm_duty = (Phase_t){
        McMath_ToFloat(fixed_output.pwm_duty.a),
        McMath_ToFloat(fixed_output.pwm_duty.b),
        McMath_ToFloat(fixed_output.pwm_duty.c)};
    Foc_Tcm = output.pwm_duty;
    output.voltage_dq = (Park_t){Foc_Ud_Ref, Foc_Uq_Ref};
    output.current_dq = (Park_t){Foc_Id_Fdbk, Foc_Iq_Fdbk};
    output.estimated_theta_rad = gateway_to_physical(
        fixed_output.estimated_angle_pu, MC_TWO_PI_F);
    output.estimated_speed_rpm = gateway_to_physical(
        fixed_output.estimated_speed_pu, MC_SPEED_BASE_RPM);
    return output;
}

bool FixedControl_BackgroundBuild(FixedControlState_t* state)
{
    return state != NULL && FocIq_BackgroundBuildMtpa(&state->foc);
}

void FixedControl_CommitBackground(FixedControlState_t* state)
{
    if (state != NULL)
        FocIq_CommitBackgroundMtpa(&state->foc);
}

FixedControlSelfTest_t FixedControl_RunSelfTest(void)
{
    FixedControlSelfTest_t result = {0};
    const mc_real_t tolerance = MC_CONST(0.00002);
    mc_real_t half = McMath_FromFloat(0.5F);
    if (McMath_Abs(McMath_Sub(half, MC_CONST(0.5))) > tolerance)
        result.conversion_failures++;
    if (McMath_Abs(McMath_Sub(McMath_Mul(half, half), MC_CONST(0.25)))
        > tolerance)
        result.arithmetic_failures++;
    if (McMath_Abs(McMath_Sub(
            McMath_Div(half, MC_CONST(0.25)), MC_CONST(2.0))) > tolerance)
        result.arithmetic_failures++;
    if (McMath_Abs(McMath_Sub(
            McMath_SinPu(MC_CONST(0.25)), MC_ONE)) > tolerance)
        result.trigonometric_failures++;
    if (McMath_Abs(McMath_Sub(
            McMath_CosPu(MC_CONST(0.5)), -MC_ONE)) > tolerance)
        result.trigonometric_failures++;
    if (McMath_Abs(McMath_Sub(
            McMath_Sqrt(MC_CONST(0.25)), half)) > tolerance)
        result.sqrt_failures++;
    return result;
}
