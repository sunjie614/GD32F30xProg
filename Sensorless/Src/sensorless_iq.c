#include "sensorless_iq.h"

#include <stddef.h>
#include "fixed_numeric_config.h"
#include "parameters.h"

void SensorlessIq_DefaultConfig(SensorlessIqConfig_t* config)
{
    if (config == NULL)
        return;
    /* Keep the original physical coefficients. Only the stored output unit
     * changes: electrical rad/s is normalized by 1000, not mechanical rpm. */
    *config = (SensorlessIqConfig_t){
        .kp = MC_CONST(SENSORLESS_PLL_KP / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .ki_step = MC_CONST(SENSORLESS_PLL_KI * MAIN_LOOP_TIME / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .kd_step = MC_CONST(SENSORLESS_PLL_KD / MAIN_LOOP_TIME / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .minimum = MC_CONST(SENSORLESS_PLL_MIN_OUTPUT / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .maximum = MC_CONST(SENSORLESS_PLL_MAX_OUTPUT / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .integral_limit = MC_CONST(SENSORLESS_PLL_INTEGRAL_LIMIT / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .angle_step = MC_CONST(SENSORLESS_IQ_OMEGA_BASE_RAD_S * MAIN_LOOP_TIME / 6.283185307179586),
        .speed_gain = MC_CONST(SENSORLESS_IQ_OMEGA_BASE_RAD_S * 60.0 / (6.283185307179586 * MOTOR_PN * MC_SPEED_BASE_RPM)),
        .switch_speed_pu = MC_CONST(SENSORLESS_SWITCH_SPEED / MC_SPEED_BASE_RPM),
        .threshold_hfi_pu = MC_CONST((SENSORLESS_SWITCH_SPEED + SENSORLESS_HYSTERESIS) / MC_SPEED_BASE_RPM),
        .threshold_leso_pu = MC_CONST((SENSORLESS_SWITCH_SPEED - SENSORLESS_HYSTERESIS) / MC_SPEED_BASE_RPM),
        .method_margin_pu = MC_CONST(10.0 / MC_SPEED_BASE_RPM),
        .pll_wc_pu = MC_CONST(SENSORLESS_PLL_WC / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .wc_speed_slope = MC_CONST(0.2 * MC_SPEED_BASE_RPM / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .dynamic_speed_threshold_pu = MC_CONST(500.0 / MC_SPEED_BASE_RPM),
        .kp_maximum = MC_CONST(130.0 / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .ki_step_maximum = MC_CONST(4225.0 * MAIN_LOOP_TIME / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .low_speed_kp = MC_CONST(50.0 / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .low_speed_ki_step = MC_CONST(625.0 * MAIN_LOOP_TIME / SENSORLESS_IQ_OMEGA_BASE_RAD_S),
        .wc_to_ki_step = MC_CONST(SENSORLESS_IQ_OMEGA_BASE_RAD_S * MAIN_LOOP_TIME),
        /* Original bilinear 10 Hz / 500 Hz IIR2. Coefficients are calculated
         * once at initialization/the gateway; never with floating math here. */
        .filter_b0 = MC_CONST(0.0036216815149286421),
        .filter_b1 = MC_CONST(0.0072433630298572842),
        .filter_b2 = MC_CONST(0.0036216815149286421),
        .filter_a1 = MC_CONST(-1.8226949251963083),
        .filter_a2 = MC_CONST(0.83718165125602262),
        .speed_prescaler = 10U};
}

void SensorlessIq_Init(SensorlessIqState_t* state,
                       const SensorlessIqConfig_t* config)
{
    if (state == NULL || config == NULL)
        return;
    *state = (SensorlessIqState_t){0};
    PidIq_Init(&state->common_pid, config->kp, config->ki_step,
               config->kd_step, config->minimum, config->maximum,
               config->integral_limit);
    BiquadIq_Init(&state->speed_filter, config->filter_b0,
                  config->filter_b1, config->filter_b2,
                  config->filter_a1, config->filter_a2);
    state->pll_wc_pu = config->pll_wc_pu;
    state->method = SENSORLESS_IQ_FLYING;
    state->initialized = config->speed_prescaler != 0U;
}

void SensorlessIq_Reset(SensorlessIqState_t* state,
                        mc_real_t initial_angle_pu)
{
    if (state == NULL)
        return;
    PidIq_Reset(&state->common_pid);
    BiquadIq_Reset(&state->speed_filter);
    state->angle_pu = AngleIq_WrapPu(initial_angle_pu);
    state->speed_pu = MC_ZERO;
    state->omega_pu = MC_ZERO;
    state->phase_error = MC_ZERO;
    state->speed_sum = 0;
    state->integral_product_residual = 0;
    state->angle_product_residual = 0;
    state->previous_integral_output = MC_ZERO;
    state->speed_count = 0U;
    state->reset_previous = false;
}

ClarkIq_t SensorlessIq_FilterCurrent(SensorlessIqState_t* state,
                                      HfiIqState_t* hfi, ClarkIq_t current,
                                      bool enabled, bool reset)
{
    if (state == NULL || hfi == NULL)
        return current;
    state->enabled = enabled;
    if (!enabled || reset || !hfi->enabled)
        return current;
    return HfiIq_ProcessCurrent(hfi, current);
}

static mc_real_t sensorless_iq_integral_product(mc_real_t a, mc_real_t b,
                                                int64_t* residual)
{
#if defined(MC_NUMERIC_IQMATH)
    int64_t product = (int64_t)a * b + *residual;
    int64_t magnitude = product < 0 ? -product : product;
    int64_t rounded = (magnitude + ((int64_t)1 << 23)) >> 24;
    if (product < 0)
        rounded = -rounded;
    if (rounded > INT32_MAX || rounded < INT32_MIN)
    {
        *residual = 0;
        return McMath_Mul(a, b); /* centralized saturation diagnostics */
    }
    *residual = product - rounded * ((int64_t)1 << 24);
    return (mc_real_t)rounded;
#else
    (void)residual;
    return McMath_Mul(a, b);
#endif
}

static mc_real_t sensorless_iq_pid_update(SensorlessIqState_t* state,
                                         mc_real_t error, bool reset)
{
    PidIqState_t* pid = &state->common_pid;
    if (reset)
    {
        PidIq_Reset(pid);
        state->integral_product_residual = 0;
        state->previous_integral_output = MC_ZERO;
        return MC_ZERO;
    }
    if (pid->integral != state->previous_integral_output)
        state->integral_product_residual = 0; /* explicit calibration write */
    mc_real_t proportional = McMath_Mul(pid->kp, error);
    mc_real_t candidate = McMath_Add(proportional, pid->integral);
    /* Original conditional integration tests P+I before the derivative. */
    if (candidate >= pid->minimum && candidate <= pid->maximum
        && pid->ki_step != MC_ZERO)
    {
        mc_real_t integrated = McMath_Add(pid->integral,
            sensorless_iq_integral_product(pid->ki_step, error,
                                          &state->integral_product_residual));
        pid->integral = McMath_Clamp(integrated, McMath_Neg(pid->integral_limit),
                                    pid->integral_limit);
        if (pid->integral != integrated)
            state->integral_product_residual = 0;
    }
    state->previous_integral_output = pid->integral;
    mc_real_t derivative = MC_ZERO;
    if (pid->kd_step != MC_ZERO)
    {
        derivative = McMath_Mul(pid->kd_step, McMath_Sub(error, pid->last_error));
        pid->last_error = error;
    }
    pid->output = McMath_Clamp(McMath_Add(McMath_Add(proportional,
        pid->integral), derivative), pid->minimum, pid->maximum);
    return pid->output;
}

static void sensorless_iq_choose_method(SensorlessIqState_t* state,
                                        const SensorlessIqConfig_t* config,
                                        const SensorlessIqInput_t* input)
{
    mc_real_t target = McMath_Abs(input->speed_target_pu);
    mc_real_t reference = McMath_Abs(input->speed_reference_pu);
    mc_real_t feedback = McMath_Abs(input->speed_feedback_pu);
    mc_real_t threshold = config->switch_speed_pu;
    if (target >= McMath_Add(threshold, config->method_margin_pu))
    {
        if (reference >= threshold)
            state->method = feedback >= threshold ? SENSORLESS_IQ_LES_OBSERVER
                                                  : SENSORLESS_IQ_HF_INJECTION;
        else
            state->method = feedback <= threshold ? SENSORLESS_IQ_HF_INJECTION
                                                  : SENSORLESS_IQ_LES_OBSERVER;
    }
    else if (target <= McMath_Sub(threshold, config->method_margin_pu))
    {
        if (reference <= threshold)
            state->method = feedback <= threshold ? SENSORLESS_IQ_HF_INJECTION
                                                  : SENSORLESS_IQ_LES_OBSERVER;
        else
            state->method = feedback >= threshold ? SENSORLESS_IQ_LES_OBSERVER
                                                  : SENSORLESS_IQ_HF_INJECTION;
    }
}

void SensorlessIq_UpdatePosition(SensorlessIqState_t* state,
                                 const SensorlessIqConfig_t* config,
                                 const SensorlessIqInput_t* input,
                                 LesoIqState_t* leso, HfiIqState_t* hfi,
                                 SensorlessIqOutput_t* output)
{
    if (state == NULL || config == NULL || input == NULL || leso == NULL
        || hfi == NULL || output == NULL || !state->initialized
        || config->speed_prescaler == 0U)
        return;
    state->enabled = input->enabled;
    state->method = input->requested_method;
    if (state->enabled)
    {
        sensorless_iq_choose_method(state, config, input);
        state->phase_error = state->method == SENSORLESS_IQ_HF_INJECTION
                           ? hfi->phase_error_pu : leso->phase_error_pu;
        /* Kp/Ki are retained from the previous adaptive update. The gateway
         * may explicitly write them; do not overwrite them from defaults. */
        state->common_pid.kd_step = config->kd_step;
        state->common_pid.minimum = config->minimum;
        state->common_pid.maximum = config->maximum;
        state->common_pid.integral_limit = config->integral_limit;
        state->omega_pu = sensorless_iq_pid_update(state,
                                                  state->phase_error, input->reset);
        if (!input->reset)
            state->angle_pu = AngleIq_WrapPu(McMath_Add(state->angle_pu,
                sensorless_iq_integral_product(state->omega_pu,
                    config->angle_step, &state->angle_product_residual)));
        state->speed_sum += McMath_Mul(state->omega_pu, config->speed_gain);
        if (++state->speed_count >= config->speed_prescaler)
        {
#if defined(MC_NUMERIC_IQMATH)
            mc_real_t averaged = (mc_real_t)(state->speed_sum / config->speed_prescaler);
#else
            mc_real_t averaged = (mc_real_t)(state->speed_sum / (mc_accum_t)config->speed_prescaler);
#endif
            state->speed_filter.b0 = config->filter_b0;
            state->speed_filter.b1 = config->filter_b1;
            state->speed_filter.b2 = config->filter_b2;
            state->speed_filter.a1 = config->filter_a1;
            state->speed_filter.a2 = config->filter_a2;
            state->speed_pu = BiquadIq_Run(&state->speed_filter, averaged);
            state->speed_sum = 0;
            state->speed_count = 0U;
        }
        leso->pll.angle_pu = state->angle_pu;
        leso->pll.speed_pu = state->speed_pu;
        hfi->pll.angle_pu = state->angle_pu;
        /* The legacy Hfi_Speed is not updated by the commented local PLL. */
        /* Apply the forward gain schedule to reverse motion as well. Only
         * scheduling uses magnitude; the PLL angle/speed retain their signs. */
        mc_real_t speed_magnitude = McMath_Abs(state->speed_pu);
        if (speed_magnitude > config->dynamic_speed_threshold_pu)
        {
            state->pll_wc_pu = McMath_Add(config->pll_wc_pu,
                McMath_Mul(McMath_Sub(speed_magnitude,
                    config->dynamic_speed_threshold_pu), config->wc_speed_slope));
            state->common_pid.kp = McMath_Add(state->pll_wc_pu, state->pll_wc_pu);
            state->common_pid.ki_step = McMath_Mul(McMath_Mul(state->pll_wc_pu,
                state->pll_wc_pu), config->wc_to_ki_step);
            if (state->common_pid.kp > config->kp_maximum)
                state->common_pid.kp = config->kp_maximum;
            if (state->common_pid.ki_step > config->ki_step_maximum)
                state->common_pid.ki_step = config->ki_step_maximum;
        }
        else
        {
            state->common_pid.kp = config->low_speed_kp;
            state->common_pid.ki_step = config->low_speed_ki_step;
        }
    }
    *output = (SensorlessIqOutput_t){
        .angle_pu = McMath_Add(state->angle_pu, config->theta_adjustment_pu),
        .speed_pu = state->speed_pu, .omega_pu = state->omega_pu,
        .phase_error = state->phase_error, .method = state->method,
        .using_hfi = state->method == SENSORLESS_IQ_HF_INJECTION};
}

void SensorlessIq_Calculate(SensorlessIqState_t* state,
                            const SensorlessIqConfig_t* config,
                            const SensorlessIqInput_t* input,
                            const LesoIqConfig_t* leso_config,
                            const HfiIqConfig_t* hfi_config,
                            LesoIqState_t* leso, HfiIqState_t* hfi,
                            FlyingIqState_t* flying)
{
    if (state == NULL || config == NULL || input == NULL || leso_config == NULL
        || hfi_config == NULL || leso == NULL || hfi == NULL || flying == NULL)
        return;
    /* Original SetCurrent/SetVoltage occur before Calculate, even on reset.
     * Voltage capture uses the previous enable state, before judge_strategy. */
    leso->last_current_ab_pu = input->current_ab_pu;
    if (leso->enabled)
        leso->last_voltage_ab_pu = input->voltage_ab_pu;
    /* MainInt also calls the original Set_Inductor before Calculate/Reset,
     * so its effective parameter mirror updates even while the observer is
     * disabled. The running observer uses this same minimum rule. */
    leso->last_inductance_q_pu = input->inductance_q_pu;
    if (leso->last_inductance_q_pu < leso_config->minimum_inductance_pu)
        leso->last_inductance_q_pu = leso_config->minimum_inductance_pu;
    /* The original Calculate checks Reset, not Sensorless_Enabled. Position,
     * current filtering and voltage injection have their separate gates. */
    if (input->reset)
        return;
    if (state->reset_previous)
        FlyingIq_SetEnabled(flying, true);
    if (!flying->completed)
        FlyingIq_Run(flying, false);
    mc_real_t reference = McMath_Abs(input->speed_reference_pu);
    mc_real_t feedback = McMath_Abs(input->speed_feedback_pu);
    if (reference >= config->threshold_leso_pu)
    {
        if (feedback >= config->threshold_leso_pu)
            leso->enabled = true;
    }
    else if (feedback < config->threshold_leso_pu)
        leso->enabled = false;
    if (reference <= config->threshold_hfi_pu)
    {
        if (feedback <= config->threshold_hfi_pu)
        {
            if (!hfi->enabled)
            {
                mc_real_t error = AngleIq_WrapPu(McMath_Add(
                    McMath_Sub(state->angle_pu, hfi->pll.angle_pu), MC_HALF));
                error = McMath_Sub(error, MC_HALF);
                if (error >= MC_CONST(0.125) || error <= MC_CONST(-0.125))
                {
                    hfi->pll.angle_pu = state->angle_pu;
                    /* Compatibility local PID is not executed, but the old
                     * SetInitialPosition also wrote its integral to theta. */
                    hfi->pll.pid.integral = state->angle_pu;
                }
            }
            hfi->enabled = true;
        }
    }
    else if (feedback > config->threshold_hfi_pu)
        hfi->enabled = false;
    if (hfi->enabled)
    {
        HfiIqInput_t hfi_input = {.reset = false};
        HfiIqOutput_t ignored;
        HfiIq_Run(hfi, hfi_config, &hfi_input, &ignored);
    }
    if (leso->enabled)
    {
        LesoIqInput_t leso_input = {
            .voltage_ab_pu = leso->last_voltage_ab_pu,
            .current_ab_pu = leso->last_current_ab_pu,
            .angle_pu = leso->pll.angle_pu,
            .speed_pu = leso->pll.speed_pu,
            .inductance_q_pu = input->inductance_q_pu,
            .reset = false};
        LesoIqOutput_t ignored;
        LesoIq_Run(leso, leso_config, &leso_input, &ignored);
    }
    /* Legacy Reset_Prev is assigned only after a non-reset Calculate. Its
     * early reset return means the release hook was never reliably armed. */
    state->reset_previous = input->reset;
}

ParkIq_t SensorlessIq_InjectVoltage(SensorlessIqState_t* state,
                                    const HfiIqConfig_t* config,
                                    HfiIqState_t* hfi, ParkIq_t voltage,
                                    bool reset)
{
    if (state == NULL || !state->enabled || reset)
        return voltage;
    ParkIq_t injection = HfiIq_InjectVoltage(hfi, config, reset);
    voltage.d = McMath_Add(voltage.d, injection.d);
    voltage.q = McMath_Add(voltage.q, injection.q);
    return voltage;
}
