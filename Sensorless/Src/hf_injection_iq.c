#include "hf_injection_iq.h"

#include <stddef.h>
#include "fixed_numeric_config.h"
#include "parameters.h"

void HfiIq_DefaultConfig(HfiIqConfig_t* config)
{
    if (config == NULL)
        return;
    *config = (HfiIqConfig_t){
        .injection_voltage_pu = MC_CONST(HF_INJECTION_AMP / MC_VOLTAGE_BASE_V),
        .pll_kp = MC_CONST(HFI_PLL_KP / MC_SPEED_BASE_RPM),
        .pll_ki_step = MC_CONST(
            HFI_PLL_KI * MAIN_LOOP_TIME / MC_SPEED_BASE_RPM),
        .speed_to_angle_step = MC_CONST(
            MC_SPEED_BASE_RPM * MOTOR_PN * MAIN_LOOP_TIME / 60.0),
        .response_filter_alpha = MC_CONST(0.50)};
}

void HfiIq_Init(HfiIqState_t* state, const HfiIqConfig_t* config)
{
    if (state == NULL || config == NULL)
        return;
    *state = (HfiIqState_t){0};
    Iir1Iq_Init(&state->response_a_filter, config->response_filter_alpha);
    Iir1Iq_Init(&state->response_b_filter, config->response_filter_alpha);
    PllIq_Init(&state->pll, config->pll_kp, config->pll_ki_step,
               MC_CONST(-1.0), MC_CONST(1.0),
               config->speed_to_angle_step);
}

void HfiIq_Reset(HfiIqState_t* state, mc_real_t initial_angle_pu)
{
    if (state == NULL)
        return;
    state->previous_current = (ClarkIq_t){0};
    state->previous_high_frequency = (ClarkIq_t){0};
    state->base_current = (ClarkIq_t){0};
    state->response_current = (ClarkIq_t){0};
    state->phase_error_pu = MC_ZERO;
    state->injection_positive = false;
    Iir1Iq_Reset(&state->response_a_filter);
    Iir1Iq_Reset(&state->response_b_filter);
    PllIq_Reset(&state->pll, initial_angle_pu);
}

ClarkIq_t HfiIq_ProcessCurrent(HfiIqState_t* state, ClarkIq_t current)
{
    if (state == NULL)
        return current;
    state->base_current.a = McMath_Mul(
        McMath_Add(current.a, state->previous_current.a), MC_HALF);
    state->base_current.b = McMath_Mul(
        McMath_Add(current.b, state->previous_current.b), MC_HALF);
    ClarkIq_t high = {
        McMath_Sub(current.a, state->base_current.a),
        McMath_Sub(current.b, state->base_current.b)};
    mc_real_t sign = state->injection_positive ? McMath_Neg(MC_ONE) : MC_ONE;
    /* The old optional response IIR calls are commented out. */
    state->response_current.a = McMath_Mul(
        McMath_Sub(high.a, state->previous_high_frequency.a), sign);
    state->response_current.b = McMath_Mul(
        McMath_Sub(high.b, state->previous_high_frequency.b), sign);
    state->previous_current = current;
    state->previous_high_frequency = high;
    return state->base_current;
}

void HfiIq_Run(HfiIqState_t* state,
               const HfiIqConfig_t* config,
               const HfiIqInput_t* input,
               HfiIqOutput_t* output)
{
    if (state == NULL || config == NULL || input == NULL || output == NULL)
        return;
    state->pll.enabled = false; /* local PLL is a compatibility view only */
    if (state->enabled && !input->reset)
    {
        mc_real_t sine = McMath_SinPu(state->pll.angle_pu);
        mc_real_t cosine = McMath_CosPu(state->pll.angle_pu);
        mc_real_t ma = McMath_Abs(state->response_current.a);
        mc_real_t mb = McMath_Abs(state->response_current.b);
        mc_real_t scale = ma > mb ? ma : mb;
        state->phase_error_pu = MC_ZERO;
        if (scale > MC_ZERO)
        {
            mc_real_t a = McMath_Div(state->response_current.a, scale);
            mc_real_t b = McMath_Div(state->response_current.b, scale);
            mc_real_t norm = McMath_Sqrt(McMath_Add(McMath_Mul(a, a), McMath_Mul(b, b)));
            if (McMath_Mul(scale, norm) > MC_CONST(0.0001 / MC_CURRENT_BASE_A))
                state->phase_error_pu = McMath_Div(McMath_Sub(
                    McMath_Mul(a, sine), McMath_Mul(b, cosine)), norm);
        }
    }

    *output = (HfiIqOutput_t){
        .filtered_current_ab_pu = state->base_current,
        .injection_voltage_dq_pu = {0},
        .angle_pu = state->pll.angle_pu,
        .speed_pu = state->pll.speed_pu,
        .phase_error_pu = state->phase_error_pu};
}

ParkIq_t HfiIq_InjectVoltage(HfiIqState_t* state,
                            const HfiIqConfig_t* config, bool reset)
{
    ParkIq_t injection = {0};
    if (state == NULL || config == NULL || !state->enabled || reset)
        return injection;
    state->injection_positive = !state->injection_positive;
    injection.d = state->injection_positive ? config->injection_voltage_pu
                                            : McMath_Neg(config->injection_voltage_pu);
    return injection;
}
