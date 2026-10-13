#include "leso_iq.h"

#include <stddef.h>
#include "fixed_numeric_config.h"
#include "parameters.h"

mc_real_t LesoIq_MaximumBandwidthStep(void)
{
#if defined(MC_NUMERIC_IQMATH)
    /* Derive Ts from the same timer constants as MAIN_LOOP_TIME, using only
     * integer constant arithmetic. Truncating the final positive quotient
     * prevents the usual nearest-Q24 conversion from exceeding the cap. */
    return (mc_real_t)(((uint64_t)LESO_IQ_WC_MAX_RAD_S
        * ((uint64_t)1 << MC_Q_FRACTIONAL_BITS)
        * (MAIN_INT_TIMER_PRESCALER + 1U)
        * MAIN_INT_TIMER_PERIOD * 2U) / MCU_MAIN_FREQ);
#else
    return MC_CONST(LESO_IQ_WC_MAX_RAD_S * MAIN_LOOP_TIME);
#endif
}

void LesoIq_DefaultConfig(LesoIqConfig_t* config)
{
    if (config == NULL)
        return;
    /* Discrete normalized coefficients derived from the original Rs/Lq model. */
    *config = (LesoIqConfig_t){
        .resistance_gain = MC_CONST(MAIN_LOOP_TIME * MOTOR_RS / MOTOR_LQ),
        .voltage_gain = MC_CONST(
            MAIN_LOOP_TIME * MC_VOLTAGE_BASE_V
            / (MOTOR_LQ * MC_CURRENT_BASE_A)),
        .beta1_step = MC_CONST(2.0 * LESO_WC_MIN * MAIN_LOOP_TIME),
        .beta2_step = MC_CONST(
            LESO_WC_MIN * LESO_WC_MIN * MAIN_LOOP_TIME * MAIN_LOOP_TIME),
        .disturbance_to_emf = MC_CONST(
            MOTOR_LQ * MC_CURRENT_BASE_A
            / (MAIN_LOOP_TIME * MC_VOLTAGE_BASE_V)),
#if defined(MC_NUMERIC_IQMATH)
        /* The old 1e25 clamp is outside Q24. Use the representable limit,
         * not the unrelated 1.5 pu limit introduced during migration. */
        .state_limit = (mc_real_t)INT32_MAX,
#else
        .state_limit = MC_CONST(1e25),
#endif
        .pll_kp = MC_CONST(SMO_PLL_KP / MC_SPEED_BASE_RPM),
        .pll_ki_step = MC_CONST(
            SMO_PLL_KI * MAIN_LOOP_TIME / MC_SPEED_BASE_RPM),
        .speed_to_angle_step = MC_CONST(
            MC_SPEED_BASE_RPM * MOTOR_PN * MAIN_LOOP_TIME / 60.0),
        .emf_filter_alpha = MC_CONST(0.75),
        .slow_filter_alpha = MC_CONST(0.95),
        .resistance_pu = MC_CONST(MOTOR_RS * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V),
        .voltage_to_flux_step = MC_CONST(MAIN_LOOP_TIME * MC_VOLTAGE_BASE_V / MC_FLUX_BASE_WB),
        .inductance_q_pu = MC_CONST(MOTOR_LQ * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB),
        .minimum_inductance_pu = MC_CONST(0.001 * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB),
        .bandwidth_gain = MC_CONST(LESO_WC_GAIN),
        .bandwidth_factor = MC_CONST(1.9),
        .bandwidth_minimum_step = MC_CONST(LESO_WC_MIN * MAIN_LOOP_TIME),
        .bandwidth_maximum_step = LesoIq_MaximumBandwidthStep(),
        .speed_to_electrical_step = MC_CONST(
            MC_SPEED_BASE_RPM * MOTOR_PN * MAIN_LOOP_TIME * 6.283185307179586 / 60.0)};
}

void LesoIq_Init(LesoIqState_t* state, const LesoIqConfig_t* config)
{
    if (state == NULL || config == NULL)
        return;
    *state = (LesoIqState_t){0};
    Iir1Iq_Init(&state->emf_d_filter, config->emf_filter_alpha);
    Iir1Iq_Init(&state->emf_q_filter, config->emf_filter_alpha);
    Iir1Iq_Init(&state->slow_d_filter, config->slow_filter_alpha);
    Iir1Iq_Init(&state->slow_q_filter, config->slow_filter_alpha);
    PllIq_Init(&state->pll,
               config->pll_kp,
               config->pll_ki_step,
               MC_CONST(-1.5),
               MC_CONST(1.5),
               config->speed_to_angle_step);
}

void LesoIq_Reset(LesoIqState_t* state, mc_real_t initial_angle_pu)
{
    if (state == NULL)
        return;
    state->current_estimate = (ClarkIq_t){0};
    state->disturbance = (ClarkIq_t){0};
    state->emf_estimate = (ClarkIq_t){0};
    state->emf_dq = (ParkIq_t){0};
    state->emf_filtered = (ParkIq_t){0};
    state->emf_slow_filtered = (ParkIq_t){0};
    state->phase_error_pu = MC_ZERO;
    state->bandwidth_step = MC_ZERO;
    state->last_voltage_ab_pu = (ClarkIq_t){0};
    state->last_current_ab_pu = (ClarkIq_t){0};
    state->last_inductance_q_pu = MC_ZERO;
    Iir1Iq_Reset(&state->emf_d_filter);
    Iir1Iq_Reset(&state->emf_q_filter);
    Iir1Iq_Reset(&state->slow_d_filter);
    Iir1Iq_Reset(&state->slow_q_filter);
    PllIq_Reset(&state->pll, initial_angle_pu);
}

static void leso_update_axis(mc_real_t voltage,
                             mc_real_t current,
                             mc_real_t beta1_step, mc_real_t beta2_step,
                             mc_real_t resistance_gain, mc_real_t voltage_gain,
                             mc_real_t disturbance_to_emf, mc_real_t limit,
                             mc_real_t* current_estimate,
                             mc_real_t* disturbance,
                             mc_real_t* emf)
{
    mc_real_t error = McMath_Sub(*current_estimate, current);
    *disturbance = McMath_Clamp(
        McMath_Sub(*disturbance, McMath_Mul(beta2_step, error)),
        McMath_Neg(limit), limit);
    mc_real_t delta = McMath_Add(
        McMath_Sub(McMath_Mul(voltage_gain, voltage),
                   McMath_Mul(resistance_gain, current)),
        McMath_Sub(*disturbance, McMath_Mul(beta1_step, error)));
    *current_estimate = McMath_Clamp(
        McMath_Add(*current_estimate, delta),
        McMath_Neg(limit), limit);
    *emf = McMath_Neg(McMath_Mul(disturbance_to_emf, *disturbance));
}

void LesoIq_Run(LesoIqState_t* state,
                const LesoIqConfig_t* config,
                const LesoIqInput_t* input,
                LesoIqOutput_t* output)
{
    if (state == NULL || config == NULL || input == NULL || output == NULL)
        return;
    /* The active legacy path freezes observer memories when stopped/disabled.
     * Only the explicit Reset API erases them. The shared PLL owns position. */
    state->pll.angle_pu = input->angle_pu;
    state->pll.speed_pu = input->speed_pu;
    state->pll.enabled = false;
    if (state->enabled && !input->reset)
    {
        state->last_voltage_ab_pu = input->voltage_ab_pu;
        state->last_current_ab_pu = input->current_ab_pu;
        mc_real_t lq = input->inductance_q_pu;
        if (lq < config->minimum_inductance_pu)
            lq = config->minimum_inductance_pu;
        state->last_inductance_q_pu = lq;
        mc_real_t voltage_gain = McMath_Div(config->voltage_to_flux_step, lq);
        mc_real_t resistance_gain = McMath_Mul(config->resistance_pu, voltage_gain);
        mc_real_t emf_gain = McMath_Div(lq, config->voltage_to_flux_step);
        /* Schedule bandwidth by speed magnitude in both directions. Keep the
         * signed input speed below for EMF phase-detector direction handling. */
        mc_real_t speed_magnitude = McMath_Abs(input->speed_pu);
        mc_real_t wc_step = McMath_Mul(McMath_Mul(config->bandwidth_gain,
            config->bandwidth_factor), McMath_Mul(speed_magnitude,
                                                config->speed_to_electrical_step));
        if (wc_step > config->bandwidth_maximum_step)
            wc_step = config->bandwidth_maximum_step;
        if (wc_step < config->bandwidth_minimum_step)
            wc_step = config->bandwidth_minimum_step;
        /* Enforce after BOTH configurable bounds: even a direct state/config
         * write with an excessive minimum cannot bypass the fixed ceiling. */
        if (wc_step > LesoIq_MaximumBandwidthStep())
            wc_step = LesoIq_MaximumBandwidthStep();
        state->bandwidth_step = wc_step;
        mc_real_t beta1 = McMath_Add(wc_step, wc_step);
        mc_real_t beta2 = McMath_Mul(wc_step, wc_step);
        leso_update_axis(input->voltage_ab_pu.a, input->current_ab_pu.a,
            beta1, beta2, resistance_gain, voltage_gain, emf_gain,
            config->state_limit, &state->current_estimate.a,
            &state->disturbance.a, &state->emf_estimate.a);
        leso_update_axis(input->voltage_ab_pu.b, input->current_ab_pu.b,
            beta1, beta2, resistance_gain, voltage_gain, emf_gain,
            config->state_limit, &state->current_estimate.b,
            &state->disturbance.b, &state->emf_estimate.b);
        mc_real_t magnitude_a = McMath_Abs(state->emf_estimate.a);
        mc_real_t magnitude_b = McMath_Abs(state->emf_estimate.b);
        mc_real_t scale = magnitude_a > magnitude_b ? magnitude_a : magnitude_b;
        state->phase_error_pu = MC_ZERO;
        if (scale > MC_ZERO)
        {
            /* Scale before squaring so small EMF does not vanish in Q24. */
            mc_real_t a = McMath_Div(state->emf_estimate.a, scale);
            mc_real_t b = McMath_Div(state->emf_estimate.b, scale);
            mc_real_t norm = McMath_Sqrt(McMath_Add(McMath_Mul(a, a),
                                                   McMath_Mul(b, b)));
            if (McMath_Mul(scale, norm) > MC_CONST(0.0001 / MC_VOLTAGE_BASE_V))
            {
                mc_real_t projected = McMath_Sub(
                    McMath_Neg(McMath_Mul(a, McMath_CosPu(input->angle_pu))),
                    McMath_Mul(b, McMath_SinPu(input->angle_pu)));
                if (input->speed_pu < MC_ZERO)
                    projected = McMath_Neg(projected);
                state->phase_error_pu = McMath_Div(projected, norm);
            }
        }
    }
    *output = (LesoIqOutput_t){
        .angle_pu = state->pll.angle_pu,
        .speed_pu = state->pll.speed_pu,
        .phase_error_pu = state->phase_error_pu,
        .emf_ab_pu = state->emf_estimate,
        .emf_dq_pu = state->emf_filtered};
}
