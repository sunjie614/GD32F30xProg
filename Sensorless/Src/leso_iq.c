#include "leso_iq.h"

#include <stddef.h>
#include "fixed_numeric_config.h"
#include "parameters.h"

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
        .state_limit = MC_CONST(1.5),
        .pll_kp = MC_CONST(SMO_PLL_KP / MC_SPEED_BASE_RPM),
        .pll_ki_step = MC_CONST(
            SMO_PLL_KI * MAIN_LOOP_TIME / MC_SPEED_BASE_RPM),
        .speed_to_angle_step = MC_CONST(
            MC_SPEED_BASE_RPM * MOTOR_PN * MAIN_LOOP_TIME / 60.0),
        .emf_filter_alpha = MC_CONST(0.75),
        .slow_filter_alpha = MC_CONST(0.95)};
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
    Iir1Iq_Reset(&state->emf_d_filter);
    Iir1Iq_Reset(&state->emf_q_filter);
    Iir1Iq_Reset(&state->slow_d_filter);
    Iir1Iq_Reset(&state->slow_q_filter);
    PllIq_Reset(&state->pll, initial_angle_pu);
}

static void leso_update_axis(mc_real_t voltage,
                             mc_real_t current,
                             const LesoIqConfig_t* config,
                             mc_real_t* current_estimate,
                             mc_real_t* disturbance,
                             mc_real_t* emf)
{
    mc_real_t error = McMath_Sub(*current_estimate, current);
    *disturbance = McMath_Clamp(
        McMath_Sub(*disturbance, McMath_Mul(config->beta2_step, error)),
        McMath_Neg(config->state_limit), config->state_limit);
    mc_real_t delta = McMath_Add(
        McMath_Sub(McMath_Mul(config->voltage_gain, voltage),
                   McMath_Mul(config->resistance_gain, current)),
        McMath_Sub(*disturbance, McMath_Mul(config->beta1_step, error)));
    *current_estimate = McMath_Clamp(
        McMath_Add(*current_estimate, delta),
        McMath_Neg(config->state_limit), config->state_limit);
    *emf = McMath_Neg(McMath_Mul(config->disturbance_to_emf,
                                 *disturbance));
}

void LesoIq_Run(LesoIqState_t* state,
                const LesoIqConfig_t* config,
                const LesoIqInput_t* input,
                LesoIqOutput_t* output)
{
    if (state == NULL || config == NULL || input == NULL || output == NULL)
        return;
    if (input->reset)
        LesoIq_Reset(state, state->pll.angle_pu);
    PllIq_SetEnabled(&state->pll, state->enabled && !input->reset);

    leso_update_axis(input->voltage_ab_pu.a, input->current_ab_pu.a,
                     config, &state->current_estimate.a,
                     &state->disturbance.a, &state->emf_estimate.a);
    leso_update_axis(input->voltage_ab_pu.b, input->current_ab_pu.b,
                     config, &state->current_estimate.b,
                     &state->disturbance.b, &state->emf_estimate.b);

    mc_real_t measured_angle = McMath_Atan2Pu(
        McMath_Neg(state->emf_estimate.a), state->emf_estimate.b);
    state->phase_error_pu = AngleIq_ErrorPu(measured_angle,
                                            state->pll.angle_pu);
    PllIq_Run(&state->pll, state->phase_error_pu);
    state->emf_dq = TransformIq_Park(state->emf_estimate,
                                     state->pll.angle_pu);
    state->emf_filtered.d = Iir1Iq_Run(&state->emf_d_filter,
                                       state->emf_dq.d);
    state->emf_filtered.q = Iir1Iq_Run(&state->emf_q_filter,
                                       state->emf_dq.q);
    state->emf_slow_filtered.d = Iir1Iq_Run(&state->slow_d_filter,
                                            state->emf_filtered.d);
    state->emf_slow_filtered.q = Iir1Iq_Run(&state->slow_q_filter,
                                            state->emf_filtered.q);
    *output = (LesoIqOutput_t){
        .angle_pu = state->pll.angle_pu,
        .speed_pu = state->pll.speed_pu,
        .phase_error_pu = state->phase_error_pu,
        .emf_ab_pu = state->emf_estimate,
        .emf_dq_pu = state->emf_filtered};
}
