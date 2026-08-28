#include "motor_iq.h"

#include <stddef.h>

static mc_real_t motor_iq_ratio_u32(uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0U)
    {
        McMath_Diagnostics.divide_by_zero_count++;
        return MC_ZERO;
    }
#if defined(MC_NUMERIC_IQMATH)
    return (mc_real_t)(((int64_t)numerator << MC_Q_FRACTIONAL_BITS)
                       / denominator);
#else
    return (mc_real_t)numerator / (mc_real_t)denominator;
#endif
}

void MotorIq_Init(MotorIqState_t* state, const MotorIqConfig_t* config)
{
    if (state == NULL || config == NULL || config->position_modulus == 0U)
        return;
    *state = (MotorIqState_t){0};
    state->config = *config;
    if (state->config.speed_prescaler == 0U)
        state->config.speed_prescaler = 1U;
    state->initialized = true;
}

void MotorIq_Reset(MotorIqState_t* state)
{
    if (state == NULL)
        return;
    MotorIqConfig_t config = state->config;
    MotorIq_Init(state, &config);
}

void MotorIq_RunPosition(MotorIqState_t* state, uint16_t raw_position)
{
    if (state == NULL || !state->initialized)
        return;
    uint32_t normalized = ((uint32_t)raw_position
                         + state->config.position_modulus
                         - state->config.position_offset)
                        % state->config.position_modulus;
    state->theta_mechanical_pu = motor_iq_ratio_u32(
        normalized, state->config.position_modulus);
    state->theta_electrical_pu = AngleIq_WrapPu(McMath_Mul(
        state->theta_mechanical_pu, state->config.pole_pairs));
    state->speed_counter++;
    if (state->speed_counter < state->config.speed_prescaler)
        return;
    state->speed_counter = 0U;
    mc_real_t delta = AngleIq_ErrorPu(state->theta_mechanical_pu,
                                      state->last_theta_mechanical_pu);
    mc_real_t raw_speed = McMath_Mul(delta, state->config.speed_scale);
    state->speed_pu = McMath_Add(
        McMath_Mul(state->config.speed_filter_alpha, state->speed_pu),
        McMath_Mul(McMath_Sub(MC_ONE, state->config.speed_filter_alpha),
                   raw_speed));
    state->last_theta_mechanical_pu = state->theta_mechanical_pu;
}

void MotorIq_SetElectricalAngle(MotorIqState_t* state, mc_real_t angle_pu)
{
    if (state != NULL)
        state->theta_electrical_pu = AngleIq_WrapPu(angle_pu);
}

void MotorIq_SetMechanicalAngle(MotorIqState_t* state, mc_real_t angle_pu)
{
    if (state != NULL)
        state->theta_mechanical_pu = AngleIq_WrapPu(angle_pu);
}

void MotorIq_SetSpeed(MotorIqState_t* state, mc_real_t speed_pu)
{
    if (state != NULL)
        state->speed_pu = speed_pu;
}

mc_real_t MotorIq_GetElectricalAngle(const MotorIqState_t* state)
{
    return state == NULL ? MC_ZERO : state->theta_electrical_pu;
}

mc_real_t MotorIq_GetMechanicalAngle(const MotorIqState_t* state)
{
    return state == NULL ? MC_ZERO : state->theta_mechanical_pu;
}

mc_real_t MotorIq_GetSpeed(const MotorIqState_t* state)
{
    return state == NULL ? MC_ZERO : state->speed_pu;
}
