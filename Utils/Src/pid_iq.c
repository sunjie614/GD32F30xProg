#include "pid_iq.h"

#include <stddef.h>

void PidIq_Init(PidIqState_t* state,
                mc_real_t kp,
                mc_real_t ki_step,
                mc_real_t kd_step,
                mc_real_t minimum,
                mc_real_t maximum,
                mc_real_t integral_limit)
{
    if (state == NULL)
        return;
    *state = (PidIqState_t){0};
    state->kp = kp;
    state->ki_step = ki_step;
    state->kd_step = kd_step;
    state->minimum = minimum;
    state->maximum = maximum;
    state->integral_limit = McMath_Abs(integral_limit);
}

void PidIq_Reset(PidIqState_t* state)
{
    if (state == NULL)
        return;
    state->integral = MC_ZERO;
    state->last_error = MC_ZERO;
    state->output = MC_ZERO;
}

mc_real_t PidIq_Run(PidIqState_t* state, mc_real_t error)
{
    if (state == NULL)
        return MC_ZERO;

    mc_real_t proportional = McMath_Mul(state->kp, error);
    mc_real_t derivative = McMath_Mul(
        state->kd_step, McMath_Sub(error, state->last_error));
    mc_real_t candidate = McMath_Add(
        McMath_Add(proportional, state->integral), derivative);

    /* Conditional integration is the same anti-windup behavior used by the
     * floating reference controller: integrate only while output is in range. */
    if (candidate >= state->minimum && candidate <= state->maximum)
    {
        state->integral = McMath_Clamp(
            McMath_Add(state->integral,
                       McMath_Mul(state->ki_step, error)),
            McMath_Neg(state->integral_limit),
            state->integral_limit);
    }
    state->last_error = error;
    state->output = McMath_Clamp(
        McMath_Add(McMath_Add(proportional, state->integral), derivative),
        state->minimum,
        state->maximum);
    return state->output;
}

void SpeedPidScheduleIq_Init(SpeedPidScheduleIqState_t* state)
{
    SpeedPidScheduleIq_Reset(state);
}

void SpeedPidScheduleIq_Reset(SpeedPidScheduleIqState_t* state)
{
    if (state == NULL)
        return;
    *state = (SpeedPidScheduleIqState_t){
        .kp_state = SPEED_PID_KP_LOW,
        .kp_multiplier = MC_ONE,
        .ki_multiplier = MC_ONE};
}

static mc_real_t schedule_interpolate(mc_real_t value,
                                      mc_real_t start,
                                      mc_real_t end,
                                      mc_real_t maximum)
{
    mc_real_t span = McMath_Sub(end, start);
    if (span <= MC_ZERO)
        return maximum;
    mc_real_t ratio = McMath_Div(
        McMath_Sub(McMath_Clamp(value, start, end), start), span);
    return McMath_Add(MC_ONE,
                      McMath_Mul(ratio, McMath_Sub(maximum, MC_ONE)));
}

void SpeedPidScheduleIq_Run(SpeedPidScheduleIqState_t* state,
                            const SpeedPidScheduleIqConfig_t* config,
                            mc_real_t reference,
                            mc_real_t absolute_error,
                            mc_real_t* kp,
                            mc_real_t* ki_step)
{
    if (state == NULL || config == NULL || kp == NULL || ki_step == NULL)
        return;

    if (McMath_Abs(reference) <= config->enable_reference_minimum)
    {
        SpeedPidScheduleIq_Reset(state);
        *kp = config->kp_base;
        *ki_step = config->ki_step_base;
        return;
    }

    switch (state->kp_state)
    {
    case SPEED_PID_KP_LOW:
        if (absolute_error >= config->kp_up_start_error)
            state->kp_state = SPEED_PID_KP_RAMP_UP;
        break;
    case SPEED_PID_KP_RAMP_UP:
        if (absolute_error >= config->kp_up_end_error)
            state->kp_state = SPEED_PID_KP_HIGH;
        else if (absolute_error < config->kp_up_start_error)
            state->kp_state = SPEED_PID_KP_LOW;
        break;
    case SPEED_PID_KP_HIGH:
        if (absolute_error <= config->kp_down_start_error)
            state->kp_state = SPEED_PID_KP_RAMP_DOWN;
        break;
    case SPEED_PID_KP_RAMP_DOWN:
        if (absolute_error > config->kp_down_start_error)
            state->kp_state = SPEED_PID_KP_HIGH;
        else if (absolute_error <= config->kp_down_end_error)
            state->kp_state = SPEED_PID_KP_LOW;
        break;
    default:
        SpeedPidScheduleIq_Reset(state);
        break;
    }

    switch (state->kp_state)
    {
    case SPEED_PID_KP_RAMP_UP:
        state->kp_multiplier = schedule_interpolate(
            absolute_error,
            config->kp_up_start_error,
            config->kp_up_end_error,
            config->kp_multiplier_max);
        break;
    case SPEED_PID_KP_HIGH:
        state->kp_multiplier = config->kp_multiplier_max;
        break;
    case SPEED_PID_KP_RAMP_DOWN:
        state->kp_multiplier = schedule_interpolate(
            absolute_error,
            config->kp_down_end_error,
            config->kp_down_start_error,
            config->kp_multiplier_max);
        break;
    default:
        state->kp_multiplier = MC_ONE;
        break;
    }

    uint16_t confirm = config->ki_confirm_cycles == 0U
                     ? 1U : config->ki_confirm_cycles;
    if (absolute_error > config->ki_switch_error)
    {
        if (state->ki_high_count < confirm)
            state->ki_high_count++;
        state->ki_low_count = 0U;
        if (state->ki_high_count >= confirm)
            state->ki_boosted = true;
    }
    else
    {
        if (state->ki_low_count < confirm)
            state->ki_low_count++;
        state->ki_high_count = 0U;
        if (state->ki_low_count >= confirm)
            state->ki_boosted = false;
    }
    state->ki_multiplier = state->ki_boosted
                         ? config->ki_multiplier_max : MC_ONE;
    *kp = McMath_Mul(config->kp_base, state->kp_multiplier);
    *ki_step = McMath_Mul(config->ki_step_base, state->ki_multiplier);
}
