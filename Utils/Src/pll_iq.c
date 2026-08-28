#include "pll_iq.h"

#include <stddef.h>

void PllIq_Init(PllIqState_t* state,
                mc_real_t kp,
                mc_real_t ki_step,
                mc_real_t speed_minimum_pu,
                mc_real_t speed_maximum_pu,
                mc_real_t speed_to_angle_step)
{
    if (state == NULL)
        return;
    *state = (PllIqState_t){.speed_to_angle_step = speed_to_angle_step};
    PidIq_Init(&state->pid, kp, ki_step, MC_ZERO,
               speed_minimum_pu, speed_maximum_pu,
               McMath_Abs(speed_maximum_pu));
}

void PllIq_Reset(PllIqState_t* state, mc_real_t initial_angle_pu)
{
    if (state == NULL)
        return;
    PidIq_Reset(&state->pid);
    state->angle_pu = AngleIq_WrapPu(initial_angle_pu);
    state->speed_pu = MC_ZERO;
}

void PllIq_SetEnabled(PllIqState_t* state, bool enabled)
{
    if (state == NULL)
        return;
    state->enabled = enabled;
    if (!enabled)
        PllIq_Reset(state, state->angle_pu);
}

mc_real_t PllIq_Run(PllIqState_t* state, mc_real_t phase_error_pu)
{
    if (state == NULL || !state->enabled)
        return state == NULL ? MC_ZERO : state->angle_pu;
    state->speed_pu = PidIq_Run(&state->pid, phase_error_pu);
    state->angle_pu = AngleIq_WrapPu(McMath_Add(
        state->angle_pu,
        McMath_Mul(state->speed_pu, state->speed_to_angle_step)));
    return state->angle_pu;
}
