#include "signal_iq.h"

#include <stddef.h>

static mc_real_t phase_wrap(mc_real_t phase)
{
    while (phase >= MC_ONE)
        phase = McMath_Sub(phase, MC_ONE);
    while (phase < MC_ZERO)
        phase = McMath_Add(phase, MC_ONE);
    return phase;
}

void RampIq_Init(RampIqState_t* state, mc_real_t initial)
{
    RampIq_Reset(state, initial);
}

void RampIq_Reset(RampIqState_t* state, mc_real_t initial)
{
    if (state != NULL)
        state->value = initial;
}

mc_real_t RampIq_Run(RampIqState_t* state,
                     mc_real_t target,
                     mc_real_t step)
{
    if (state == NULL)
        return target;
    step = McMath_Abs(step);
    mc_real_t delta = McMath_Sub(target, state->value);
    if (delta > step)
        state->value = McMath_Add(state->value, step);
    else if (delta < McMath_Neg(step))
        state->value = McMath_Sub(state->value, step);
    else
        state->value = target;
    return state->value;
}

void PhaseGeneratorIq_Init(PhaseGeneratorIqState_t* state,
                           mc_real_t initial_phase_pu)
{
    PhaseGeneratorIq_Reset(state, initial_phase_pu);
}

void PhaseGeneratorIq_Reset(PhaseGeneratorIqState_t* state,
                            mc_real_t initial_phase_pu)
{
    if (state == NULL)
        return;
    state->phase_pu = phase_wrap(initial_phase_pu);
    state->held_phase_pu = state->phase_pu;
    state->sweep_active = false;
}

mc_real_t PhaseGeneratorIq_Run(PhaseGeneratorIqState_t* state,
                               mc_real_t frequency_step_pu,
                               mc_real_t offset_pu,
                               bool sweep_request)
{
    if (state == NULL)
        return phase_wrap(offset_pu);

    mc_real_t previous = state->phase_pu;
    state->phase_pu = phase_wrap(
        McMath_Add(state->phase_pu, frequency_step_pu));
    mc_real_t wrap_delta = McMath_Abs(McMath_Sub(state->phase_pu, previous));
    if (wrap_delta > MC_HALF)
        state->sweep_active = false;
    if (sweep_request)
        state->sweep_active = true;
    if (state->sweep_active)
        state->held_phase_pu = state->phase_pu;
    return phase_wrap(McMath_Add(state->held_phase_pu, offset_pu));
}
