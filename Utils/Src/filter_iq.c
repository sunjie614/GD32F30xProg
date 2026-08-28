#include "filter_iq.h"

#include <stddef.h>

void Iir1Iq_Init(Iir1IqState_t* state, mc_real_t alpha)
{
    if (state == NULL)
        return;
    *state = (Iir1IqState_t){.alpha = McMath_Clamp(alpha, MC_ZERO, MC_ONE)};
}

void Iir1Iq_Reset(Iir1IqState_t* state)
{
    if (state == NULL)
        return;
    state->previous_output = MC_ZERO;
    state->initialized = false;
}

mc_real_t Iir1Iq_Run(Iir1IqState_t* state, mc_real_t input)
{
    if (state == NULL)
        return input;
    if (!state->initialized)
    {
        state->previous_output = input;
        state->initialized = true;
        return input;
    }
    state->previous_output = McMath_Add(
        McMath_Mul(state->alpha, state->previous_output),
        McMath_Mul(McMath_Sub(MC_ONE, state->alpha), input));
    return state->previous_output;
}

void BiquadIq_Init(BiquadIqState_t* state,
                   mc_real_t b0,
                   mc_real_t b1,
                   mc_real_t b2,
                   mc_real_t a1,
                   mc_real_t a2)
{
    if (state == NULL)
        return;
    *state = (BiquadIqState_t){
        .b0 = b0, .b1 = b1, .b2 = b2, .a1 = a1, .a2 = a2,
        .initialized = true};
}

void BiquadIq_Reset(BiquadIqState_t* state)
{
    if (state == NULL)
        return;
    state->x1 = MC_ZERO;
    state->x2 = MC_ZERO;
    state->y1 = MC_ZERO;
    state->y2 = MC_ZERO;
}

mc_real_t BiquadIq_Run(BiquadIqState_t* state, mc_real_t input)
{
    if (state == NULL || !state->initialized)
        return input;
    mc_real_t output = McMath_Sub(
        McMath_Add(McMath_Add(McMath_Mul(state->b0, input),
                              McMath_Mul(state->b1, state->x1)),
                   McMath_Mul(state->b2, state->x2)),
        McMath_Add(McMath_Mul(state->a1, state->y1),
                   McMath_Mul(state->a2, state->y2)));
    state->x2 = state->x1;
    state->x1 = input;
    state->y2 = state->y1;
    state->y1 = output;
    return output;
}
