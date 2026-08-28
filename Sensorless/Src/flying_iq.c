#include "flying_iq.h"

#include <stddef.h>

void FlyingIq_Init(FlyingIqState_t* state, uint16_t startup_delay)
{
    if (state == NULL)
        return;
    *state = (FlyingIqState_t){
        .startup_delay = startup_delay == 0U ? 1U : startup_delay};
}

void FlyingIq_Reset(FlyingIqState_t* state)
{
    if (state == NULL)
        return;
    state->count = 0U;
    state->completed = false;
}

void FlyingIq_SetEnabled(FlyingIqState_t* state, bool enabled)
{
    if (state == NULL)
        return;
    if (state->enabled == enabled)
        return;
    state->enabled = enabled;
    FlyingIq_Reset(state);
}

void FlyingIq_Run(FlyingIqState_t* state, bool reset)
{
    if (state == NULL)
        return;
    if (reset || !state->enabled)
    {
        FlyingIq_Reset(state);
        return;
    }
    if (state->completed)
        return;
    if (++state->count >= state->startup_delay)
        state->completed = true;
}
