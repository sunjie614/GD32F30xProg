#ifndef FILTER_IQ_H
#define FILTER_IQ_H

#include <stdbool.h>
#include "mc_math.h"

typedef struct
{
    mc_real_t alpha;
    mc_real_t previous_output;
    bool initialized;
} Iir1IqState_t;

typedef struct
{
    mc_real_t b0;
    mc_real_t b1;
    mc_real_t b2;
    mc_real_t a1;
    mc_real_t a2;
    mc_real_t x1;
    mc_real_t x2;
    mc_real_t y1;
    mc_real_t y2;
    bool initialized;
} BiquadIqState_t;

void Iir1Iq_Init(Iir1IqState_t* state, mc_real_t alpha);
void Iir1Iq_Reset(Iir1IqState_t* state);
mc_real_t Iir1Iq_Run(Iir1IqState_t* state, mc_real_t input);

void BiquadIq_Init(BiquadIqState_t* state,
                   mc_real_t b0,
                   mc_real_t b1,
                   mc_real_t b2,
                   mc_real_t a1,
                   mc_real_t a2);
void BiquadIq_Reset(BiquadIqState_t* state);
mc_real_t BiquadIq_Run(BiquadIqState_t* state, mc_real_t input);

#endif
