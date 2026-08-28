#ifndef SIGNAL_IQ_H
#define SIGNAL_IQ_H

#include <stdbool.h>
#include "mc_math.h"

typedef struct
{
    mc_real_t value;
} RampIqState_t;

typedef struct
{
    mc_real_t phase_pu;
    mc_real_t held_phase_pu;
    bool sweep_active;
} PhaseGeneratorIqState_t;

void RampIq_Init(RampIqState_t* state, mc_real_t initial);
void RampIq_Reset(RampIqState_t* state, mc_real_t initial);
mc_real_t RampIq_Run(RampIqState_t* state,
                     mc_real_t target,
                     mc_real_t step);

void PhaseGeneratorIq_Init(PhaseGeneratorIqState_t* state,
                           mc_real_t initial_phase_pu);
void PhaseGeneratorIq_Reset(PhaseGeneratorIqState_t* state,
                            mc_real_t initial_phase_pu);
mc_real_t PhaseGeneratorIq_Run(PhaseGeneratorIqState_t* state,
                               mc_real_t frequency_step_pu,
                               mc_real_t offset_pu,
                               bool sweep_request);

#endif
