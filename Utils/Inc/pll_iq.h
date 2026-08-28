#ifndef PLL_IQ_H
#define PLL_IQ_H

#include <stdbool.h>
#include "pid_iq.h"
#include "transformation_iq.h"

typedef struct
{
    PidIqState_t pid;
    mc_real_t angle_pu;
    mc_real_t speed_pu;
    mc_real_t speed_to_angle_step;
    bool enabled;
} PllIqState_t;

void PllIq_Init(PllIqState_t* state,
                mc_real_t kp,
                mc_real_t ki_step,
                mc_real_t speed_minimum_pu,
                mc_real_t speed_maximum_pu,
                mc_real_t speed_to_angle_step);
void PllIq_Reset(PllIqState_t* state, mc_real_t initial_angle_pu);
void PllIq_SetEnabled(PllIqState_t* state, bool enabled);
mc_real_t PllIq_Run(PllIqState_t* state, mc_real_t phase_error_pu);

#endif
