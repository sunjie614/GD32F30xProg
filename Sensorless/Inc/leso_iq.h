#ifndef LESO_IQ_H
#define LESO_IQ_H

#include <stdbool.h>
#include "filter_iq.h"
#include "pll_iq.h"
#include "transformation_iq.h"

typedef struct
{
    mc_real_t resistance_gain;
    mc_real_t voltage_gain;
    mc_real_t beta1_step;
    mc_real_t beta2_step;
    mc_real_t disturbance_to_emf;
    mc_real_t state_limit;
    mc_real_t pll_kp;
    mc_real_t pll_ki_step;
    mc_real_t speed_to_angle_step;
    mc_real_t emf_filter_alpha;
    mc_real_t slow_filter_alpha;
} LesoIqConfig_t;

typedef struct
{
    ClarkIq_t current_estimate;
    ClarkIq_t disturbance;
    ClarkIq_t emf_estimate;
    ParkIq_t emf_dq;
    ParkIq_t emf_filtered;
    ParkIq_t emf_slow_filtered;
    Iir1IqState_t emf_d_filter;
    Iir1IqState_t emf_q_filter;
    Iir1IqState_t slow_d_filter;
    Iir1IqState_t slow_q_filter;
    PllIqState_t pll;
    mc_real_t phase_error_pu;
    bool enabled;
} LesoIqState_t;

typedef struct
{
    ClarkIq_t voltage_ab_pu;
    ClarkIq_t current_ab_pu;
    bool reset;
} LesoIqInput_t;

typedef struct
{
    mc_real_t angle_pu;
    mc_real_t speed_pu;
    mc_real_t phase_error_pu;
    ClarkIq_t emf_ab_pu;
    ParkIq_t emf_dq_pu;
} LesoIqOutput_t;

void LesoIq_DefaultConfig(LesoIqConfig_t* config);
void LesoIq_Init(LesoIqState_t* state, const LesoIqConfig_t* config);
void LesoIq_Reset(LesoIqState_t* state, mc_real_t initial_angle_pu);
void LesoIq_Run(LesoIqState_t* state,
                const LesoIqConfig_t* config,
                const LesoIqInput_t* input,
                LesoIqOutput_t* output);

#endif
