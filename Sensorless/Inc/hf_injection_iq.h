#ifndef HF_INJECTION_IQ_H
#define HF_INJECTION_IQ_H

#include <stdbool.h>
#include "filter_iq.h"
#include "pll_iq.h"
#include "transformation_iq.h"

typedef struct
{
    mc_real_t injection_voltage_pu;
    mc_real_t pll_kp;
    mc_real_t pll_ki_step;
    mc_real_t speed_to_angle_step;
    mc_real_t response_filter_alpha;
} HfiIqConfig_t;

typedef struct
{
    ClarkIq_t previous_current;
    ClarkIq_t previous_high_frequency;
    ClarkIq_t base_current;
    ClarkIq_t response_current;
    Iir1IqState_t response_a_filter;
    Iir1IqState_t response_b_filter;
    PllIqState_t pll;
    mc_real_t phase_error_pu;
    bool injection_positive;
    bool enabled;
} HfiIqState_t;

typedef struct
{
    ClarkIq_t current_ab_pu;
    bool reset;
} HfiIqInput_t;

typedef struct
{
    ClarkIq_t filtered_current_ab_pu;
    ParkIq_t injection_voltage_dq_pu;
    mc_real_t angle_pu;
    mc_real_t speed_pu;
    mc_real_t phase_error_pu;
} HfiIqOutput_t;

void HfiIq_DefaultConfig(HfiIqConfig_t* config);
void HfiIq_Init(HfiIqState_t* state, const HfiIqConfig_t* config);
void HfiIq_Reset(HfiIqState_t* state, mc_real_t initial_angle_pu);
void HfiIq_Run(HfiIqState_t* state,
               const HfiIqConfig_t* config,
               const HfiIqInput_t* input,
               HfiIqOutput_t* output);

#endif
