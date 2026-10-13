#ifndef LESO_IQ_H
#define LESO_IQ_H

#include <stdbool.h>
#include "filter_iq.h"
#include "pll_iq.h"
#include "transformation_iq.h"

/* IQ observer safety envelope; the retained FLOAT_REF default is unchanged. */
#define LESO_IQ_WC_MAX_RAD_S (3990U)

/* Wc*Ts hard ceiling, rounded down in Q24 (never above 3990 rad/s). */
mc_real_t LesoIq_MaximumBandwidthStep(void);

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
    /* Legacy observer coefficients: bandwidth is Wc*Ts, inductance is
     * Lq*current_base/flux_base, Rs is Rs*current_base/voltage_base. */
    mc_real_t resistance_pu;
    mc_real_t voltage_to_flux_step;
    mc_real_t inductance_q_pu;
    mc_real_t minimum_inductance_pu;
    mc_real_t bandwidth_gain, bandwidth_factor;
    mc_real_t bandwidth_minimum_step, bandwidth_maximum_step;
    mc_real_t speed_to_electrical_step;
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
    mc_real_t bandwidth_step;
    ClarkIq_t last_voltage_ab_pu, last_current_ab_pu;
    mc_real_t last_inductance_q_pu; /* effective Lq after the legacy minimum */
} LesoIqState_t;

typedef struct
{
    ClarkIq_t voltage_ab_pu;
    ClarkIq_t current_ab_pu;
    bool reset;
    mc_real_t angle_pu, speed_pu, inductance_q_pu;
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
