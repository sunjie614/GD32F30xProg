#ifndef IDENTIFICATION_IQ_H
#define IDENTIFICATION_IQ_H

#include <stdbool.h>
#include <stdint.h>
#include "mc_math.h"
#include "mtpa_iq.h"

#define IDENTIFICATION_IQ_SAMPLE_CAPACITY 512U
#define IDENTIFICATION_IQ_MAX_STEPS       20U

typedef enum
{
    IDENTIFICATION_IQ_WAIT = 0,
    IDENTIFICATION_IQ_EST_RS,
    IDENTIFICATION_IQ_INJECT_COLLECT,
    IDENTIFICATION_IQ_PROCESS,
    IDENTIFICATION_IQ_NEXT_CURRENT,
    IDENTIFICATION_IQ_LLS,
    IDENTIFICATION_IQ_PENDING,
    IDENTIFICATION_IQ_DONE,
    IDENTIFICATION_IQ_FAILED
} IdentificationIqState_e;

typedef enum
{
    IDENTIFICATION_IQ_INJECT_D = 0,
    IDENTIFICATION_IQ_INJECT_Q,
    IDENTIFICATION_IQ_INJECT_DQ
} IdentificationIqInjectionMode_e;

typedef enum
{
    IDENTIFICATION_IQ_NO_ERROR = 0,
    IDENTIFICATION_IQ_INVALID_CONFIG,
    IDENTIFICATION_IQ_SAMPLE_OVERFLOW,
    IDENTIFICATION_IQ_RS_NOT_CONVERGED,
    IDENTIFICATION_IQ_SINGULAR_LLS
} IdentificationIqError_e;

typedef struct
{
    uint16_t sample_capacity;
    uint16_t repeat_times;
    uint16_t max_steps;
    uint16_t wait_edges;
    uint16_t injection_half_period_cycles;
    uint32_t rs_hold_cycles;
    mc_real_t current_start_pu;
    mc_real_t current_final_pu;
    mc_real_t current_step_pu;
    mc_real_t current_limit_pu;
    mc_real_t rs_current_target_pu;
    mc_real_t rs_threshold_pu;
    mc_real_t rs_voltage_step_pu;
    mc_real_t injection_voltage_pu;
    mc_real_t voltage_to_flux_step;
} IdentificationIqConfig_t;

typedef struct
{
    mc_real_t current_pu;
    mc_real_t maximum_flux_pu;
    bool valid;
} IdentificationIqPoint_t;

typedef struct
{
    mc_real_t ad0;
    mc_real_t add;
    mc_real_t aq0;
    mc_real_t aqq;
    mc_real_t adq;
    mc_real_t ssr_d;
    mc_real_t ssr_q;
    mc_real_t r2_d;
    mc_real_t r2_q;
    bool valid;
} IdentificationIqCoefficients_t;

typedef struct
{
    IdentificationIqState_e state;
    IdentificationIqInjectionMode_e injection_mode;
    IdentificationIqError_e error;
    IdentificationIqConfig_t config;
    IdentificationIqCoefficients_t coefficients;
    IdentificationIqPoint_t d_results[IDENTIFICATION_IQ_MAX_STEPS];
    IdentificationIqPoint_t q_results[IDENTIFICATION_IQ_MAX_STEPS];
    mc_real_t voltage_d[IDENTIFICATION_IQ_SAMPLE_CAPACITY];
    mc_real_t voltage_q[IDENTIFICATION_IQ_SAMPLE_CAPACITY];
    mc_real_t current_d[IDENTIFICATION_IQ_SAMPLE_CAPACITY];
    mc_real_t current_q[IDENTIFICATION_IQ_SAMPLE_CAPACITY];
    mc_real_t flux_d[IDENTIFICATION_IQ_SAMPLE_CAPACITY];
    mc_real_t flux_q[IDENTIFICATION_IQ_SAMPLE_CAPACITY];
    mc_real_t resistance_pu;
    mc_real_t rs_voltage_pu;
    mc_real_t rs_previous_pu;
    mc_real_t rs_previous_voltage_pu;
    mc_real_t rs_previous_current_pu;
    mc_real_t rs_current_filtered_pu;
    mc_real_t current_target_pu;
    mc_real_t accumulated_flux;
    mc_real_t accumulated_current;
    uint32_t rs_hold_count;
    uint16_t rs_estimation_step;
    uint32_t cycle_count;
    uint16_t sample_count;
    uint16_t edge_count;
    uint16_t repeat_count;
    uint16_t step_index;
    mc_accum_t dq_sum_xx;
    mc_accum_t dq_sum_xy;
    mc_accum_t dq_sum_id;
    mc_accum_t dq_sum_id_squared;
    mc_accum_t dq_sum_iq;
    mc_accum_t dq_sum_iq_squared;
    mc_accum_t dq_sum_residual_d_squared;
    mc_accum_t dq_sum_residual_q_squared;
    uint32_t dq_sample_count;
    bool single_axis_fit_complete;
    bool rs_first_sample;
    bool injection_positive;
    bool injection_d_positive;
    bool injection_q_positive;
    bool start_requested;
} IdentificationIqState_t;

typedef struct
{
    mc_real_t current_d_pu;
    mc_real_t current_q_pu;
    bool reset;
} IdentificationIqInput_t;

typedef struct
{
    mc_real_t voltage_d_pu;
    mc_real_t voltage_q_pu;
    bool running;
    bool complete;
    bool valid;
} IdentificationIqOutput_t;

void IdentificationIq_DefaultConfig(IdentificationIqConfig_t* config);
void IdentificationIq_Init(IdentificationIqState_t* state,
                           const IdentificationIqConfig_t* config);
void IdentificationIq_Reset(IdentificationIqState_t* state);
void IdentificationIq_Start(IdentificationIqState_t* state);
void IdentificationIq_Run(IdentificationIqState_t* state,
                           const IdentificationIqInput_t* input,
                           IdentificationIqOutput_t* output);
bool IdentificationIq_GetMtpaParameters(
    const IdentificationIqState_t* state,
    MtpaIqParameters_t* parameters);

#endif
