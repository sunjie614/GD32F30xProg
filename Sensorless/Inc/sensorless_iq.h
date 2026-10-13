#ifndef SENSORLESS_IQ_H
#define SENSORLESS_IQ_H

#include <stdbool.h>
#include <stdint.h>
#include "filter_iq.h"
#include "flying_iq.h"
#include "hf_injection_iq.h"
#include "leso_iq.h"
#include "pid_iq.h"

/* The public PLL output is electrical omega, not mechanical speed.
 * Physical rad/s = omega_pu * this base. All algorithm fields are fixed. */
#define SENSORLESS_IQ_OMEGA_BASE_RAD_S (1000.0F)
typedef enum
{
    SENSORLESS_IQ_FLYING = 0,
    SENSORLESS_IQ_LES_OBSERVER = 1,
    SENSORLESS_IQ_HF_INJECTION = 2
} SensorlessIqMethod_t;

typedef struct
{
    mc_real_t kp, ki_step, kd_step;
    mc_real_t minimum, maximum, integral_limit;
    mc_real_t angle_step; /* omega_pu -> electrical revolutions per sample */
    mc_real_t speed_gain; /* omega_pu -> mechanical speed pu */
    mc_real_t theta_adjustment_pu;
    mc_real_t switch_speed_pu;
    mc_real_t threshold_hfi_pu, threshold_leso_pu;
    mc_real_t method_margin_pu;
    mc_real_t pll_wc_pu; /* rad/s / omega base */
    mc_real_t wc_speed_slope; /* speed pu -> wc pu */
    mc_real_t dynamic_speed_threshold_pu;
    mc_real_t kp_maximum, ki_step_maximum;
    mc_real_t low_speed_kp, low_speed_ki_step;
    mc_real_t wc_to_ki_step;
    mc_real_t filter_b0, filter_b1, filter_b2, filter_a1, filter_a2;
    uint16_t speed_prescaler;
} SensorlessIqConfig_t;

typedef struct
{
    PidIqState_t common_pid;
    BiquadIqState_t speed_filter;
    mc_real_t angle_pu, speed_pu, omega_pu, phase_error;
    mc_real_t pll_wc_pu;
    /* Legacy Set_SpeedRef/Target/Fdbk snapshots, assigned after position. */
    mc_real_t speed_reference_pu, speed_target_pu, speed_feedback_pu;
    mc_accum_t speed_sum;
    /* Preserve discarded fractional bits through long-running integrators.
     * Mirrors remain Q24; these are Q48 product remainders, not hidden state. */
    int64_t integral_product_residual, angle_product_residual;
    mc_real_t previous_integral_output;
    uint16_t speed_count;
    SensorlessIqMethod_t method;
    bool enabled, reset_previous;
    bool initialized;
} SensorlessIqState_t;

typedef struct
{
    bool enabled, reset;
    SensorlessIqMethod_t requested_method;
    mc_real_t real_angle_pu, measured_speed_pu;
    mc_real_t speed_target_pu, speed_reference_pu, speed_feedback_pu;
    ClarkIq_t current_ab_pu, voltage_ab_pu;
    mc_real_t inductance_q_pu; /* Lq * current base / flux base */
} SensorlessIqInput_t;

typedef struct
{
    mc_real_t angle_pu, speed_pu, omega_pu, phase_error;
    SensorlessIqMethod_t method;
    bool using_hfi;
} SensorlessIqOutput_t;

void SensorlessIq_DefaultConfig(SensorlessIqConfig_t* config);
void SensorlessIq_Init(SensorlessIqState_t* state,
                       const SensorlessIqConfig_t* config);
void SensorlessIq_Reset(SensorlessIqState_t* state,
                        mc_real_t initial_angle_pu);
ClarkIq_t SensorlessIq_FilterCurrent(SensorlessIqState_t* state,
                                      HfiIqState_t* hfi, ClarkIq_t current,
                                      bool enabled, bool reset);
void SensorlessIq_UpdatePosition(SensorlessIqState_t* state,
                                 const SensorlessIqConfig_t* config,
                                 const SensorlessIqInput_t* input,
                                 LesoIqState_t* leso, HfiIqState_t* hfi,
                                 SensorlessIqOutput_t* output);
void SensorlessIq_Calculate(SensorlessIqState_t* state,
                            const SensorlessIqConfig_t* config,
                            const SensorlessIqInput_t* input,
                            const LesoIqConfig_t* leso_config,
                            const HfiIqConfig_t* hfi_config,
                            LesoIqState_t* leso, HfiIqState_t* hfi,
                            FlyingIqState_t* flying);
ParkIq_t SensorlessIq_InjectVoltage(SensorlessIqState_t* state,
                                    const HfiIqConfig_t* config,
                                    HfiIqState_t* hfi, ParkIq_t voltage,
                                    bool reset);

#endif
