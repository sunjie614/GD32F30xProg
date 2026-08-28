#ifndef PID_IQ_H
#define PID_IQ_H

#include <stdbool.h>
#include <stdint.h>
#include "mc_math.h"

typedef struct
{
    mc_real_t kp;
    mc_real_t ki_step;
    mc_real_t kd_step;
    mc_real_t integral;
    mc_real_t last_error;
    mc_real_t minimum;
    mc_real_t maximum;
    mc_real_t integral_limit;
    mc_real_t output;
} PidIqState_t;

typedef struct
{
    mc_real_t kp_base;
    mc_real_t ki_step_base;
    mc_real_t kp_multiplier_max;
    mc_real_t ki_multiplier_max;
    mc_real_t kp_up_start_error;
    mc_real_t kp_up_end_error;
    mc_real_t kp_down_start_error;
    mc_real_t kp_down_end_error;
    mc_real_t ki_switch_error;
    mc_real_t enable_reference_minimum;
    uint16_t ki_confirm_cycles;
} SpeedPidScheduleIqConfig_t;

typedef enum
{
    SPEED_PID_KP_LOW = 0,
    SPEED_PID_KP_RAMP_UP,
    SPEED_PID_KP_HIGH,
    SPEED_PID_KP_RAMP_DOWN
} SpeedPidKpStateIq_t;

typedef struct
{
    SpeedPidKpStateIq_t kp_state;
    mc_real_t kp_multiplier;
    mc_real_t ki_multiplier;
    uint16_t ki_high_count;
    uint16_t ki_low_count;
    bool ki_boosted;
} SpeedPidScheduleIqState_t;

void PidIq_Init(PidIqState_t* state,
                mc_real_t kp,
                mc_real_t ki_step,
                mc_real_t kd_step,
                mc_real_t minimum,
                mc_real_t maximum,
                mc_real_t integral_limit);
void PidIq_Reset(PidIqState_t* state);
mc_real_t PidIq_Run(PidIqState_t* state, mc_real_t error);

void SpeedPidScheduleIq_Init(SpeedPidScheduleIqState_t* state);
void SpeedPidScheduleIq_Reset(SpeedPidScheduleIqState_t* state);
void SpeedPidScheduleIq_Run(SpeedPidScheduleIqState_t* state,
                            const SpeedPidScheduleIqConfig_t* config,
                            mc_real_t reference,
                            mc_real_t absolute_error,
                            mc_real_t* kp,
                            mc_real_t* ki_step);

#endif
