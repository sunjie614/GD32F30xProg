#ifndef FOC_IQ_H
#define FOC_IQ_H

#include <stdbool.h>
#include <stdint.h>
#include "flying_iq.h"
#include "foc_mode.h"
#include "hf_injection_iq.h"
#include "identification_iq.h"
#include "leso_iq.h"
#include "mtpa_iq.h"
#include "pid_iq.h"
#include "signal_iq.h"
#include "transformation_iq.h"

enum
{
    FOC_IQ_SENSORLESS_LESO = 1U << 0,
    FOC_IQ_SENSORLESS_HFI = 1U << 1,
    FOC_IQ_SENSORLESS_FLYING = 1U << 2
};

typedef struct
{
    uint16_t speed_prescaler;
    uint16_t startup_hold_cycles;
    uint16_t flying_startup_delay;
    mc_real_t speed_ramp_step_pu;
    mc_real_t sensorless_switch_speed_pu;
    mc_real_t sensorless_hysteresis_pu;
    mc_real_t startup_voltage_d_pu;
    SpeedPidScheduleIqConfig_t speed_schedule;
    LesoIqConfig_t leso;
    HfiIqConfig_t hfi;
} FocIqConfig_t;

/* A complete parameter snapshot is supplied once per control cycle by the
 * A2L gateway.  The algorithm never reads the gateway globals directly. */
typedef struct
{
    FocMode_t mode;
    bool reset;
    bool sweep_request;
    bool startup_prepare_request;
    bool use_real_angle;
    uint16_t sensorless_method;
    mc_real_t speed_reference_pu;
    ParkIq_t vf_voltage_reference_pu;
    mc_real_t vf_frequency_step_pu;
    mc_real_t vf_offset_pu;
    ParkIq_t if_current_reference_pu;
    mc_real_t if_frequency_step_pu;
    mc_real_t if_offset_pu;
    bool if_use_sensor;
    mc_real_t speed_kp;
    mc_real_t speed_ki_step;
    mc_real_t speed_kd_step;
    mc_real_t speed_minimum_pu;
    mc_real_t speed_maximum_pu;
    mc_real_t speed_integral_limit_pu;
    mc_real_t current_d_kp;
    mc_real_t current_d_ki_step;
    mc_real_t current_d_kd_step;
    mc_real_t current_d_minimum_pu;
    mc_real_t current_d_maximum_pu;
    mc_real_t current_d_integral_limit_pu;
    mc_real_t current_q_kp;
    mc_real_t current_q_ki_step;
    mc_real_t current_q_kd_step;
    mc_real_t current_q_minimum_pu;
    mc_real_t current_q_maximum_pu;
    mc_real_t current_q_integral_limit_pu;
} FocIqParameters_t;

typedef struct
{
    PhaseIq_t current_abc_pu;
    mc_real_t measured_angle_pu;
    mc_real_t measured_speed_pu;
    mc_real_t bus_voltage_pu;
} FocIqInput_t;

typedef struct
{
    PhaseIq_t pwm_duty;
    ClarkIq_t current_ab_pu;
    ParkIq_t current_dq_pu;
    ParkIq_t current_reference_dq_pu;
    ParkIq_t voltage_reference_dq_pu;
    ClarkIq_t voltage_reference_ab_pu;
    ParkIq_t inductance_dq_pu;
    mc_real_t control_angle_pu;
    mc_real_t speed_ramp_pu;
    mc_real_t speed_feedback_pu;
    mc_real_t estimated_angle_pu;
    mc_real_t estimated_speed_pu;
    FocMode_t requested_mode;
    IdentificationIqState_e identification_state;
    IdentificationIqError_e identification_error;
    bool startup_active;
    bool using_hfi;
} FocIqOutput_t;

typedef struct
{
    FocIqConfig_t config;
    FocMode_t mode;
    FocMode_t previous_mode;
    uint16_t speed_counter;
    uint16_t startup_count;
    bool startup_active;
    bool startup_request_latched;
    bool using_hfi;
    bool reset_previous;
    RampIqState_t speed_ramp;
    PhaseGeneratorIqState_t vf_phase;
    PhaseGeneratorIqState_t if_phase;
    PidIqState_t speed_pid;
    PidIqState_t current_d_pid;
    PidIqState_t current_q_pid;
    SpeedPidScheduleIqState_t speed_schedule;
    MtpaIqState_t mtpa;
    MtpaIqState_t mtpa_staging;
    MtpaIqParameters_t pending_mtpa_parameters;
    IdentificationIqState_t identification;
    LesoIqState_t leso;
    HfiIqState_t hfi;
    FlyingIqState_t flying;
    ClarkIq_t current_ab_pu;
    ParkIq_t current_dq_pu;
    ParkIq_t current_reference_dq_pu;
    ParkIq_t voltage_reference_dq_pu;
    ClarkIq_t voltage_reference_ab_pu;
    mc_real_t control_angle_pu;
    mc_real_t speed_feedback_pu;
    volatile bool mtpa_rebuild_pending;
    volatile bool mtpa_rebuild_in_progress;
    volatile bool mtpa_staging_ready;
    volatile bool mtpa_rebuild_failed;
    bool initialized;
} FocIqState_t;

void FocIq_DefaultConfig(FocIqConfig_t* config);
void FocIq_DefaultParameters(FocIqParameters_t* parameters);
void FocIq_Init(FocIqState_t* state, const FocIqConfig_t* config);
void FocIq_Reset(FocIqState_t* state, mc_real_t initial_angle_pu);
void FocIq_Run(FocIqState_t* state,
               const FocIqParameters_t* parameters,
               const FocIqInput_t* input,
               FocIqOutput_t* output);
bool FocIq_BackgroundBuildMtpa(FocIqState_t* state);
void FocIq_CommitBackgroundMtpa(FocIqState_t* state);

/* Familiar, explicit-state accessors retained for readable integration code. */
void FocIq_Set_Mode(FocIqState_t* state, FocMode_t mode);
FocMode_t FocIq_Get_Mode(const FocIqState_t* state);
void FocIq_Request_StartupPrepare(FocIqState_t* state);
void FocIq_Set_Angle(FocIqState_t* state, mc_real_t angle_pu);
void FocIq_Set_Speed(FocIqState_t* state, mc_real_t speed_pu);
ParkIq_t FocIq_Get_Inductor(const FocIqState_t* state);

#endif
