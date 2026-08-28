#ifndef MOTOR_IQ_H
#define MOTOR_IQ_H

#include <stdbool.h>
#include <stdint.h>
#include "mc_math.h"
#include "transformation_iq.h"

typedef struct
{
    uint16_t speed_prescaler;
    uint32_t position_modulus;
    uint32_t position_offset;
    mc_real_t pole_pairs;
    mc_real_t speed_filter_alpha;
    mc_real_t speed_scale;
} MotorIqConfig_t;

typedef struct
{
    MotorIqConfig_t config;
    uint16_t speed_counter;
    mc_real_t theta_mechanical_pu;
    mc_real_t theta_electrical_pu;
    mc_real_t last_theta_mechanical_pu;
    mc_real_t speed_pu;
    bool initialized;
} MotorIqState_t;

void MotorIq_Init(MotorIqState_t* state, const MotorIqConfig_t* config);
void MotorIq_Reset(MotorIqState_t* state);
void MotorIq_RunPosition(MotorIqState_t* state, uint16_t raw_position);
void MotorIq_SetElectricalAngle(MotorIqState_t* state, mc_real_t angle_pu);
void MotorIq_SetMechanicalAngle(MotorIqState_t* state, mc_real_t angle_pu);
void MotorIq_SetSpeed(MotorIqState_t* state, mc_real_t speed_pu);
mc_real_t MotorIq_GetElectricalAngle(const MotorIqState_t* state);
mc_real_t MotorIq_GetMechanicalAngle(const MotorIqState_t* state);
mc_real_t MotorIq_GetSpeed(const MotorIqState_t* state);

#endif
