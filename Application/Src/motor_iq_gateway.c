#include "motor.h"

#include <stddef.h>
#include "fixed_numeric_config.h"
#include "motor_iq.h"
#include "transformation_iq.h"

/* Compatibility owner for the legacy Motor_* API.  MotorIq_* itself has no
 * hidden state and is also available to explicit-state callers. */
static MotorIqState_t MotorIq_CompatibilityState;
static MotorIqConfig_t MotorIq_CompatibilityConfig;

bool Motor_Initialization(const MotorParam_t* parameters)
{
    if (parameters == NULL || parameters->Position_Scale < 1.0F)
        return false;
    MotorIq_CompatibilityConfig = (MotorIqConfig_t){
        .speed_prescaler = 10U,
        .position_modulus = (uint32_t)parameters->Position_Scale + 1U,
        .position_offset = (uint32_t)parameters->Position_Offset,
        .pole_pairs = McMath_FromFloat(parameters->Pn),
        .speed_filter_alpha = MC_CONST(0.8819113783),
        .speed_scale = MC_ZERO};
    MotorIq_CompatibilityConfig.position_offset %=
        MotorIq_CompatibilityConfig.position_modulus;
    MotorIq_Init(&MotorIq_CompatibilityState, &MotorIq_CompatibilityConfig);
    return true;
}

bool Motor_Set_SampleTime(const SystemTimeConfig_t* time_config)
{
    if (time_config == NULL || time_config->speed.inv <= 0.0F)
        return false;
    MotorIq_CompatibilityConfig.speed_scale = McMath_FromFloat(
        time_config->speed.inv * 60.0F / MC_SPEED_BASE_RPM);
    MotorIq_CompatibilityState.config.speed_scale =
        MotorIq_CompatibilityConfig.speed_scale;
    return true;
}

bool Motor_Set_SpeedPrescaler(uint16_t prescaler)
{
    if (prescaler == 0U)
        return false;
    MotorIq_CompatibilityConfig.speed_prescaler = prescaler;
    MotorIq_CompatibilityState.config.speed_prescaler = prescaler;
    return true;
}

bool Motor_Set_Filter(float cutoff_freq, float sample_freq)
{
    if (cutoff_freq <= 0.0F || sample_freq <= 0.0F)
        return false;
    float ratio = cutoff_freq / sample_freq;
    float alpha = 1.0F / (1.0F + MC_TWO_PI_F * ratio);
    MotorIq_CompatibilityConfig.speed_filter_alpha = McMath_FromFloat(alpha);
    MotorIq_CompatibilityState.config.speed_filter_alpha =
        MotorIq_CompatibilityConfig.speed_filter_alpha;
    return true;
}

void Motor_Set_Position(uint16_t position)
{
    MotorIq_RunPosition(&MotorIq_CompatibilityState, position);
}

void Motor_Set_Theta_Elec(float theta)
{
    MotorIq_SetElectricalAngle(&MotorIq_CompatibilityState,
        McMath_FromFloat(theta * MC_INV_TWO_PI_F));
}

float Motor_Get_ThetaElec(void)
{
    return McMath_ToFloat(MotorIq_GetElectricalAngle(
        &MotorIq_CompatibilityState)) * MC_TWO_PI_F;
}

void Motor_Set_Theta_Mech(float theta)
{
    MotorIq_SetMechanicalAngle(&MotorIq_CompatibilityState,
        McMath_FromFloat(theta * MC_INV_TWO_PI_F));
}

float Motor_Get_Theta_Mech(void)
{
    return McMath_ToFloat(MotorIq_GetMechanicalAngle(
        &MotorIq_CompatibilityState)) * MC_TWO_PI_F;
}

void Motor_Set_Speed(float speed)
{
    MotorIq_SetSpeed(&MotorIq_CompatibilityState,
        McMath_FromFloat(speed / MC_SPEED_BASE_RPM));
}

float Motor_Get_Speed(void)
{
    return McMath_ToFloat(MotorIq_GetSpeed(
        &MotorIq_CompatibilityState)) * MC_SPEED_BASE_RPM;
}
