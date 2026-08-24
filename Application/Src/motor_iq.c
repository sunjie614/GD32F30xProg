#include "motor.h"

#include <stddef.h>
#include "mc_math.h"
#include "parameters.h"

#define MOTOR_DEFAULT_PRESCALER 10U
#define MOTOR_SPEED_BASE_RPM 1800.0F
#define MOTOR_TWO_PI 6.28318530717958647692F

typedef struct
{
    bool initialized;
    uint16_t speed_prescaler;
    uint16_t speed_counter;
    uint32_t position_modulus;
    uint32_t position_offset;
    mc_real_t pole_pairs;
    mc_real_t theta_mechanical_pu;
    mc_real_t theta_electrical_pu;
    mc_real_t last_theta_mechanical_pu;
    mc_real_t speed_pu;
    mc_real_t speed_filter_alpha;
    mc_real_t speed_scale;
} MotorFixedContext_t;

static MotorFixedContext_t MotorFixed_Context;

static mc_real_t motor_ratio_u32(uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0U)
    {
        McMath_Diagnostics.divide_by_zero_count++;
        return MC_ZERO;
    }
    return (mc_real_t)(((int64_t)numerator << 24) / denominator);
}

static mc_real_t motor_wrap_pu(mc_real_t angle)
{
    while (angle >= MC_ONE) angle = McMath_Sub(angle, MC_ONE);
    while (angle < MC_ZERO) angle = McMath_Add(angle, MC_ONE);
    return angle;
}

bool Motor_Set_SampleTime(const SystemTimeConfig_t* time_config)
{
    if (time_config == NULL || time_config->speed.inv <= 0.0F)
        return false;
    MotorFixed_Context.speed_scale = McMath_FromFloat(
        time_config->speed.inv * 60.0F / MOTOR_SPEED_BASE_RPM);
    return true;
}

bool Motor_Initialization(const MotorParam_t* parameters)
{
    if (parameters == NULL || parameters->Position_Scale < 1.0F)
        return false;
    MotorFixed_Context = (MotorFixedContext_t){0};
    MotorFixed_Context.position_modulus = (uint32_t)parameters->Position_Scale + 1U;
    MotorFixed_Context.position_offset = (uint32_t)parameters->Position_Offset
                                      % MotorFixed_Context.position_modulus;
    MotorFixed_Context.pole_pairs = McMath_FromFloat(parameters->Pn);
    MotorFixed_Context.speed_prescaler = MOTOR_DEFAULT_PRESCALER;
    MotorFixed_Context.speed_filter_alpha = MC_CONST(0.8819113783);
    MotorFixed_Context.initialized = true;
    return true;
}

bool Motor_Set_SpeedPrescaler(uint16_t prescaler)
{
    if (prescaler == 0U)
    {
        MotorFixed_Context.speed_prescaler = MOTOR_DEFAULT_PRESCALER;
        return false;
    }
    MotorFixed_Context.speed_prescaler = prescaler;
    return true;
}

bool Motor_Set_Filter(float cutoff_freq, float sample_freq)
{
    if (cutoff_freq <= 0.0F || sample_freq <= 0.0F)
        return false;
    /* Coefficient generation is an initialization/A2L boundary operation. */
    float ratio = cutoff_freq / sample_freq;
    float alpha = 1.0F / (1.0F + 6.283185307179586F * ratio);
    MotorFixed_Context.speed_filter_alpha = McMath_FromFloat(alpha);
    return true;
}

void Motor_Set_Position(uint16_t position)
{
    if (!MotorFixed_Context.initialized) return;
    uint32_t normalized = ((uint32_t)position
                         + MotorFixed_Context.position_modulus
                         - MotorFixed_Context.position_offset)
                        % MotorFixed_Context.position_modulus;
    MotorFixed_Context.theta_mechanical_pu = motor_ratio_u32(
        normalized, MotorFixed_Context.position_modulus);
    MotorFixed_Context.theta_electrical_pu = motor_wrap_pu(McMath_Mul(
        MotorFixed_Context.theta_mechanical_pu,
        MotorFixed_Context.pole_pairs));

    MotorFixed_Context.speed_counter++;
    if (MotorFixed_Context.speed_counter >= MotorFixed_Context.speed_prescaler)
    {
        MotorFixed_Context.speed_counter = 0U;
        mc_real_t delta = McMath_Sub(MotorFixed_Context.theta_mechanical_pu,
                                     MotorFixed_Context.last_theta_mechanical_pu);
        if (delta > MC_HALF) delta = McMath_Sub(delta, MC_ONE);
        else if (delta < -MC_HALF) delta = McMath_Add(delta, MC_ONE);
        mc_real_t raw_speed = McMath_Mul(delta, MotorFixed_Context.speed_scale);
        MotorFixed_Context.speed_pu = McMath_Add(
            McMath_Mul(MotorFixed_Context.speed_filter_alpha,
                       MotorFixed_Context.speed_pu),
            McMath_Mul(McMath_Sub(MC_ONE,
                                  MotorFixed_Context.speed_filter_alpha),
                       raw_speed));
        MotorFixed_Context.last_theta_mechanical_pu =
            MotorFixed_Context.theta_mechanical_pu;
    }
}

void Motor_Set_Theta_Elec(float theta)
{
    MotorFixed_Context.theta_electrical_pu = motor_wrap_pu(
        McMath_FromFloat(theta / MOTOR_TWO_PI));
}

float Motor_Get_ThetaElec(void)
{
    return McMath_ToFloat(MotorFixed_Context.theta_electrical_pu) * MOTOR_TWO_PI;
}

void Motor_Set_Theta_Mech(float theta)
{
    MotorFixed_Context.theta_mechanical_pu = motor_wrap_pu(
        McMath_FromFloat(theta / MOTOR_TWO_PI));
}

float Motor_Get_Theta_Mech(void)
{
    return McMath_ToFloat(MotorFixed_Context.theta_mechanical_pu) * MOTOR_TWO_PI;
}

void Motor_Set_Speed(float speed)
{
    MotorFixed_Context.speed_pu = McMath_FromFloat(speed / MOTOR_SPEED_BASE_RPM);
}

float Motor_Get_Speed(void)
{
    return McMath_ToFloat(MotorFixed_Context.speed_pu) * MOTOR_SPEED_BASE_RPM;
}
