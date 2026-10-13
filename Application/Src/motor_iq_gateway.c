#include "motor.h"

#include <math.h>
#include <stddef.h>
#include "fixed_numeric_config.h"
#include "motor_iq.h"
#include "transformation_iq.h"

/* Compatibility owner for the legacy Motor_* API.  MotorIq_* itself has no
 * hidden state and is also available to explicit-state callers. */
static MotorIqState_t MotorIq_CompatibilityState;
static MotorIqConfig_t MotorIq_CompatibilityConfig;
static MotorParam_t MotorIq_CompatibilityParameters;

/* Legacy A2L registers: encoder counts, rad and rpm. Only Position_Offset
 * is a live parameter. Other motor_param fields are initialization snapshots;
 * attempted writes are read back to accepted values at the next boundary.
 * The MotorIq_* core never implicitly reads any of these gateway globals. */
volatile MotorParam_t motor_param = {0};
volatile bool Motor_Initialized = false;
volatile float Motor_Position = 0.0F;
volatile float Motor_Position_Scale = 0.0F;
volatile float Motor_Position_Offset = 0.0F;
volatile float Motor_Pn = 0.0F;
volatile float Motor_Theta_Elec = 0.0F;
volatile float Motor_Theta_Mech = 0.0F;
volatile float Motor_Speed = 0.0F;
volatile float Motor_SampleFreq = 0.0F;
volatile uint16_t Motor_Speed_Prescaler = 0U;

static bool motor_gateway_valid_offset(float offset, uint32_t modulus)
{
    /* Reject partial counts; do not silently truncate the float A2L input. */
    return isfinite(offset) && offset >= 0.0F && offset < (float)modulus
        && offset == (float)(uint32_t)offset;
}

static void motor_gateway_publish_parameters(void)
{
    motor_param = MotorIq_CompatibilityParameters;
    Motor_Initialized = MotorIq_CompatibilityState.initialized;
    Motor_Position_Scale = (float)(
        MotorIq_CompatibilityState.config.position_modulus - 1U);
    Motor_Position_Offset = (float)
        MotorIq_CompatibilityState.config.position_offset;
    Motor_Pn = McMath_ToFloat(MotorIq_CompatibilityState.config.pole_pairs);
    Motor_Speed_Prescaler =
        MotorIq_CompatibilityState.config.speed_prescaler;
}

static void motor_gateway_apply_offset(void)
{
    uint32_t old_offset = MotorIq_CompatibilityState.config.position_offset;
    uint32_t modulus = MotorIq_CompatibilityState.config.position_modulus;
    float accepted = (float)old_offset;
    float parameter_request = motor_param.Position_Offset;
    float alias_request = Motor_Position_Offset;
    /* Structured input wins if both changed. An invalid winner rejects both;
     * the two registers then report the actual accepted integer offset. */
    float requested = parameter_request != accepted
                    ? parameter_request : alias_request;
    if (requested != accepted)
    {
        if (motor_gateway_valid_offset(requested, modulus))
        {
            uint32_t new_offset = (uint32_t)requested;
            mc_real_t shift = McMath_FromFloat(
                ((float)old_offset - (float)new_offset) / (float)modulus);
            /* Rebase speed history: zero-point calibration is not movement. */
            MotorIq_CompatibilityState.last_theta_mechanical_pu =
                AngleIq_WrapPu(McMath_Add(
                    MotorIq_CompatibilityState.last_theta_mechanical_pu,
                    shift));
            MotorIq_CompatibilityState.config.position_offset = new_offset;
            MotorIq_CompatibilityConfig.position_offset = new_offset;
            MotorIq_CompatibilityParameters.Position_Offset = requested;
        }
        else
            McMath_Diagnostics.invalid_input_count++;
    }
    motor_gateway_publish_parameters();
}

bool Motor_Initialization(const MotorParam_t* parameters)
{
    if (parameters == NULL || !isfinite(parameters->Position_Scale)
        || parameters->Position_Scale < 1.0F
        || parameters->Position_Scale > (float)UINT16_MAX
        || parameters->Position_Scale != (float)(uint32_t)parameters->Position_Scale
        || !isfinite(parameters->Pn) || parameters->Pn <= 0.0F
        || parameters->Pn >= 128.0F
        || !motor_gateway_valid_offset(parameters->Position_Offset,
            (uint32_t)parameters->Position_Scale + 1U))
        return false;
    MotorIq_CompatibilityParameters = *parameters;
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
    motor_gateway_publish_parameters();
    Motor_Position = 0.0F;
    Motor_Theta_Elec = 0.0F;
    Motor_Theta_Mech = 0.0F;
    Motor_Speed = 0.0F;
    Motor_SampleFreq = 0.0F;
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
    Motor_SampleFreq = time_config->speed.inv;
    return true;
}

bool Motor_Set_SpeedPrescaler(uint16_t prescaler)
{
    if (prescaler == 0U)
        return false;
    MotorIq_CompatibilityConfig.speed_prescaler = prescaler;
    MotorIq_CompatibilityState.config.speed_prescaler = prescaler;
    Motor_Speed_Prescaler = prescaler;
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
    if (!MotorIq_CompatibilityState.initialized)
        return;
    motor_gateway_apply_offset();
    MotorIq_RunPosition(&MotorIq_CompatibilityState, position);
    Motor_Position = (float)position;
    Motor_Theta_Elec = McMath_ToFloat(MotorIq_GetElectricalAngle(
        &MotorIq_CompatibilityState)) * MC_TWO_PI_F;
    Motor_Theta_Mech = McMath_ToFloat(MotorIq_GetMechanicalAngle(
        &MotorIq_CompatibilityState)) * MC_TWO_PI_F;
    Motor_Speed = McMath_ToFloat(MotorIq_GetSpeed(
        &MotorIq_CompatibilityState)) * MC_SPEED_BASE_RPM;
}

void Motor_Set_Theta_Elec(float theta)
{
    MotorIq_SetElectricalAngle(&MotorIq_CompatibilityState,
        McMath_FromFloat(theta * MC_INV_TWO_PI_F));
    Motor_Theta_Elec = McMath_ToFloat(MotorIq_GetElectricalAngle(
        &MotorIq_CompatibilityState)) * MC_TWO_PI_F;
}

float Motor_Get_ThetaElec(void)
{
    Motor_Theta_Elec = McMath_ToFloat(MotorIq_GetElectricalAngle(
        &MotorIq_CompatibilityState)) * MC_TWO_PI_F;
    return Motor_Theta_Elec;
}

void Motor_Set_Theta_Mech(float theta)
{
    MotorIq_SetMechanicalAngle(&MotorIq_CompatibilityState,
        McMath_FromFloat(theta * MC_INV_TWO_PI_F));
    Motor_Theta_Mech = McMath_ToFloat(MotorIq_GetMechanicalAngle(
        &MotorIq_CompatibilityState)) * MC_TWO_PI_F;
}

float Motor_Get_Theta_Mech(void)
{
    Motor_Theta_Mech = McMath_ToFloat(MotorIq_GetMechanicalAngle(
        &MotorIq_CompatibilityState)) * MC_TWO_PI_F;
    return Motor_Theta_Mech;
}

void Motor_Set_Speed(float speed)
{
    MotorIq_SetSpeed(&MotorIq_CompatibilityState,
        McMath_FromFloat(speed / MC_SPEED_BASE_RPM));
    Motor_Speed = McMath_ToFloat(MotorIq_GetSpeed(
        &MotorIq_CompatibilityState)) * MC_SPEED_BASE_RPM;
}

float Motor_Get_Speed(void)
{
    Motor_Speed = McMath_ToFloat(MotorIq_GetSpeed(
        &MotorIq_CompatibilityState)) * MC_SPEED_BASE_RPM;
    return Motor_Speed;
}
