#include "protect_iq.h"

#include <stddef.h>
#include "fixed_numeric_config.h"

static ProtectIqState_t ProtectIq_CompatibilityState;

bool Protect_Initialization(const Protect_Parameter_t* parameters)
{
    if (parameters == NULL)
        return false;
    ProtectIqConfig_t config = {
        .bus_rate_pu = McMath_FromFloat(
            parameters->Udc_rate / MC_VOLTAGE_BASE_V),
        .bus_fluctuation_pu = McMath_FromFloat(
            parameters->Udc_fluctuation / MC_VOLTAGE_BASE_V),
        .bus_low_pu = MC_CONST(20.0 / MC_VOLTAGE_BASE_V),
        .current_max_pu = McMath_FromFloat(
            parameters->I_Max / MC_CURRENT_BASE_A),
        .temperature_max_pu = McMath_FromFloat(
            parameters->Temperature / MC_TEMPERATURE_BASE_C),
        .average_current_ratio = MC_CONST(0.9),
        .fan_on_ratio = MC_CONST(0.40),
        .fan_off_ratio = MC_CONST(0.36),
        .average_current_cycles = 10U};
    ProtectIq_Init(&ProtectIq_CompatibilityState, &config, parameters->Flag);
    return true;
}

bool Protect_PhaseCurrent(Phase_t current)
{
    PhaseIq_t fixed = {
        McMath_FromFloat(current.a / MC_CURRENT_BASE_A),
        McMath_FromFloat(current.b / MC_CURRENT_BASE_A),
        McMath_FromFloat(current.c / MC_CURRENT_BASE_A)};
    return ProtectIq_RunPhaseCurrent(&ProtectIq_CompatibilityState, fixed);
}

bool Protect_BusVoltage(float voltage)
{
    return ProtectIq_RunBusVoltage(&ProtectIq_CompatibilityState,
        McMath_FromFloat(voltage / MC_VOLTAGE_BASE_V));
}

void Protect_HardWareFault(bool status)
{
    ProtectIq_RunHardwareFault(&ProtectIq_CompatibilityState, status);
}

bool Protect_Temperature(float temperature)
{
    return ProtectIq_RunTemperature(&ProtectIq_CompatibilityState,
        McMath_FromFloat(temperature / MC_TEMPERATURE_BASE_C));
}

bool Protect_Get_FanState(float temperature)
{
    return ProtectIq_RunFan(&ProtectIq_CompatibilityState,
        McMath_FromFloat(temperature / MC_TEMPERATURE_BASE_C));
}

bool Protect_Validate_Flag(void)
{
    return ProtectIq_CompatibilityState.flags != No_Protect;
}

void Protect_Reset_Flag(void)
{
    ProtectIq_Reset(&ProtectIq_CompatibilityState);
}
