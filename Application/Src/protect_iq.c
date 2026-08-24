#include "protect.h"

#include <stddef.h>
#include "mc_math.h"

#define PROTECT_VOLTAGE_BASE 800.0F
#define PROTECT_CURRENT_BASE 30.0F
#define PROTECT_TEMPERATURE_BASE 200.0F

typedef struct
{
    mc_real_t bus_rate;
    mc_real_t bus_fluctuation;
    mc_real_t current_max;
    mc_real_t temperature_max;
    Protect_Flag_t flags;
    uint16_t average_overcurrent_count;
    bool fan;
} ProtectFixedContext_t;

static ProtectFixedContext_t ProtectFixed_Context;

bool Protect_Initialization(const Protect_Parameter_t* parameters)
{
    if (parameters == NULL) return false;
    ProtectFixed_Context = (ProtectFixedContext_t){0};
    ProtectFixed_Context.bus_rate = McMath_FromFloat(
        parameters->Udc_rate / PROTECT_VOLTAGE_BASE);
    ProtectFixed_Context.bus_fluctuation = McMath_FromFloat(
        parameters->Udc_fluctuation / PROTECT_VOLTAGE_BASE);
    ProtectFixed_Context.current_max = McMath_FromFloat(
        parameters->I_Max / PROTECT_CURRENT_BASE);
    ProtectFixed_Context.temperature_max = McMath_FromFloat(
        parameters->Temperature / PROTECT_TEMPERATURE_BASE);
    ProtectFixed_Context.flags = parameters->Flag;
    return true;
}

bool Protect_PhaseCurrent(Phase_t current)
{
    mc_real_t a = McMath_Abs(McMath_FromFloat(current.a / PROTECT_CURRENT_BASE));
    mc_real_t b = McMath_Abs(McMath_FromFloat(current.b / PROTECT_CURRENT_BASE));
    mc_real_t c = McMath_Abs(McMath_FromFloat(current.c / PROTECT_CURRENT_BASE));
    mc_real_t warning = McMath_Mul(MC_CONST(0.9), ProtectFixed_Context.current_max);
    if (a > warning || b > warning || c > warning)
    {
        if (++ProtectFixed_Context.average_overcurrent_count > 10U)
        {
            ProtectFixed_Context.flags |= Over_Avg_Current;
            ProtectFixed_Context.average_overcurrent_count = 0U;
        }
    }
    else
    {
        ProtectFixed_Context.average_overcurrent_count = 0U;
    }
    if (a > ProtectFixed_Context.current_max
        || b > ProtectFixed_Context.current_max
        || c > ProtectFixed_Context.current_max)
        ProtectFixed_Context.flags |= Over_Max_Current;
    return (ProtectFixed_Context.flags
            & (Over_Avg_Current | Over_Max_Current)) != 0;
}

bool Protect_BusVoltage(float voltage)
{
    mc_real_t value = McMath_FromFloat(voltage / PROTECT_VOLTAGE_BASE);
    if (value > McMath_Add(ProtectFixed_Context.bus_rate,
                           ProtectFixed_Context.bus_fluctuation))
        ProtectFixed_Context.flags |= Over_Voltage;
    if (value < McMath_FromFloat(20.0F / PROTECT_VOLTAGE_BASE))
        ProtectFixed_Context.flags |= Low_Voltage;
    return (ProtectFixed_Context.flags & (Over_Voltage | Low_Voltage)) != 0;
}

void Protect_HardWareFault(bool status)
{
    if (!status) ProtectFixed_Context.flags |= Hardware_Fault;
}

bool Protect_Temperature(float temperature)
{
    if (McMath_FromFloat(temperature / PROTECT_TEMPERATURE_BASE)
        > ProtectFixed_Context.temperature_max)
        ProtectFixed_Context.flags |= Over_Heat;
    return (ProtectFixed_Context.flags & Over_Heat) != 0;
}

bool Protect_Get_FanState(float temperature)
{
    mc_real_t value = McMath_FromFloat(temperature / PROTECT_TEMPERATURE_BASE);
    if (value > McMath_Mul(MC_CONST(0.40), ProtectFixed_Context.temperature_max))
        ProtectFixed_Context.fan = true;
    else if (value < McMath_Mul(MC_CONST(0.36), ProtectFixed_Context.temperature_max))
        ProtectFixed_Context.fan = false;
    return ProtectFixed_Context.fan;
}

bool Protect_Validate_Flag(void)
{
    return ProtectFixed_Context.flags != No_Protect;
}

void Protect_Reset_Flag(void)
{
    ProtectFixed_Context.flags = No_Protect;
    ProtectFixed_Context.average_overcurrent_count = 0U;
}
