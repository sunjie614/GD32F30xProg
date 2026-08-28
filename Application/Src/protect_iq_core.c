#include "protect_iq.h"

#include <stddef.h>

static void protect_iq_set_flag(ProtectIqState_t* state,
                                Protect_Flag_t flag)
{
    (void)__atomic_fetch_or(&state->flags, (uint32_t)flag, __ATOMIC_RELAXED);
}

void ProtectIq_Init(ProtectIqState_t* state,
                    const ProtectIqConfig_t* config,
                    Protect_Flag_t initial_flags)
{
    if (state == NULL || config == NULL)
        return;
    *state = (ProtectIqState_t){0};
    state->config = *config;
    state->flags = (uint32_t)initial_flags;
}

void ProtectIq_Reset(ProtectIqState_t* state)
{
    if (state == NULL)
        return;
    __atomic_store_n(&state->flags, (uint32_t)No_Protect,
                     __ATOMIC_RELAXED);
    state->average_overcurrent_count = 0U;
}

bool ProtectIq_RunPhaseCurrent(ProtectIqState_t* state,
                               PhaseIq_t current_abc_pu)
{
    if (state == NULL)
        return true;
    mc_real_t warning = McMath_Mul(state->config.average_current_ratio,
                                   state->config.current_max_pu);
    mc_real_t a = McMath_Abs(current_abc_pu.a);
    mc_real_t b = McMath_Abs(current_abc_pu.b);
    mc_real_t c = McMath_Abs(current_abc_pu.c);
    if (a > warning || b > warning || c > warning)
    {
        state->average_overcurrent_count++;
        if (state->average_overcurrent_count
            > state->config.average_current_cycles)
        {
            protect_iq_set_flag(state, Over_Avg_Current);
            state->average_overcurrent_count = 0U;
        }
    }
    else
        state->average_overcurrent_count = 0U;
    if (a > state->config.current_max_pu
        || b > state->config.current_max_pu
        || c > state->config.current_max_pu)
        protect_iq_set_flag(state, Over_Max_Current);
    return (state->flags & (Over_Avg_Current | Over_Max_Current)) != 0;
}

bool ProtectIq_RunBusVoltage(ProtectIqState_t* state,
                             mc_real_t bus_voltage_pu)
{
    if (state == NULL)
        return true;
    if (bus_voltage_pu > McMath_Add(state->config.bus_rate_pu,
                                    state->config.bus_fluctuation_pu))
        protect_iq_set_flag(state, Over_Voltage);
    if (bus_voltage_pu < state->config.bus_low_pu)
        protect_iq_set_flag(state, Low_Voltage);
    return (state->flags & (Over_Voltage | Low_Voltage)) != 0;
}

bool ProtectIq_RunTemperature(ProtectIqState_t* state,
                              mc_real_t temperature_pu)
{
    if (state == NULL)
        return true;
    if (temperature_pu > state->config.temperature_max_pu)
        protect_iq_set_flag(state, Over_Heat);
    return (state->flags & Over_Heat) != 0;
}

void ProtectIq_RunHardwareFault(ProtectIqState_t* state, bool healthy)
{
    if (state != NULL && !healthy)
        protect_iq_set_flag(state, Hardware_Fault);
}

bool ProtectIq_RunFan(ProtectIqState_t* state, mc_real_t temperature_pu)
{
    if (state == NULL)
        return false;
    if (temperature_pu > McMath_Mul(state->config.fan_on_ratio,
                                    state->config.temperature_max_pu))
        state->fan = true;
    else if (temperature_pu < McMath_Mul(state->config.fan_off_ratio,
                                         state->config.temperature_max_pu))
        state->fan = false;
    return state->fan;
}
