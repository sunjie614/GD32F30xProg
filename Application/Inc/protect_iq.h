#ifndef PROTECT_IQ_H
#define PROTECT_IQ_H

#include <stdbool.h>
#include <stdint.h>
#include "mc_math.h"
#include "protect.h"
#include "transformation_iq.h"

typedef struct
{
    mc_real_t bus_rate_pu;
    mc_real_t bus_fluctuation_pu;
    mc_real_t bus_low_pu;
    mc_real_t current_max_pu;
    mc_real_t temperature_max_pu;
    mc_real_t average_current_ratio;
    mc_real_t fan_on_ratio;
    mc_real_t fan_off_ratio;
    uint16_t average_current_cycles;
} ProtectIqConfig_t;

typedef struct
{
    ProtectIqConfig_t config;
    volatile uint32_t flags;
    uint16_t average_overcurrent_count;
    bool fan;
} ProtectIqState_t;

void ProtectIq_Init(ProtectIqState_t* state,
                    const ProtectIqConfig_t* config,
                    Protect_Flag_t initial_flags);
void ProtectIq_Reset(ProtectIqState_t* state);
bool ProtectIq_RunPhaseCurrent(ProtectIqState_t* state,
                               PhaseIq_t current_abc_pu);
bool ProtectIq_RunBusVoltage(ProtectIqState_t* state,
                             mc_real_t bus_voltage_pu);
bool ProtectIq_RunTemperature(ProtectIqState_t* state,
                              mc_real_t temperature_pu);
void ProtectIq_RunHardwareFault(ProtectIqState_t* state, bool healthy);
bool ProtectIq_RunFan(ProtectIqState_t* state, mc_real_t temperature_pu);

#endif
