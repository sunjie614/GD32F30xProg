#include "protect_iq.h"

#include <math.h>
#include <stddef.h>
#include "fixed_numeric_config.h"

static ProtectIqState_t ProtectIq_CompatibilityState;

/* Legacy A2L names retain physical units only at the integration boundary.
 * The algorithm receives an explicit fixed-point configuration. */
volatile float Protect_BusVolRate = 0.0F;
volatile float Protect_BusVolFluc = 0.0F;
volatile float Protect_CurrentMax = 0.0F;
volatile float Protect_TempMax = 0.0F;
/* This is the real, legacy-writable latch, not an observation mirror.
 * CCP/A2L may clear it or write a combination of Protect_Flag_t bits. */
volatile Protect_Flag_t Flag = No_Protect;

typedef struct
{
    float bus_rate;
    float bus_fluctuation;
    float current_max;
    float temperature_max;
} ProtectIqPhysicalParameters_t;

static ProtectIqPhysicalParameters_t ProtectIq_LastApplied;

static bool protect_iq_convert_threshold(float physical, float base,
                                         bool allow_zero, mc_real_t* fixed)
{
    if (!isfinite(physical) || physical < 0.0F
        || (!allow_zero && physical == 0.0F))
        return false;
    float normalized = physical / base;
    if (!isfinite(normalized) || normalized >= 128.0F)
        return false;
    *fixed = McMath_FromFloat(normalized);
    return allow_zero || *fixed > MC_ZERO;
}

static bool protect_iq_convert_bus_limits(float rate, float fluctuation,
                                          mc_real_t* rate_fixed,
                                          mc_real_t* fluctuation_fixed)
{
    if (!protect_iq_convert_threshold(rate, MC_VOLTAGE_BASE_V, false,
                                      rate_fixed)
        || !protect_iq_convert_threshold(fluctuation, MC_VOLTAGE_BASE_V,
                                         true, fluctuation_fixed))
        return false;
#if defined(MC_NUMERIC_IQMATH)
    return (int64_t)*rate_fixed + (int64_t)*fluctuation_fixed <= INT32_MAX;
#else
    return isfinite(*rate_fixed + *fluctuation_fixed)
        && *rate_fixed + *fluctuation_fixed < 128.0F;
#endif
}

static void protect_iq_refresh_current_limit(void)
{
    float requested = Protect_CurrentMax;
    if (requested == ProtectIq_LastApplied.current_max)
        return;
    mc_real_t fixed;
    if (protect_iq_convert_threshold(requested, MC_CURRENT_BASE_A,
                                     false, &fixed))
    {
        ProtectIq_CompatibilityState.config.current_max_pu = fixed;
        ProtectIq_LastApplied.current_max = requested;
    }
    else
    {
        McMath_Diagnostics.invalid_input_count++;
        Protect_CurrentMax = ProtectIq_LastApplied.current_max;
    }
}

static void protect_iq_refresh_bus_limits(void)
{
    float rate = Protect_BusVolRate;
    float fluctuation = Protect_BusVolFluc;
    if (rate == ProtectIq_LastApplied.bus_rate
        && fluctuation == ProtectIq_LastApplied.bus_fluctuation)
        return;
    mc_real_t rate_fixed;
    mc_real_t fluctuation_fixed;
    if (protect_iq_convert_bus_limits(rate, fluctuation,
                                      &rate_fixed, &fluctuation_fixed))
    {
        ProtectIq_CompatibilityState.config.bus_rate_pu = rate_fixed;
        ProtectIq_CompatibilityState.config.bus_fluctuation_pu
            = fluctuation_fixed;
        ProtectIq_LastApplied.bus_rate = rate;
        ProtectIq_LastApplied.bus_fluctuation = fluctuation;
    }
    else
    {
        McMath_Diagnostics.invalid_input_count++;
        Protect_BusVolRate = ProtectIq_LastApplied.bus_rate;
        Protect_BusVolFluc = ProtectIq_LastApplied.bus_fluctuation;
    }
}

static void protect_iq_refresh_temperature_limit(void)
{
    float requested = Protect_TempMax;
    if (requested == ProtectIq_LastApplied.temperature_max)
        return;
    mc_real_t fixed;
    if (protect_iq_convert_threshold(requested, MC_TEMPERATURE_BASE_C,
                                     false, &fixed))
    {
        ProtectIq_CompatibilityState.config.temperature_max_pu = fixed;
        ProtectIq_LastApplied.temperature_max = requested;
    }
    else
    {
        McMath_Diagnostics.invalid_input_count++;
        Protect_TempMax = ProtectIq_LastApplied.temperature_max;
    }
}

static uint32_t protect_iq_latch_detected(uint32_t detected)
{
    /* A check contributes only newly detected faults. Never copy a private
     * latch over Flag: that could erase a preempting hardware fault, or bring
     * back a fault that CCP just cleared. Atomic OR also preserves different
     * fault bits detected by nested interrupt handlers. */
    return (uint32_t)__atomic_or_fetch(&Flag, (Protect_Flag_t)detected,
                                      __ATOMIC_RELAXED);
}

static ProtectIqState_t protect_iq_check_state(void)
{
    /* State passed into the core remains explicit. Only the gateway owns the
     * public latch; each check starts with no previously latched fault bits. */
    ProtectIqState_t check = {0};
    check.config = ProtectIq_CompatibilityState.config;
    return check;
}

bool Protect_Initialization(const Protect_Parameter_t* parameters)
{
    if (parameters == NULL)
        return false;
    mc_real_t bus_rate;
    mc_real_t bus_fluctuation;
    mc_real_t current_max;
    mc_real_t temperature_max;
    if (!protect_iq_convert_bus_limits(parameters->Udc_rate,
                                       parameters->Udc_fluctuation,
                                       &bus_rate, &bus_fluctuation)
        || !protect_iq_convert_threshold(parameters->I_Max,
                                         MC_CURRENT_BASE_A, false,
                                         &current_max)
        || !protect_iq_convert_threshold(parameters->Temperature,
                                         MC_TEMPERATURE_BASE_C, false,
                                         &temperature_max))
    {
        McMath_Diagnostics.invalid_input_count++;
        return false;
    }
    ProtectIqConfig_t config = {
        .bus_rate_pu = bus_rate,
        .bus_fluctuation_pu = bus_fluctuation,
        .bus_low_pu = MC_CONST(20.0 / MC_VOLTAGE_BASE_V),
        .current_max_pu = current_max,
        .temperature_max_pu = temperature_max,
        .average_current_ratio = MC_CONST(0.9),
        .fan_on_ratio = MC_CONST(0.40),
        .fan_off_ratio = MC_CONST(0.36),
        .average_current_cycles = 10U};
    ProtectIq_Init(&ProtectIq_CompatibilityState, &config, No_Protect);
    ProtectIq_LastApplied = (ProtectIqPhysicalParameters_t){
        .bus_rate = parameters->Udc_rate,
        .bus_fluctuation = parameters->Udc_fluctuation,
        .current_max = parameters->I_Max,
        .temperature_max = parameters->Temperature};
    Protect_BusVolRate = parameters->Udc_rate;
    Protect_BusVolFluc = parameters->Udc_fluctuation;
    Protect_CurrentMax = parameters->I_Max;
    Protect_TempMax = parameters->Temperature;
    __atomic_store_n(&Flag, parameters->Flag, __ATOMIC_RELAXED);
    return true;
}

bool Protect_PhaseCurrent(Phase_t current)
{
    protect_iq_refresh_current_limit();
    PhaseIq_t fixed = {
        McMath_FromFloat(current.a / MC_CURRENT_BASE_A),
        McMath_FromFloat(current.b / MC_CURRENT_BASE_A),
        McMath_FromFloat(current.c / MC_CURRENT_BASE_A)};
    ProtectIqState_t check = protect_iq_check_state();
    check.average_overcurrent_count
        = ProtectIq_CompatibilityState.average_overcurrent_count;
    (void)ProtectIq_RunPhaseCurrent(&check, fixed);
    ProtectIq_CompatibilityState.average_overcurrent_count
        = check.average_overcurrent_count;
    return (protect_iq_latch_detected(check.flags)
            & (Over_Avg_Current | Over_Max_Current)) != 0U;
}

bool Protect_BusVoltage(float voltage)
{
    protect_iq_refresh_bus_limits();
    ProtectIqState_t check = protect_iq_check_state();
    (void)ProtectIq_RunBusVoltage(&check,
        McMath_FromFloat(voltage / MC_VOLTAGE_BASE_V));
    return (protect_iq_latch_detected(check.flags)
            & (Over_Voltage | Low_Voltage)) != 0U;
}

void Protect_HardWareFault(bool status)
{
    ProtectIqState_t check = protect_iq_check_state();
    ProtectIq_RunHardwareFault(&check, status);
    (void)protect_iq_latch_detected(check.flags);
}

bool Protect_Temperature(float temperature)
{
    protect_iq_refresh_temperature_limit();
    ProtectIqState_t check = protect_iq_check_state();
    (void)ProtectIq_RunTemperature(&check,
        McMath_FromFloat(temperature / MC_TEMPERATURE_BASE_C));
    return (protect_iq_latch_detected(check.flags) & Over_Heat) != 0U;
}

bool Protect_Get_FanState(float temperature)
{
    protect_iq_refresh_temperature_limit();
    return ProtectIq_RunFan(&ProtectIq_CompatibilityState,
        McMath_FromFloat(temperature / MC_TEMPERATURE_BASE_C));
}

bool Protect_Validate_Flag(void)
{
    return __atomic_load_n(&Flag, __ATOMIC_RELAXED) != No_Protect;
}

void Protect_Reset_Flag(void)
{
    /* Legacy public Reset clears the latched flags, not the running
     * over-average-current sample count. ProtectIq_Reset remains the full
     * state reset for independently owned algorithm instances. */
    __atomic_store_n(&Flag, No_Protect, __ATOMIC_RELAXED);
}
