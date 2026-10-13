#include <math.h>
#include <stdio.h>

#include "fixed_numeric_config.h"
#include "mc_math.h"
#include "protect.h"
#include "protect_iq.h"

typedef enum
{
    PREEMPT_NONE,
    PREEMPT_HARDWARE_FAULT,
    PREEMPT_CCP_CLEAR,
    PREEMPT_CCP_CLEAR_THEN_HARDWARE_FAULT
} ProtectionPreemption_t;

static ProtectionPreemption_t BusCheckPreemption;
static bool test_preemptible_bus_check(ProtectIqState_t* state,
                                       mc_real_t bus_voltage_pu);

/* Execute the real gateway with a deterministic interrupt/CCP interleaving
 * immediately after the real core evaluates a sample. Firmware has no test
 * hook; only this translation unit redirects the one core call. */
#define ProtectIq_RunBusVoltage test_preemptible_bus_check
#include "../Application/Src/protect_iq_gateway.c"
#undef ProtectIq_RunBusVoltage

static bool test_preemptible_bus_check(ProtectIqState_t* state,
                                       mc_real_t bus_voltage_pu)
{
    bool result = ProtectIq_RunBusVoltage(state, bus_voltage_pu);
    ProtectionPreemption_t preemption = BusCheckPreemption;
    BusCheckPreemption = PREEMPT_NONE;
    if (preemption == PREEMPT_CCP_CLEAR
        || preemption == PREEMPT_CCP_CLEAR_THEN_HARDWARE_FAULT)
        Flag = No_Protect;
    if (preemption == PREEMPT_HARDWARE_FAULT
        || preemption == PREEMPT_CCP_CLEAR_THEN_HARDWARE_FAULT)
        Protect_HardWareFault(false);
    return result;
}

/* These are the physical-value symbols exported by the IQMATH A2L gateway. */
extern volatile float Protect_BusVolRate;
extern volatile float Protect_BusVolFluc;
extern volatile float Protect_CurrentMax;
extern volatile float Protect_TempMax;
extern volatile Protect_Flag_t Flag;

static unsigned Failures;

static void check(const char* name, bool condition)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", name);
        Failures++;
    }
}

static void initialize(void)
{
    const Protect_Parameter_t parameters = {
        .Udc_rate = 580.0F,
        .Udc_fluctuation = 140.0F,
        .I_Max = 30.0F,
        .Temperature = 80.0F,
        .Flag = No_Protect};
    check("initialize", Protect_Initialization(&parameters));
}

static Phase_t phase_current(float amperes)
{
    return (Phase_t){.a = amperes};
}

static void current_parameter_writes(void)
{
    initialize();
    check("current physical initialization", Protect_CurrentMax == 30.0F);
    Protect_CurrentMax = 40.0F;
    check("40 A write changes threshold at next current call",
          !Protect_PhaseCurrent(phase_current(35.0F)));
    check("new threshold trips", Protect_PhaseCurrent(phase_current(40.1F)));
    check("real instant latch mirrored", (Flag & Over_Max_Current) != 0);
    Protect_CurrentMax = 50.0F;
    check("raising threshold preserves latched trip",
          Protect_PhaseCurrent(phase_current(0.0F)));
    Protect_Reset_Flag();
    check("reset updates latch mirror", Flag == No_Protect);
    check("50 A threshold applied", !Protect_PhaseCurrent(phase_current(40.1F)));

    initialize();
    for (unsigned count = 0U; count < 5U; ++count)
        check("five old-limit warning samples",
              !Protect_PhaseCurrent(phase_current(27.5F)));
    Protect_CurrentMax = 40.0F;
    for (unsigned count = 0U; count < 5U; ++count)
    {
        check("new-limit warning count accumulates",
              !Protect_PhaseCurrent(phase_current(36.5F)));
        check("normal samples do not clear warning count",
              !Protect_PhaseCurrent(phase_current(0.0F)));
    }
    check("parameter update preserves average count",
          Protect_PhaseCurrent(phase_current(36.5F)));
    check("real average latch mirrored", (Flag & Over_Avg_Current) != 0);
}

static void bus_parameter_writes(void)
{
    initialize();
    check("bus physical initialization", Protect_BusVolRate == 580.0F
          && Protect_BusVolFluc == 140.0F);
    Protect_BusVolRate = 400.0F;
    Protect_BusVolFluc = 100.0F;
    check("new bus upper limit equal is safe", !Protect_BusVoltage(500.0F));
    check("new bus upper limit trips", Protect_BusVoltage(500.1F));
    check("real voltage latch mirrored", (Flag & Over_Voltage) != 0);
    Protect_Reset_Flag();
    check("legacy 20 V low boundary retained", !Protect_BusVoltage(20.0F));
    check("legacy undervoltage retained", Protect_BusVoltage(19.9F));
    Protect_Reset_Flag();
    Protect_BusVolFluc = 0.0F;
    check("zero fluctuation is a valid write", !Protect_BusVoltage(400.0F)
          && Protect_BusVolFluc == 0.0F);
    check("zero fluctuation changes actual limit", Protect_BusVoltage(400.1F));
}

static void temperature_parameter_writes(void)
{
    initialize();
    check("temperature physical initialization", Protect_TempMax == 80.0F);
    Protect_TempMax = 100.0F;
    check("new temperature boundary retained", !Protect_Temperature(100.0F));
    check("new temperature threshold trips", Protect_Temperature(100.1F));
    check("real heat latch mirrored", (Flag & Over_Heat) != 0);
    Protect_Reset_Flag();
    check("new fan threshold is physical", !Protect_Get_FanState(39.0F));
    check("new fan on threshold", Protect_Get_FanState(41.0F));
    check("new fan hysteresis retained", Protect_Get_FanState(38.0F));
    check("new fan off threshold", !Protect_Get_FanState(35.0F));
}

static void invalid_parameter_writes(void)
{
    initialize();
    McMath_ResetDiagnostics();
    Protect_CurrentMax = NAN;
    check("NaN rejected without disabling current protection",
          Protect_PhaseCurrent(phase_current(30.1F))
          && Protect_CurrentMax == 30.0F);
    Protect_Reset_Flag();
    Protect_CurrentMax = 0.0F;
    check("zero current limit rejected", !Protect_PhaseCurrent(phase_current(0.0F))
          && Protect_CurrentMax == 30.0F);
    Protect_CurrentMax = 128.0F * MC_CURRENT_BASE_A;
    check("unrepresentable current limit rejected",
          !Protect_PhaseCurrent(phase_current(0.0F))
          && Protect_CurrentMax == 30.0F);
    Protect_BusVolRate = INFINITY;
    check("infinite bus limit rejected", !Protect_BusVoltage(720.0F)
          && Protect_BusVolRate == 580.0F);
    Protect_BusVolFluc = -1.0F;
    check("negative fluctuation rejected", !Protect_BusVoltage(720.0F)
          && Protect_BusVolFluc == 140.0F);
    Protect_BusVolRate = 100.0F * MC_VOLTAGE_BASE_V;
    Protect_BusVolFluc = 30.0F * MC_VOLTAGE_BASE_V;
    check("overflowing bus upper-limit sum rejected", !Protect_BusVoltage(720.0F)
          && Protect_BusVolRate == 580.0F && Protect_BusVolFluc == 140.0F);
    Protect_TempMax = -20.0F;
    check("negative temperature limit rejected", !Protect_Temperature(80.0F)
          && Protect_TempMax == 80.0F);
    check("invalid writes diagnosed once each",
          McMath_Diagnostics.invalid_input_count == 7U);
    check("invalid writes do not saturate arithmetic",
          McMath_Diagnostics.saturation_count == 0U);
    Protect_HardWareFault(false);
    check("hardware fault is visible in writable latch", (Flag & Hardware_Fault) != 0);
    Flag = No_Protect;
    check("legacy A2L write clears the real hardware latch",
          !Protect_Validate_Flag() && Flag == No_Protect);
    Protect_HardWareFault(false);
    check("ongoing hardware fault latches again on next check",
          Protect_Validate_Flag() && Flag == Hardware_Fault);
    Protect_Reset_Flag();
    check("public reset clears real flags", !Protect_Validate_Flag()
          && Flag == No_Protect);
}

static void legacy_flag_writes(void)
{
    initialize();
    Flag = (Protect_Flag_t)(Over_Avg_Current | Over_Voltage | Over_Heat);
    check("A2L combination is the authoritative latch", Protect_Validate_Flag());
    check("A2L current bit affects current result",
          Protect_PhaseCurrent(phase_current(0.0F)));
    check("A2L voltage bit affects voltage result", Protect_BusVoltage(580.0F));
    check("A2L temperature bit affects temperature result", Protect_Temperature(20.0F));
    Protect_HardWareFault(true);
    check("healthy hardware does not clear other latched faults",
          Flag == (Over_Avg_Current | Over_Voltage | Over_Heat));
    Flag = No_Protect;
    check("A2L clear is not repopulated by normal current",
          !Protect_PhaseCurrent(phase_current(0.0F)));
    check("A2L clear is not repopulated by normal voltage",
          !Protect_BusVoltage(580.0F));
    check("A2L clear is not repopulated by normal temperature",
          !Protect_Temperature(20.0F));
    check("all old bits remain cleared", !Protect_Validate_Flag());

    Flag = Hardware_Fault;
    check("unrelated hardware flag does not change current-specific return",
          !Protect_PhaseCurrent(phase_current(0.0F)));
    check("unrelated hardware flag does not change voltage-specific return",
          !Protect_BusVoltage(580.0F));
    check("unrelated hardware flag does not change temperature-specific return",
          !Protect_Temperature(20.0F));
    check("unrelated hardware latch remains set", Protect_Validate_Flag()
          && Flag == Hardware_Fault);

    Flag = No_Protect;
    check("new overcurrent latches after clear",
          Protect_PhaseCurrent(phase_current(31.0F)));
    Flag = No_Protect;
    check("ongoing overcurrent relatches after A2L clear",
          Protect_PhaseCurrent(phase_current(31.0F))
          && (Flag & Over_Max_Current) != 0);
    Flag = No_Protect;
    check("new overvoltage latches after clear", Protect_BusVoltage(721.0F));
    Flag = No_Protect;
    check("ongoing overvoltage relatches after A2L clear",
          Protect_BusVoltage(721.0F) && Flag == Over_Voltage);
    Flag = No_Protect;
    check("ongoing undervoltage relatches after A2L clear",
          Protect_BusVoltage(19.0F) && Flag == Low_Voltage);
    Flag = No_Protect;
    check("new overtemperature latches after clear", Protect_Temperature(81.0F));
    Flag = No_Protect;
    check("ongoing overtemperature relatches after A2L clear",
          Protect_Temperature(81.0F) && Flag == Over_Heat);

    initialize();
    for (unsigned count = 0U; count < 5U; ++count)
        check("warning count before A2L clear",
              !Protect_PhaseCurrent(phase_current(27.5F)));
    Flag = Hardware_Fault;
    Flag = No_Protect;
    Protect_Reset_Flag();
    for (unsigned count = 0U; count < 5U; ++count)
        check("warning count after A2L clear",
              !Protect_PhaseCurrent(phase_current(27.5F)));
    check("A2L and public reset retain average-current accumulation",
          Protect_PhaseCurrent(phase_current(27.5F))
          && Flag == Over_Avg_Current);
}

static void fault_preemption(void)
{
    initialize();
    Flag = Over_Heat;
    BusCheckPreemption = PREEMPT_HARDWARE_FAULT;
    check("overvoltage check still reports its newly detected fault",
          Protect_BusVoltage(721.0F));
    check("nested hardware fault and existing bits survive fault publication",
          Flag == (Over_Heat | Over_Voltage | Hardware_Fault));

    initialize();
    (void)Protect_BusVoltage(721.0F);
    BusCheckPreemption = PREEMPT_CCP_CLEAR;
    check("CCP clears earlier fault during a normal voltage check",
          !Protect_BusVoltage(580.0F));
    check("normal check does not revive the cleared earlier fault",
          Flag == No_Protect && !Protect_Validate_Flag());

    initialize();
    (void)Protect_BusVoltage(721.0F);
    BusCheckPreemption = PREEMPT_CCP_CLEAR_THEN_HARDWARE_FAULT;
    check("cleared voltage latch does not remain in return value",
          !Protect_BusVoltage(580.0F));
    check("new ISR fault survives while cleared voltage fault stays clear",
          Flag == Hardware_Fault && Protect_Validate_Flag());

    initialize();
    Flag = Over_Heat;
    BusCheckPreemption = PREEMPT_HARDWARE_FAULT;
    check("healthy voltage check returns no voltage fault",
          !Protect_BusVoltage(580.0F));
    check("healthy voltage check cannot erase nested hardware fault",
          Flag == (Over_Heat | Hardware_Fault));

    initialize();
    Flag = Over_Heat;
    BusCheckPreemption = PREEMPT_CCP_CLEAR;
    check("new abnormal sample relatches after concurrent CCP clear",
          Protect_BusVoltage(721.0F));
    check("only newly detected voltage fault is relatch after clear",
          Flag == Over_Voltage);
}

int main(void)
{
    current_parameter_writes();
    bus_parameter_writes();
    temperature_parameter_writes();
    invalid_parameter_writes();
    legacy_flag_writes();
    fault_preemption();
    if (Failures != 0U)
        return 1;
    puts("PASS: legacy protection parameters/writable Flag and fault interleavings");
    return 0;
}
