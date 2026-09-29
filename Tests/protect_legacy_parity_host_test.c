#include <stdio.h>

#include "protect.h"

static unsigned Failures;

static void check(const char* name, bool condition)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", name);
        Failures++;
    }
}

static Phase_t phase_current(float amperes)
{
    Phase_t current = {0};
    current.a = amperes;
    return current;
}

int main(void)
{
    const Protect_Parameter_t parameters = {
        .Udc_rate = 580.0F,
        .Udc_fluctuation = 140.0F,
        .I_Max = 30.0F,
        .Temperature = 80.0F,
        .Flag = No_Protect};
    check("initialize", Protect_Initialization(&parameters));
    check("initial clear", !Protect_Validate_Flag());

    /* Legacy over-average counter accumulates high samples even when
     * normal samples occur between them: the 11th high sample trips. */
    for (unsigned sample = 0U; sample < 10U; ++sample)
    {
        check("intermittent high before trip",
              !Protect_PhaseCurrent(phase_current(27.5F)));
        check("intermittent normal before trip",
              !Protect_PhaseCurrent(phase_current(0.0F)));
    }
    check("intermittent 11th high trips",
          Protect_PhaseCurrent(phase_current(27.5F)));
    check("average flag latched", Protect_Validate_Flag());
    Protect_Reset_Flag();
    check("flag reset", !Protect_Validate_Flag());

    /* Protect_Reset_Flag clears the legacy flag but not the count. */
    for (unsigned sample = 0U; sample < 5U; ++sample)
        check("pre-reset high", !Protect_PhaseCurrent(phase_current(27.5F)));
    Protect_Reset_Flag();
    for (unsigned sample = 0U; sample < 5U; ++sample)
        check("post-reset high", !Protect_PhaseCurrent(phase_current(27.5F)));
    check("count retained after flag reset",
          Protect_PhaseCurrent(phase_current(27.5F)));
    Protect_Reset_Flag();

    check("30 A not instant overcurrent",
          !Protect_PhaseCurrent(phase_current(30.0F)));
    check("above 30 A instant overcurrent",
          Protect_PhaseCurrent(phase_current(30.1F)));
    check("instant flag latched after normal sample",
          Protect_PhaseCurrent(phase_current(0.0F)));
    Protect_Reset_Flag();
    Phase_t negative_b = {0};
    negative_b.b = -30.1F;
    check("negative B phase instant overcurrent",
          Protect_PhaseCurrent(negative_b));
    Protect_Reset_Flag();

    check("720 V not overvoltage", !Protect_BusVoltage(720.0F));
    check("above 720 V overvoltage", Protect_BusVoltage(720.1F));
    Protect_Reset_Flag();
    check("20 V not undervoltage", !Protect_BusVoltage(20.0F));
    check("below 20 V undervoltage", Protect_BusVoltage(19.9F));
    Protect_Reset_Flag();

    check("80 C not overtemperature", !Protect_Temperature(80.0F));
    check("above 80 C overtemperature", Protect_Temperature(80.1F));
    Protect_Reset_Flag();
    check("fan turns on above 32 C", Protect_Get_FanState(33.0F));
    check("fan holds at 30 C", Protect_Get_FanState(30.0F));
    check("fan turns off below 28.8 C", !Protect_Get_FanState(28.0F));

    Protect_HardWareFault(true);
    check("healthy hardware does not trip", !Protect_Validate_Flag());
    Protect_HardWareFault(false);
    check("hardware fault latches", Protect_Validate_Flag());

    if (Failures != 0U)
        return 1;
#if defined(MC_NUMERIC_IQMATH)
    puts("PASS: IQMATH protection matches legacy public-interface vectors");
#else
    puts("PASS: legacy FLOAT_REF protection vectors");
#endif
    return 0;
}
