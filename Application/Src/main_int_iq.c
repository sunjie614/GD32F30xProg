#include "main_int.h"

#include "buffer.h"
#include "fixed_control.h"
#include "gd32f30x.h"
#include "hardware_interface.h"
#include "parameters.h"
#include "reciprocal.h"
#include "telemetry_monitor.h"

/* Serial-only display filter: tau = 10 ms (about 16 Hz at the configured
 * 5 kHz sample rate). Its output is never passed to FixedControl_Step. */
#define TELEMETRY_SPEED_TAU_S (0.010F)

/* The interrupt integration layer owns the single top-level mutable instance.
 * Every algorithm below it receives its state explicitly. */
static FixedControlState_t MainInt_ControlState;
static TelemetryMonitorState_t MainInt_TelemetryState;
volatile uint32_t MainInt_PeriodTicks = 0U;
volatile uint32_t MainInt_CycleTicks = 0U;
volatile uint32_t MainInt_MaxCycleTicks = 0U;
volatile uint32_t MainInt_FrequencyHz = 0U;
volatile uint16_t MainInt_LoadPermille = 0U;
static uint32_t MainInt_PreviousEntryTick = 0U;

void MainInt_ControlInit(void)
{
    TelemetryMonitor_Init(&MainInt_TelemetryState,
                          MAIN_LOOP_TIME, TELEMETRY_SPEED_TAU_S);
    FixedControl_Init(&MainInt_ControlState);
}

bool MainInt_ControlReady(void)
{
    return MainInt_ControlState.initialized
        && MainInt_ControlState.foc.initialized
        && MainInt_ControlState.foc.mtpa.table_valid
        && FixedControl_SelfTest.conversion_failures == 0U
        && FixedControl_SelfTest.arithmetic_failures == 0U
        && FixedControl_SelfTest.trigonometric_failures == 0U
        && FixedControl_SelfTest.sqrt_failures == 0U;
}

void MainInt_ControlBackground(void)
{
    if (!FixedControl_BackgroundBuild(&MainInt_ControlState))
        return;
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    FixedControl_CommitBackground(&MainInt_ControlState);
    if (interrupt_state == 0U)
        __enable_irq();
}

void Main_Int_Handler(void)
{
    uint32_t entry_tick = DWT->CYCCNT;
    if (MainInt_PreviousEntryTick != 0U)
    {
        MainInt_PeriodTicks = entry_tick - MainInt_PreviousEntryTick;
        if (MainInt_PeriodTicks != 0U)
            MainInt_FrequencyHz = SystemCoreClock / MainInt_PeriodTicks;
    }
    MainInt_PreviousEntryTick = entry_tick;
    Phase_t current = Peripheral_Get_PhaseCurrent();
    AngleResult_t position = Peripheral_Update_Position();
    FloatWithInv_t bus = Peripheral_UpdateUdc();
    if (FixedControl_ModeRequestsStop())
        Peripheral_Set_Stop(true);
    bool stop = Peripheral_Update_Break();

    FixedControlInput_t input = {
        .current_abc = current,
        .theta_rad = position.theta,
        .speed_rpm = position.speed,
        .bus_voltage = bus.val,
        .reset = stop};
    FixedControlOutput_t output = FixedControl_Step(
        &MainInt_ControlState, &input);

    if (FixedControl_ModeRequestsStop())
        stop = true;
    Peripheral_Set_Stop(stop);
    if (stop)
        (void)Peripheral_Update_Break();
    Peripheral_Set_PWMChangePoint(output.pwm_duty);

    /* VOFA JustFloat serial telemetry only. Never use the display-filtered
     * speed or the Q24 round-trip diagnostics as control feedback. */
    Buffer_Put(position.theta, 0U);
    Buffer_Put(output.estimated_theta_rad, 1U);
    Buffer_Put(position.speed, 2U);
    Buffer_Put(TelemetryMonitor_FilterSpeed(
        &MainInt_TelemetryState, output.estimated_speed_rpm), 3U);
    Buffer_Put(TelemetryMonitor_AngleErrorDeg(
        position.theta, output.estimated_theta_rad), 4U);
    Buffer_Put(bus.val, 5U);
    Buffer_Put(output.bus_voltage_q24_v, 6U);
    Buffer_Put((float)output.bus_voltage_q24_raw, 7U);
    Buffer_Put(current.a, 9U);
    Buffer_Put(output.phase_a_current_q24_a, 10U);
    Buffer_Put((float)output.phase_a_current_q24_raw, 11U);
    Buffer_Put(output.voltage_dq.d, 15U);
    Buffer_Put(Foc_Id_Fdbk, 16U);
    Buffer_Put(Foc_Iq_Fdbk, 17U);
    Buffer_Send();
    MainInt_CycleTicks = DWT->CYCCNT - entry_tick;
    if (MainInt_CycleTicks > MainInt_MaxCycleTicks)
        MainInt_MaxCycleTicks = MainInt_CycleTicks;
    if (MainInt_PeriodTicks != 0U)
        MainInt_LoadPermille = (uint16_t)(
            ((uint64_t)MainInt_CycleTicks * 1000U) / MainInt_PeriodTicks);
}
