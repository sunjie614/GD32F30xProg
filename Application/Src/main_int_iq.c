#include "main_int.h"

#include "Initialization.h"
#include "adc.h"
#include "buffer.h"
#include "fixed_control.h"
#include "fixed_debug_gateway.h"
#include "gd32f30x.h"
#include "hardware_interface.h"
#include "parameters.h"
#include "protect.h"
#include "reciprocal.h"
#include "telemetry_monitor.h"

/* Serial-only display filter: tau = 10 ms (about 16 Hz at the configured
 * 5 kHz sample rate). Its output is never passed to FixedControl_Step. */
#define TELEMETRY_SPEED_TAU_S (0.010F)

/* The interrupt integration layer owns the single top-level mutable instance.
 * Every algorithm below it receives its state explicitly. */
static FixedControlState_t MainInt_ControlState;
static TelemetryMonitorState_t MainInt_TelemetryState;
/* Integration-owned latch: ordinary Stop/mode/Flag writes cannot acknowledge
 * a failed INIT. ADC protection resumes even while this latch blocks PWM. */
static volatile bool MainInt_InitOutputBlocked = true;
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
    MainInt_InitOutputBlocked = !MainInt_ControlReady();
}

bool MainInt_InitBlocksOutput(void)
{
    return MainInt_InitOutputBlocked;
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

static void main_int_resume_after_init(bool ready)
{
    /* Every runtime INIT exit restores periodic sampling/protection. Publish
     * the gate and stopped mode BEFORE unmasking ADC; do not run a partly
     * initialized algorithm or permit an old PWM compare to resume. */
    Peripheral_Set_Stop(true);
    Foc_Mode = IDLE;
    MainInt_InitOutputBlocked = !ready;
    MainInt_State = ready ? RUNNING : EXIT;
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    NVIC_ClearPendingIRQ(ADC0_1_IRQn);
    MainInt_PreviousEntryTick = 0U; /* Calibration pause is not a sample period. */
    nvic_irq_enable(ADC0_1_IRQn, 3U, 0U);
}

void MainInt_ControlBackground(void)
{
    if (MainInt_State == INIT)
    {
        MainInt_InitOutputBlocked = true;
        const uint32_t fault_epoch = FixedHardwareFaultEvents;
        /* Restore the legacy INIT command: initialize the algorithm,
         * recalibrate current offsets, re-arm BRK when Udc > 200 V and clear
         * the previous protection latch. Keep this work outside the 200 us
         * ISR, with PWM disabled and CAN/fault interrupts still available. */
        Peripheral_Set_Stop(true);
        Foc_Mode = IDLE;
        TIMER_CCHP(TIMER0) &= ~((uint32_t)TIMER_CCHP_POEN);
        nvic_irq_disable(ADC0_1_IRQn);
        (void)Peripheral_Update_Break();
        TelemetryMonitor_Init(&MainInt_TelemetryState,
                              MAIN_LOOP_TIME, TELEMETRY_SPEED_TAU_S);
        FixedControl_Reinitialize(&MainInt_ControlState);
        Foc_Mode = IDLE;
        if (!MainInt_ControlReady())
        {
            FixedStartup_Status = 1U;
            main_int_resume_after_init(false);
            return;
        }
        if (!Adc_Calibrate_CurrentOffsetChecked())
        {
            FixedStartup_Status = 2U;
            main_int_resume_after_init(false);
            return; /* Preserve old faults; sampling/protection stay alive. */
        }
        const float calibrated_bus = Adc_Get_VoltageBus();
        uint32_t interrupt_state = __get_PRIMASK();
        __disable_irq();
        /* A new transient fault during INIT must survive even if PE15 has
         * already recovered. Clear/re-arm only the acknowledged old epoch;
         * the short critical section closes the check/clear/arm IRQ race. */
        if (FixedHardwareFaultEvents == fault_epoch)
        {
            Peripheral_Reset_ProtectFlag();
            /* A fault may previously have disabled BRK despite an old
             * 'armed' mirror. An explicit successful INIT may re-arm it. */
            FixedBreakIrqArmed = false;
            if (calibrated_bus > 200.0F)
            {
                Peripheral_EnableHardwareProtect();
                FixedBreakIrqArmed = true;
            }
        }
        if (interrupt_state == 0U)
            __enable_irq();
        /* Clear historical faults, not a still-present unsafe condition.
         * Retain the existing bus/temperature limits and PE15 fault input. */
        (void)Peripheral_UpdateUdc();
        Protect_HardWareFault(Peripheral_Get_HardwareBrk());
        Peripheral_Update_Temperature();
        FixedStartup_Status = 3U;
        main_int_resume_after_init(true);
        return;
    }
    if (!FixedControl_BackgroundBuild(&MainInt_ControlState))
        return;
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    /* CAN may request INIT during the slow table build. Leave the candidate
     * uncommitted so the next background INIT can cancel it atomically. */
    if (MainInt_State != INIT)
        FixedControl_CommitBackground(&MainInt_ControlState);
    if (interrupt_state == 0U)
        __enable_irq();
    FixedDebug_PublishMtpa(&MainInt_ControlState.foc.mtpa);
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
    MainInt_PrepareMode();
    Phase_t current = Peripheral_Get_PhaseCurrent();
    AngleResult_t position = Peripheral_Update_Position();
    FloatWithInv_t bus = Peripheral_UpdateUdc();
    if (MainInt_InitBlocksOutput() || FixedControl_ModeRequestsStop())
        Peripheral_Set_Stop(true);
    bool stop = Peripheral_Update_Break();

    FixedControlInput_t input = {
        .current_abc = current,
        .theta_rad = position.theta,
        .speed_rpm = position.speed,
        .bus_voltage = bus.val,
        .reset = stop};
    FixedControlOutput_t output = {.pwm_duty = {0.5F, 0.5F, 0.5F}};
    if (!MainInt_InitBlocksOutput())
        output = FixedControl_Step(&MainInt_ControlState, &input);
    /* The phase-current and bus-voltage protection above still run after a
     * failed INIT; only the not-ready control algorithm is skipped. */

    if (MainInt_InitBlocksOutput() || FixedControl_ModeRequestsStop())
        Peripheral_Set_Stop(true);
    /* Like the legacy flow, never write the pre-algorithm Stop snapshot back
     * here. EXTI4/BRK can assert Stop while FixedControl_Step runs. Recheck
     * the live latch before publishing PWM, including at Udc <= 200 V when
     * an EXTI stop need not have a corresponding BRK-IRQ fault flag. */
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
