#include "main_int.h"

#include "buffer.h"
#include "fixed_control.h"
#include "gd32f30x.h"
#include "hardware_interface.h"
#include "reciprocal.h"

/* The interrupt integration layer owns the single top-level mutable instance.
 * Every algorithm below it receives its state explicitly. */
static FixedControlState_t MainInt_ControlState;
volatile uint32_t MainInt_PeriodTicks = 0U;
volatile uint32_t MainInt_CycleTicks = 0U;
volatile uint32_t MainInt_MaxCycleTicks = 0U;
volatile uint32_t MainInt_FrequencyHz = 0U;
volatile uint16_t MainInt_LoadPermille = 0U;
static uint32_t MainInt_PreviousEntryTick = 0U;

void MainInt_ControlInit(void)
{
    FixedControl_Init(&MainInt_ControlState);
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
    bool stop = Peripheral_Update_Break();

    FixedControlInput_t input = {
        .current_abc = current,
        .theta_rad = position.theta,
        .speed_rpm = position.speed,
        .bus_voltage = bus.val,
        .reset = stop};
    FixedControlOutput_t output = FixedControl_Step(
        &MainInt_ControlState, &input);

    Peripheral_Set_Stop(stop);
    Peripheral_Set_PWMChangePoint(output.pwm_duty);

    /* Float telemetry is deliberately isolated at the CAN/A2L boundary. */
    Buffer_Put(position.theta, 0U);
    Buffer_Put(output.estimated_theta_rad, 1U);
    Buffer_Put(position.speed, 2U);
    Buffer_Put(output.estimated_speed_rpm, 3U);
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
