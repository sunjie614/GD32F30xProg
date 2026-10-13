/* Compile the real hardware_interface.c with only peripheral/other-module
 * dependencies mocked. This validates C recovery conditions and register
 * requests, not real EXTI/NVIC behavior, idle pin levels or gate waveforms. */
#include <assert.h>
#include <stdio.h>
#include "hardware_interface.h"
#include "adc.h"
#include "can.h"
#include "gpio.h"
#include "motor.h"
#include "position_sensor.h"
#include "protect.h"
#include "tim.h"
#include "usart.h"

volatile uint32_t HardwareTest_Swevg;
static bool pe4_high, pe15_high = true;
static bool protected_fault, mock_moe;
static unsigned pe4_reads, pe15_reads, moe_enable_calls, protection_resets;
static uint32_t enabled_timer_interrupts;
static uint32_t mock_primask;
#if defined(MC_NUMERIC_IQMATH)
static bool init_output_blocked;
bool MainInt_InitBlocksOutput(void) { return init_output_blocked; }
#endif
enum InterruptEvent { NO_EVENT, CURRENT_FAULT, PE4_STOP };
enum InterruptPoint { NO_POINT, BEFORE_MASK, AFTER_PE15_READ, BEFORE_MOE_ENABLE };
static enum InterruptEvent scheduled_event, pending_event;
static enum InterruptPoint scheduled_point;

static void deliver_interrupt(enum InterruptEvent event)
{
    if (event == CURRENT_FAULT)
        protected_fault = true;
    Peripheral_Set_Stop(true);
    /* PE4 directly generates BRKG. An ADC protection handler calls the real
     * shared stop function. In both cases hardware drops MOE immediately. */
    mock_moe = false;
    if (event == CURRENT_FAULT)
        assert(Peripheral_Update_Break());
    HardwareTest_Swevg = 0U; /* Event generation bits are not a latch. */
}

static void interrupt_at(enum InterruptPoint point)
{
    if (scheduled_point != point || scheduled_event == NO_EVENT)
        return;
    enum InterruptEvent event = scheduled_event;
    scheduled_event = NO_EVENT;
    scheduled_point = NO_POINT;
    if (mock_primask != 0U)
        pending_event = event;
    else
        deliver_interrupt(event);
}

#if defined(MC_NUMERIC_IQMATH)
uint32_t __get_PRIMASK(void) { return mock_primask; }
void __disable_irq(void)
{
    /* A preemption just before CPSID must be observed by the checks which
     * follow it, not hidden by a snapshot from before the critical section. */
    interrupt_at(BEFORE_MASK);
    mock_primask = 1U;
}
void __enable_irq(void)
{
    mock_primask = 0U;
    if (pending_event != NO_EVENT)
    {
        enum InterruptEvent event = pending_event;
        pending_event = NO_EVENT;
        deliver_interrupt(event);
    }
}
#endif

FlagStatus gpio_input_bit_get(uint32_t port, uint32_t pin)
{
    assert(port == GPIOE);
    if (pin == GPIO_PIN_4)
    {
        ++pe4_reads;
        return pe4_high ? SET : RESET;
    }
    assert(pin == GPIO_PIN_15);
    ++pe15_reads;
    FlagStatus sampled = pe15_high ? SET : RESET;
    interrupt_at(AFTER_PE15_READ);
    return sampled;
}
void gpio_bit_set(uint32_t port, uint32_t pin) { (void)port; (void)pin; }
void gpio_bit_reset(uint32_t port, uint32_t pin) { (void)port; (void)pin; }
void timer_primary_output_config(uint32_t timer, unsigned enabled)
{
    assert(timer == TIMER0);
    if (enabled == ENABLE)
        interrupt_at(BEFORE_MOE_ENABLE);
    mock_moe = enabled == ENABLE;
    if (enabled == ENABLE) ++moe_enable_calls;
}
void timer_interrupt_enable(uint32_t timer, uint32_t interrupt)
{
    assert(timer == TIMER0);
    enabled_timer_interrupts |= interrupt;
}
void timer_interrupt_disable(uint32_t timer, uint32_t interrupt)
{
    assert(timer == TIMER0);
    enabled_timer_interrupts &= ~interrupt;
}
bool Protect_Validate_Flag(void) { return protected_fault; }
void Protect_Reset_Flag(void) { ++protection_resets; protected_fault = false; }
bool Protect_PhaseCurrent(Phase_t current) { (void)current; return false; }
bool Protect_BusVoltage(float voltage) { (void)voltage; return false; }
bool Protect_Temperature(float temperature) { (void)temperature; return false; }
bool Protect_Get_FanState(float temperature) { (void)temperature; return false; }
float Adc_Get_VoltageBus(void) { return 200.0F; }
float Adc_Get_VoltageBusInv(void) { return 0.005F; }
float Adc_Get_Temperature(void) { return 25.0F; }
void Adc_Get_ThreePhaseCurrent(float* a, float* b, float* c) { *a = *b = *c = 0.0F; }
void Adc_Calibrate_CurrentOffset(void) {}
void Motor_Set_Position(uint16_t position) { (void)position; }
float Motor_Get_ThetaElec(void) { return 0.0F; }
float Motor_Get_Speed(void) { return 0.0F; }
void ReadPositionSensor(uint16_t* position) { *position = 0U; }
bool CAN_Buffer_Get(can_receive_message_struct* message) { (void)message; return false; }
uint8_t can_message_transmit(uint32_t can, can_transmit_message_struct* message)
{ (void)can; (void)message; return CAN_NOMAILBOX; }
void USART_DMA_Send(float* buffer, uint8_t count) { (void)buffer; (void)count; }
void Set_PWM_Compare(float a, float b, float c, float brake)
{ (void)a; (void)b; (void)c; (void)brake; }

static void ready_fixture(bool pin4, bool pin15)
{
#if defined(MC_NUMERIC_IQMATH)
    init_output_blocked = false;
#endif
    mock_primask = 0U;
    scheduled_event = pending_event = NO_EVENT;
    scheduled_point = NO_POINT;
    /* Reset the private Software_BRK through the public successful-recovery
     * path instead of modifying it or replacing the function under test. */
    protected_fault = false;
    pe15_high = true;
    pe4_high = false;
    Peripheral_Set_Stop(false);
    assert(!Peripheral_Update_Break());
    assert(!Peripheral_Get_SoftwareBrk());
    pe4_high = pin4;
    pe15_high = pin15;
    mock_moe = false;
    HardwareTest_Swevg = 0U;
    pe4_reads = pe15_reads = moe_enable_calls = protection_resets = 0U;
}

static void stopped_state_requests_break_once_or_legacy_repeat(void)
{
    for (unsigned pins = 0U; pins < 4U; ++pins)
    {
        ready_fixture((pins & 1U) != 0U, (pins & 2U) != 0U);
        Peripheral_Set_Stop(true);
        assert(Peripheral_Update_Break());
        assert(Peripheral_Get_Stop() && Peripheral_Get_SoftwareBrk());
        assert((HardwareTest_Swevg & TIMER_SWEVG_BRKG) != 0U);
        assert(moe_enable_calls == 0U && pe4_reads == 0U && pe15_reads == 0U);
        HardwareTest_Swevg = 0U;
        for (unsigned n = 0U; n < 100U; ++n)
        {
#if defined(MC_NUMERIC_IQMATH)
            /* Regression: a previous recovery race left MOE enabled with
             * Software_BRK already true. Every Stop must force it off. */
            mock_moe = true;
#endif
            assert(Peripheral_Update_Break());
#if defined(MC_NUMERIC_IQMATH)
            assert(!mock_moe);
#endif
        }
#if defined(MC_NUMERIC_IQMATH)
        assert(HardwareTest_Swevg == 0U); /* retain the approved one-shot */
#else
        assert((HardwareTest_Swevg & TIMER_SWEVG_BRKG) != 0U);
#endif
        assert(moe_enable_calls == 0U && protection_resets == 0U);
    }
}

static void recovery_matches_original_pe15_only(void)
{
    for (unsigned pins = 0U; pins < 4U; ++pins)
    {
        bool high15 = (pins & 2U) != 0U;
        ready_fixture((pins & 1U) != 0U, high15);
        Peripheral_Set_Stop(false);
        bool stopped = Peripheral_Update_Break();
        assert(stopped == !high15 && Peripheral_Get_Stop() == !high15);
        assert(mock_moe == high15 && moe_enable_calls == (unsigned)high15);
        assert(pe4_reads == 0U && pe15_reads == 1U);
#if defined(MC_NUMERIC_IQMATH)
        assert((HardwareTest_Swevg & TIMER_SWEVG_BRKG)
            == (high15 ? 0U : TIMER_SWEVG_BRKG));
#else
        assert(HardwareTest_Swevg == 0U);
#endif
        if (high15) assert(!Peripheral_Get_SoftwareBrk());
    }
}

static void protection_latch_prevents_recovery(void)
{
    for (unsigned pins = 0U; pins < 4U; ++pins)
    {
        ready_fixture((pins & 1U) != 0U, (pins & 2U) != 0U);
        protected_fault = true;
        Peripheral_Set_Stop(false);
        assert(Peripheral_Update_Break());
        assert(Peripheral_Get_Stop() && protected_fault);
        assert((HardwareTest_Swevg & TIMER_SWEVG_BRKG) != 0U);
        assert(!mock_moe && moe_enable_calls == 0U && protection_resets == 0U);
    }
}

static void interrupt_control_does_not_enable_moe(void)
{
    ready_fixture(false, true);
    enabled_timer_interrupts = 0U;
    Peripheral_EnableHardwareProtect();
    assert((enabled_timer_interrupts & TIMER_INT_BRK) != 0U);
    assert(!mock_moe && moe_enable_calls == 0U);
    Peripheral_DisableHardwareProtect();
    assert(enabled_timer_interrupts == 0U && !mock_moe);
}

#if defined(MC_NUMERIC_IQMATH)
static void failed_init_cannot_arm_pwm(void)
{
    for (unsigned mask = 0U; mask <= 1U; ++mask)
    {
        ready_fixture(false, true);
        init_output_blocked = true;
        mock_primask = mask;
        for (unsigned cycle = 0U; cycle < 4U; ++cycle)
        {
            protected_fault = false; /* CCP Flag=0 */
            Peripheral_Set_Stop(false); /* CCP Stop=0 */
            mock_moe = true; /* Even a residual output must be turned off. */
            assert(Peripheral_Update_Break());
            assert(Peripheral_Get_Stop() && !mock_moe && moe_enable_calls == 0U);
            assert(mock_primask == mask && pe4_reads == 0U);
        }
        init_output_blocked = false; /* Only successful INIT clears this gate. */
        Peripheral_Set_Stop(false);
        assert(!Peripheral_Update_Break() && mock_moe && moe_enable_calls == 1U);
        assert(mock_primask == mask);
    }
    mock_primask = 0U;
}

static void interrupts_cannot_undo_stop(void)
{
    const enum InterruptPoint points[] = {
        BEFORE_MASK, AFTER_PE15_READ, BEFORE_MOE_ENABLE};
    for (unsigned event = CURRENT_FAULT; event <= PE4_STOP; ++event)
    {
        for (unsigned point = 0U; point < sizeof(points) / sizeof(points[0]); ++point)
        {
            ready_fixture(false, true);
            Peripheral_Set_Stop(false);
            scheduled_event = (enum InterruptEvent)event;
            scheduled_point = points[point];
            assert(Peripheral_Update_Break());
            assert(scheduled_event == NO_EVENT && pending_event == NO_EVENT);
            assert(Peripheral_Get_Stop() && !mock_moe);
            assert(mock_primask == 0U);
            if (event == CURRENT_FAULT) assert(protected_fault);
            for (unsigned cycle = 0U; cycle < 3U; ++cycle)
            {
                assert(Peripheral_Update_Break());
                assert(Peripheral_Get_Stop() && !mock_moe);
            }
        }
    }
}

static void preserve_callers_interrupt_mask(void)
{
    for (unsigned mask = 0U; mask <= 1U; ++mask)
    {
        ready_fixture(false, true);
        mock_primask = mask;
        Peripheral_Set_Stop(false);
        assert(!Peripheral_Update_Break());
        assert(mock_primask == mask && mock_moe);
        Peripheral_Set_Stop(true);
        assert(Peripheral_Update_Break());
        assert(mock_primask == mask && !mock_moe);
    }
    mock_primask = 0U;
    for (unsigned event = CURRENT_FAULT; event <= PE4_STOP; ++event)
    {
        ready_fixture(false, true);
        mock_primask = 1U;
        Peripheral_Set_Stop(false);
        scheduled_event = (enum InterruptEvent)event;
        scheduled_point = BEFORE_MOE_ENABLE;
        assert(!Peripheral_Update_Break());
        assert(mock_primask == 1U && pending_event == (enum InterruptEvent)event);
        /* A caller which already masked IRQs owns the unmask operation.
         * The shared recovery must not discard that pending stop event. */
        __enable_irq();
        assert(mock_primask == 0U && pending_event == NO_EVENT);
        assert(Peripheral_Get_Stop() && !mock_moe);
    }
}
#endif

int main(void)
{
    /* Cold-boot Stop remains asserted before any mock recovery. */
    assert(Peripheral_Get_Stop());
    assert(!Peripheral_Get_SoftwareBrk());
    stopped_state_requests_break_once_or_legacy_repeat();
    recovery_matches_original_pe15_only();
    protection_latch_prevents_recovery();
    interrupt_control_does_not_enable_moe();
#if defined(MC_NUMERIC_IQMATH)
    failed_init_cannot_arm_pwm();
    interrupts_cannot_undo_stop();
    preserve_callers_interrupt_mask();
    puts("PASS: real IQMATH continuous gate-off and IRQ-safe Stop/PE15 recovery.");
#else
    puts("PASS: real FLOAT_REF hardware-interface original Stop/PE15 recovery.");
#endif
    return 0;
}
