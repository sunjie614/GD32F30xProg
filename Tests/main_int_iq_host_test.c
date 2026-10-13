#include <math.h>
#include <stdio.h>
#include <string.h>

/* Compile the real integration implementation, including its privately owned
 * contexts. Only target peripherals and the algorithm boundary are stubbed. */
#include "../Application/Src/main_int_iq.c"

MainIntTestDwt_t MainIntTestDwt;
uint32_t MainIntTestTimerCchp;
uint32_t SystemCoreClock = MCU_MAIN_FREQ;
volatile FocMode_t Foc_Mode = IDLE;
volatile float Foc_Id_Fdbk, Foc_Iq_Fdbk;
volatile FixedControlSelfTest_t FixedControl_SelfTest;
volatile uint16_t FixedStartup_Status;
volatile bool FixedBreakIrqArmed;
volatile uint32_t FixedHardwareFaultEvents;

enum InitResult { INIT_READY, INIT_UNINITIALIZED, INIT_BAD_TABLE, INIT_BAD_SELFTEST };
enum FaultStage { NO_NEW_FAULT, FAULT_DURING_ALGORITHM, FAULT_DURING_CALIBRATION };
enum ControlInterruption { CONTROL_UNINTERRUPTED, CONTROL_EXTI4_STOP,
                           CONTROL_HARDWARE_FAULT, CONTROL_MODE_IDLE,
                           CONTROL_MODE_EXIT };
enum Event { EV_STOP, EV_IRQ_DISABLE, EV_BREAK, EV_ALGORITHM, EV_CALIBRATION,
             EV_ARM_BREAK, EV_CLEAR_FLAGS, EV_BUS_CHECK, EV_HARDWARE_CHECK,
             EV_TEMP_CHECK, EV_CLEAR_EOIC, EV_CLEAR_PENDING, EV_IRQ_ENABLE };

static unsigned Failures;
static enum InitResult AlgorithmResult;
static enum FaultStage InjectFault;
static enum ControlInterruption InjectControlInterruption;
static bool CalibrationSucceeds;
static bool CheckInitContract;
static bool StopRequested, AdcIrqEnabled, CanIrqEnabled, BreakIrqEnabled;
static bool AdcEoicPending, AdcNvicPending, HardwareBreakSafe;
static uint32_t Primask, InitialPrimask, ProtectionFlags;
static float BusVoltage, Temperature, EstimatedSpeed;
static unsigned AlgorithmCalls, CalibrationCalls, ArmCalls, ClearFlagsCalls;
static unsigned ColdInitCalls, WarmInitCalls, CommitCalls;
static bool BackgroundBuildSucceeds, RequestInitDuringBuild;
static unsigned AdcIrqEnableCalls, ClearEoicCalls, ClearPendingCalls;
static unsigned GlobalDisableCalls, GlobalEnableCalls;
static unsigned Events[64], EventCount;
static float BufferValues[18];
static unsigned BufferSendCalls;
static FixedControlInput_t ControlInputs[256];
static unsigned ControlInputCount;
static unsigned ClearStopCalls, BreakCheckCalls, ControlInterruptions;
static unsigned PhaseProtectionCalls, BusProtectionCalls;
static bool PhaseCurrentFault;

static void expect_true(const char* name, bool condition)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL %s\n", name);
        Failures++;
    }
}

static void expect_close(const char* name, float actual, float expected,
                         float tolerance)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance)
    {
        fprintf(stderr, "FAIL %s: actual=%.8f expected=%.8f\n", name, actual, expected);
        Failures++;
    }
}

static void record_event(enum Event event)
{
    if (CheckInitContract && EventCount < sizeof(Events) / sizeof(Events[0]))
        Events[EventCount++] = (unsigned)event;
}

static unsigned event_position(enum Event event)
{
    for (unsigned index = 0U; index < EventCount; ++index)
        if (Events[index] == (unsigned)event)
            return index;
    return UINT32_MAX;
}

static void expect_stopped_init(const char* name)
{
    expect_true(name, StopRequested && Foc_Mode == IDLE && !AdcIrqEnabled
        && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U && CanIrqEnabled);
}

static void inject_hardware_fault(void)
{
    FixedHardwareFaultEvents++;
    ProtectionFlags |= (uint32_t)Hardware_Fault;
    BreakIrqEnabled = false; /* The real fault ISR disables the BRK IRQ. */
    FixedBreakIrqArmed = true; /* Suppress normal background auto-rearming. */
    StopRequested = true;
}

void nvic_irq_disable(int irq)
{
    expect_true("INIT disables only ADC IRQ", irq == ADC0_1_IRQn);
    expect_true("MOE is off before disabling ADC IRQ",
        StopRequested && Foc_Mode == IDLE
        && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
    AdcIrqEnabled = false;
    record_event(EV_IRQ_DISABLE);
}

void nvic_irq_enable(int irq, uint8_t priority, uint8_t subpriority)
{
    expect_true("ADC IRQ restored with legacy priority",
        irq == ADC0_1_IRQn && priority == 3U && subpriority == 0U);
    expect_stopped_init("successful INIT is still stopped before ADC IRQ enable");
    expect_true("EOIC and NVIC pending cleared before ADC IRQ restore",
        !AdcEoicPending && !AdcNvicPending);
    expect_true("INIT result gate published before ADC IRQ restore",
        (FixedStartup_Status == 3U && MainInt_State == RUNNING
         && !MainInt_InitBlocksOutput())
        || ((FixedStartup_Status == 1U || FixedStartup_Status == 2U)
            && MainInt_State == EXIT && MainInt_InitBlocksOutput()));
    expect_true("INIT restores caller PRIMASK before ADC IRQ restore",
        Primask == InitialPrimask);
    AdcIrqEnabled = true;
    AdcIrqEnableCalls++;
    record_event(EV_IRQ_ENABLE);
}

void adc_interrupt_flag_clear(uint32_t adc, uint32_t flags)
{
    expect_true("clear ADC EOIC only", adc == ADC0 && flags == ADC_INT_FLAG_EOIC);
    expect_stopped_init("clear EOIC under stopped INIT boundary");
    AdcEoicPending = false;
    ClearEoicCalls++;
    record_event(EV_CLEAR_EOIC);
}

void NVIC_ClearPendingIRQ(int irq)
{
    expect_true("clear ADC pending only", irq == ADC0_1_IRQn);
    expect_stopped_init("clear NVIC pending under stopped INIT boundary");
    AdcNvicPending = false;
    ClearPendingCalls++;
    record_event(EV_CLEAR_PENDING);
}

uint32_t __get_PRIMASK(void) { return Primask; }
void __disable_irq(void) { Primask = 1U; GlobalDisableCalls++; }
void __enable_irq(void) { Primask = 0U; GlobalEnableCalls++; }

static void mock_algorithm_initialize(FixedControlState_t* state)
{
    if (CheckInitContract)
    {
        expect_stopped_init("algorithm INIT runs after Stop/MOE/ADC gates");
        expect_true("algorithm build does not change caller mask or CAN IRQ gate",
            Primask == InitialPrimask && CanIrqEnabled);
    }
    AlgorithmCalls++;
    record_event(EV_ALGORITHM);
    memset(state, 0, sizeof(*state));
    state->initialized = AlgorithmResult != INIT_UNINITIALIZED;
    state->foc.initialized = true;
    state->foc.mtpa.table_valid = AlgorithmResult != INIT_BAD_TABLE;
    FixedControl_SelfTest = (FixedControlSelfTest_t){0};
    if (AlgorithmResult == INIT_BAD_SELFTEST)
        FixedControl_SelfTest.arithmetic_failures = 1U;
    if (InjectFault == FAULT_DURING_ALGORITHM)
        inject_hardware_fault();
}

void FixedControl_Init(FixedControlState_t* state)
{
    ColdInitCalls++;
    mock_algorithm_initialize(state);
}

void FixedControl_Reinitialize(FixedControlState_t* state)
{
    WarmInitCalls++;
    mock_algorithm_initialize(state);
}

bool Adc_Calibrate_CurrentOffsetChecked(void)
{
    expect_stopped_init("ADC recalibration is stopped with IRQ disabled");
    expect_true("ADC recalibration follows ready algorithm INIT",
        AlgorithmCalls == 1U && MainInt_ControlReady());
    expect_true("recalibration does not extend a global interrupt critical section",
        Primask == InitialPrimask);
    CalibrationCalls++;
    record_event(EV_CALIBRATION);
    if (InjectFault == FAULT_DURING_CALIBRATION)
        inject_hardware_fault();
    return CalibrationSucceeds;
}

float Adc_Get_VoltageBus(void) { return BusVoltage; }

void Peripheral_Set_Stop(bool stop)
{
    if (!stop)
        ClearStopCalls++;
    StopRequested = stop;
    record_event(EV_STOP);
}

bool Peripheral_Update_Break(void)
{
    BreakCheckCalls++;
    if (ProtectionFlags != 0U || MainInt_InitBlocksOutput() || !HardwareBreakSafe)
        StopRequested = true;
    if (StopRequested)
        MainIntTestTimerCchp &= ~((uint32_t)TIMER_CCHP_POEN);
    else
        MainIntTestTimerCchp |= TIMER_CCHP_POEN;
    record_event(EV_BREAK);
    return StopRequested;
}

void Peripheral_EnableHardwareProtect(void)
{
    expect_stopped_init("re-arm BRK only at stopped INIT boundary");
    expect_true("BRK re-arm occurs after successful recalibration", CalibrationCalls == 1U);
    expect_true("BRK re-arm is protected by the fault-epoch critical section", Primask == 1U);
    BreakIrqEnabled = true;
    ArmCalls++;
    record_event(EV_ARM_BREAK);
}

void Peripheral_Reset_ProtectFlag(void)
{
    expect_stopped_init("clear old protection only at stopped INIT boundary");
    expect_true("clear old protection after successful recalibration", CalibrationCalls == 1U);
    expect_true("historical flag clear is protected by fault-epoch critical section", Primask == 1U);
    ProtectionFlags = 0U;
    ClearFlagsCalls++;
    record_event(EV_CLEAR_FLAGS);
}

FloatWithInv_t Peripheral_UpdateUdc(void)
{
    BusProtectionCalls++;
    record_event(EV_BUS_CHECK);
    if (CheckInitContract && CalibrationCalls != 0U)
        expect_true("live bus check runs after restoring caller PRIMASK", Primask == InitialPrimask);
    if (BusVoltage < 20.0F)
        ProtectionFlags |= (uint32_t)Low_Voltage;
    if (BusVoltage > PROTECT_VOLTAGE_RATE + PROTECT_VOLTAGE_FLUCTUATION)
        ProtectionFlags |= (uint32_t)Over_Voltage;
    if (ProtectionFlags != 0U)
        StopRequested = true;
    return (FloatWithInv_t){BusVoltage, BusVoltage > 0.0F ? 1.0F / BusVoltage : 0.0F};
}

bool Peripheral_Get_HardwareBrk(void) { return HardwareBreakSafe; }

void Protect_HardWareFault(bool status)
{
    record_event(EV_HARDWARE_CHECK);
    if (!status)
        ProtectionFlags |= (uint32_t)Hardware_Fault;
}

void Peripheral_Update_Temperature(void)
{
    record_event(EV_TEMP_CHECK);
    if (Temperature > PROTECT_TEMPERATURE)
        ProtectionFlags |= (uint32_t)Over_Heat;
}

bool FixedControl_BackgroundBuild(FixedControlState_t* state)
{
    (void)state;
    if (RequestInitDuringBuild)
        MainInt_State = INIT;
    return BackgroundBuildSucceeds;
}

void FixedControl_CommitBackground(FixedControlState_t* state)
{
    (void)state;
    CommitCalls++;
}
void FixedDebug_PublishMtpa(const MtpaIqState_t* state) { (void)state; }

bool FixedControl_ModeRequestsStop(void)
{
    return MainInt_ModeRequestsStop() || Foc_Mode == IDLE;
}

Phase_t Peripheral_Get_PhaseCurrent(void)
{
    PhaseProtectionCalls++;
    if (PhaseCurrentFault)
    {
        ProtectionFlags |= (uint32_t)Over_Max_Current;
        StopRequested = true;
    }
    return (Phase_t){3.0F, -2.0F, -1.0F};
}
AngleResult_t Peripheral_Update_Position(void) { return (AngleResult_t){1.25F, 300.0F}; }

FixedControlOutput_t FixedControl_Step(FixedControlState_t* state,
                                      const FixedControlInput_t* input)
{
    if (ControlInputCount < sizeof(ControlInputs) / sizeof(ControlInputs[0]))
        ControlInputs[ControlInputCount++] = *input;
    /* Inject at the control call boundary after the real handler has captured
     * its stop snapshot. Model the externally visible ISR effects; the real
     * EXTI/BRK handlers are not compiled into this host integration test. */
    if (InjectControlInterruption != CONTROL_UNINTERRUPTED)
    {
        ControlInterruptions++;
        switch (InjectControlInterruption)
        {
        case CONTROL_EXTI4_STOP:
            StopRequested = true;
            MainIntTestTimerCchp &= ~((uint32_t)TIMER_CCHP_POEN);
            break;
        case CONTROL_HARDWARE_FAULT:
            inject_hardware_fault();
            MainIntTestTimerCchp &= ~((uint32_t)TIMER_CCHP_POEN);
            break;
        case CONTROL_MODE_IDLE:
            Foc_Mode = IDLE;
            break;
        case CONTROL_MODE_EXIT:
            MainInt_State = EXIT;
            break;
        default:
            break;
        }
        InjectControlInterruption = CONTROL_UNINTERRUPTED;
    }
    /* A deterministic control-state sentinel depends only on real input, not
     * on any monitor output. Comparing it across alpha choices tests the
     * actual Main_Int_Handler boundary, rather than just the filter helper. */
    state->foc.speed_pid.integral += 7;
    state->foc.speed_feedback_pu = (mc_real_t)(input->speed_rpm * 1000.0F);
    state->foc.control_angle_pu = (mc_real_t)(input->theta_rad * 1000.0F);
    Foc_Id_Fdbk = 4.0F;
    Foc_Iq_Fdbk = -2.0F;
    MainIntTestDwt.CYCCNT += 100U;
    return (FixedControlOutput_t){
        .pwm_duty = {0.4F, 0.5F, 0.6F}, .voltage_dq = {12.0F, -7.0F},
        .current_dq = {4.0F, -2.0F}, .estimated_theta_rad = 1.0F,
        .estimated_speed_rpm = EstimatedSpeed,
        .bus_voltage_q24_raw = 12163482, .phase_a_current_q24_raw = 1677722,
        .bus_voltage_q24_v = input->bus_voltage,
        .phase_a_current_q24_a = input->current_abc.a};
}

void Peripheral_Set_PWMChangePoint(Phase_t duty)
{
    expect_close("PWM is neutral when INIT blocks, otherwise the control result",
        duty.a, MainInt_InitBlocksOutput() ? 0.5F : 0.4F, 0.0F);
}

void Buffer_Put(float value, size_t index)
{
    expect_true("serial channel in bounds", index < 18U);
    if (index < 18U)
        BufferValues[index] = value;
}

void Buffer_Send(void) { BufferSendCalls++; }

static void reset_mocks(void)
{
    AlgorithmResult = INIT_READY;
    InjectFault = NO_NEW_FAULT;
    InjectControlInterruption = CONTROL_UNINTERRUPTED;
    CalibrationSucceeds = true;
    CheckInitContract = true;
    StopRequested = false;
    AdcIrqEnabled = CanIrqEnabled = true;
    BreakIrqEnabled = false;
    AdcEoicPending = AdcNvicPending = true;
    HardwareBreakSafe = true;
    Primask = 0U;
    InitialPrimask = 0U;
    ProtectionFlags = (uint32_t)Hardware_Fault;
    BusVoltage = 580.0F;
    Temperature = 25.0F;
    EstimatedSpeed = 0.0F;
    AlgorithmCalls = CalibrationCalls = ArmCalls = ClearFlagsCalls = 0U;
    ColdInitCalls = WarmInitCalls = CommitCalls = 0U;
    BackgroundBuildSucceeds = RequestInitDuringBuild = false;
    AdcIrqEnableCalls = ClearEoicCalls = ClearPendingCalls = 0U;
    GlobalDisableCalls = GlobalEnableCalls = 0U;
    EventCount = BufferSendCalls = ControlInputCount = 0U;
    ClearStopCalls = BreakCheckCalls = ControlInterruptions = 0U;
    PhaseProtectionCalls = BusProtectionCalls = 0U;
    PhaseCurrentFault = false;
    MainInt_InitOutputBlocked = false; /* Fixture starts from a running system. */
    memset(Events, 0, sizeof(Events));
    memset(BufferValues, 0, sizeof(BufferValues));
    memset(&MainInt_ControlState, 0, sizeof(MainInt_ControlState));
    MainIntTestTimerCchp = TIMER_CCHP_POEN | 0x40U;
    MainIntTestDwt.CYCCNT = 0U;
    MainInt_PreviousEntryTick = 0U;
    MainInt_PeriodTicks = MainInt_CycleTicks = MainInt_MaxCycleTicks = 0U;
    MainInt_FrequencyHz = MainInt_LoadPermille = 0U;
    FixedStartup_Status = 0U;
    FixedBreakIrqArmed = true; /* Old epoch may say armed after a fault ISR. */
    FixedHardwareFaultEvents = 9U;
    FixedControl_SelfTest = (FixedControlSelfTest_t){0};
    MainInt_State = INIT;
    Foc_Mode = SPEED;
}

static void init_failures_leave_output_disabled(void)
{
    const enum InitResult failures[] = {INIT_UNINITIALIZED, INIT_BAD_TABLE, INIT_BAD_SELFTEST};
    for (unsigned index = 0U; index < sizeof(failures) / sizeof(failures[0]); ++index)
    {
        reset_mocks();
        AlgorithmResult = failures[index];
        MainInt_ControlBackground();
        expect_true("algorithm failure status 1 and EXIT",
            FixedStartup_Status == 1U && MainInt_State == EXIT);
        expect_true("algorithm failure restores ADC but leaves output blocked",
            AdcIrqEnabled && MainInt_InitBlocksOutput() && StopRequested
            && Foc_Mode == IDLE && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
        expect_true("algorithm failure skips ADC calibration/clear/re-arm",
            AlgorithmCalls == 1U && CalibrationCalls == 0U
            && ClearFlagsCalls == 0U && ArmCalls == 0U && AdcIrqEnableCalls == 1U
            && ClearEoicCalls == 1U && ClearPendingCalls == 1U);
        expect_true("algorithm failure preserves existing protection",
            ProtectionFlags == (uint32_t)Hardware_Fault);
        expect_true("algorithm failure does not mask global interrupts",
            Primask == InitialPrimask && GlobalDisableCalls == 0U && GlobalEnableCalls == 0U);
    }
    reset_mocks();
    CalibrationSucceeds = false;
    MainInt_ControlBackground();
    expect_true("calibration failure status 2 and EXIT",
        FixedStartup_Status == 2U && MainInt_State == EXIT);
    expect_true("calibration failure restores ADC but leaves output blocked",
        AdcIrqEnabled && MainInt_InitBlocksOutput() && StopRequested
        && Foc_Mode == IDLE && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
    expect_true("calibration failure preserves latch and clears stale IRQ pending",
        CalibrationCalls == 1U && ClearFlagsCalls == 0U && ArmCalls == 0U
        && AdcIrqEnableCalls == 1U && !AdcEoicPending && !AdcNvicPending
        && ProtectionFlags == (uint32_t)Hardware_Fault);
    expect_true("calibration failure does not mask global interrupts",
        Primask == InitialPrimask && GlobalDisableCalls == 0U && GlobalEnableCalls == 0U);
}

static void failed_init_keeps_protection_and_requires_successful_retry(void)
{
    const enum InitResult cases[] = {
        INIT_UNINITIALIZED, INIT_BAD_TABLE, INIT_BAD_SELFTEST, INIT_READY};
    for (unsigned n = 0U; n < sizeof(cases) / sizeof(cases[0]); ++n)
    {
        reset_mocks();
        AlgorithmResult = cases[n];
        CalibrationSucceeds = cases[n] != INIT_READY;
        MainInt_ControlBackground();
        expect_true("all runtime INIT failures restore ADC and block output",
            AdcIrqEnabled && MainInt_InitBlocksOutput());
        CheckInitContract = false;
        /* Reproduce the old bypass commands, including a forged public
         * status. Neither hardware recovery nor the ISR may trust them. */
        MainInt_State = RUNNING;
        Foc_Mode = IF_MODE;
        FixedStartup_Status = 3U;
        ProtectionFlags = 0U;
        Peripheral_Set_Stop(false);
        expect_true("INIT failure cannot be cleared by ordinary CAN commands",
            Peripheral_Update_Break() && MainInt_InitBlocksOutput()
            && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
        unsigned phase_before = PhaseProtectionCalls;
        unsigned bus_before = BusProtectionCalls;
        PhaseCurrentFault = true;
        BusVoltage = 721.0F;
        Main_Int_Handler();
        expect_true("ADC failure path still samples and checks current and bus",
            PhaseProtectionCalls == phase_before + 1U
            && BusProtectionCalls == bus_before + 1U
            && (ProtectionFlags & (uint32_t)Over_Max_Current) != 0U
            && (ProtectionFlags & (uint32_t)Over_Voltage) != 0U);
        expect_true("failed initialization never enters a not-ready core",
            ControlInputCount == 0U && BufferSendCalls == 1U
            && StopRequested && AdcIrqEnabled);

        /* Retry without clearing the private latch or resetting the instance. */
        MainInt_State = INIT;
        AlgorithmResult = INIT_READY;
        CalibrationSucceeds = true;
        PhaseCurrentFault = false;
        BusVoltage = 580.0F;
        AlgorithmCalls = CalibrationCalls = 0U;
        MainInt_ControlBackground();
        expect_true("successful retry restores eligibility but remains stopped",
            FixedStartup_Status == 3U && MainInt_State == RUNNING
            && !MainInt_InitBlocksOutput() && AdcIrqEnabled
            && StopRequested && Foc_Mode == IDLE);
        Main_Int_Handler();
        expect_true("post-INIT ADC resumes the regular control path",
            ControlInputCount == 1U && ControlInputs[0].reset);
        Foc_Mode = IF_MODE;
        Peripheral_Set_Stop(false);
        Main_Int_Handler();
        expect_true("explicit normal run request works after successful retry",
            ControlInputCount == 2U && !ControlInputs[1].reset
            && !StopRequested && (MainIntTestTimerCchp & TIMER_CCHP_POEN) != 0U);
    }
}

static void successful_init_recalibrates_and_revalidates(void)
{
    const float voltages[] = {199.0F, 200.0F, 200.25F, 580.0F, 0.0F, 721.0F};
    for (unsigned index = 0U; index < sizeof(voltages) / sizeof(voltages[0]); ++index)
    {
        reset_mocks();
        BusVoltage = voltages[index];
        MainInt_ControlBackground();
        expect_true("successful INIT status 3 RUNNING but stopped IDLE",
            FixedStartup_Status == 3U && MainInt_State == RUNNING
            && StopRequested && Foc_Mode == IDLE && AdcIrqEnabled && CanIrqEnabled
            && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
        expect_true("successful INIT recalibrates exactly once",
            AlgorithmCalls == 1U && CalibrationCalls == 1U && ClearFlagsCalls == 1U);
        expect_true("BRK re-arm uses strict greater-than-200V new epoch",
            ArmCalls == (BusVoltage > 200.0F ? 1U : 0U)
            && BreakIrqEnabled == (BusVoltage > 200.0F)
            && FixedBreakIrqArmed == (BusVoltage > 200.0F));
        uint32_t expected_flags = BusVoltage < 20.0F ? (uint32_t)Low_Voltage
            : BusVoltage > PROTECT_VOLTAGE_RATE + PROTECT_VOLTAGE_FLUCTUATION
            ? (uint32_t)Over_Voltage : 0U;
        expect_true("present bus fault is detected again after clearing old latch",
            ProtectionFlags == expected_flags);
        expect_true("success clears pending ADC flags once",
            ClearEoicCalls == 1U && ClearPendingCalls == 1U && AdcIrqEnableCalls == 1U);
        expect_true("INIT sequence disables ADC before building then recalibrates",
            event_position(EV_IRQ_DISABLE) < event_position(EV_ALGORITHM)
            && event_position(EV_ALGORITHM) < event_position(EV_CALIBRATION)
            && event_position(EV_CALIBRATION) < event_position(EV_CLEAR_FLAGS));
        expect_true("old latch cleared before live protection revalidation",
            event_position(EV_CLEAR_FLAGS) < event_position(EV_BUS_CHECK)
            && event_position(EV_CLEAR_FLAGS) < event_position(EV_HARDWARE_CHECK)
            && event_position(EV_CLEAR_FLAGS) < event_position(EV_TEMP_CHECK));
        expect_true("IRQ pending cleared before enabling ADC",
            event_position(EV_CLEAR_EOIC) < event_position(EV_IRQ_ENABLE)
            && event_position(EV_CLEAR_PENDING) < event_position(EV_IRQ_ENABLE));
        expect_true("INIT preserves unrelated timer CCHP bits", (MainIntTestTimerCchp & 0x40U) != 0U);
        expect_true("INIT uses one short critical section then unmasks",
            Primask == 0U && GlobalDisableCalls == 1U && GlobalEnableCalls == 1U);
    }
    reset_mocks();
    HardwareBreakSafe = false;
    MainInt_ControlBackground();
    expect_true("unsafe PE15 re-latches hardware fault after historical clear",
        ClearFlagsCalls == 1U && (ProtectionFlags & (uint32_t)Hardware_Fault) != 0U
        && StopRequested && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
    reset_mocks();
    Temperature = PROTECT_TEMPERATURE + 1.0F;
    MainInt_ControlBackground();
    expect_true("still-present overheat re-latches after historical clear",
        (ProtectionFlags & (uint32_t)Over_Heat) != 0U && StopRequested);

    reset_mocks();
    Primask = InitialPrimask = 1U;
    MainInt_ControlBackground();
    expect_true("INIT preserves a caller's already-disabled PRIMASK",
        Primask == 1U && GlobalDisableCalls == 1U && GlobalEnableCalls == 0U
        && FixedStartup_Status == 3U && AdcIrqEnabled);
}

static void new_transient_fault_is_not_erased(void)
{
    const enum FaultStage stages[] = {FAULT_DURING_ALGORITHM, FAULT_DURING_CALIBRATION};
    for (unsigned index = 0U; index < sizeof(stages) / sizeof(stages[0]); ++index)
    {
        reset_mocks();
        InjectFault = stages[index];
        if (index == 1U)
            FixedHardwareFaultEvents = UINT32_MAX;
        uint32_t events_before = FixedHardwareFaultEvents;
        MainInt_ControlBackground();
        expect_true("new transient fault does not erase latch or re-arm BRK",
            ClearFlagsCalls == 0U && ArmCalls == 0U && !BreakIrqEnabled
            && (ProtectionFlags & (uint32_t)Hardware_Fault) != 0U);
        expect_true("new fault blocks background auto re-arm with armed epoch marker",
            FixedBreakIrqArmed && StopRequested && Foc_Mode == IDLE);
        expect_true("fault event counter observes increment including single wrap",
            FixedHardwareFaultEvents == events_before + 1U);
        expect_true("algorithm/calibration complete without concealing new fault",
            AlgorithmCalls == 1U && CalibrationCalls == 1U
            && FixedStartup_Status == 3U && MainInt_State == RUNNING && AdcIrqEnabled
            && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
        expect_true("new-fault branch also restores caller PRIMASK",
            Primask == InitialPrimask && GlobalDisableCalls == 1U && GlobalEnableCalls == 1U);
    }
}

static void prepare_running_irq(float alpha)
{
    reset_mocks();
    CheckInitContract = false;
    ProtectionFlags = 0U;
    StopRequested = false;
    MainInt_State = RUNNING;
    Foc_Mode = IF_MODE;
    MainInt_ControlInit();
    expect_true("startup uses cold initialization", ColdInitCalls == 1U && WarmInitCalls == 0U);
    expect_close("MainInt default monitor is configured for 10ms at 5kHz",
        MainInt_TelemetryState.alpha, 1.0F / 51.0F, 0.000001F);
    MainInt_TelemetryState.alpha = alpha;
}

static void background_init_uses_warm_path_and_cancels_candidate(void)
{
    reset_mocks();
    MainInt_ControlBackground();
    expect_true("runtime INIT uses warm API only", WarmInitCalls == 1U && ColdInitCalls == 0U);
    reset_mocks();
    CheckInitContract = false;
    MainInt_State = RUNNING;
    Foc_Mode = IDLE;
    BackgroundBuildSucceeds = RequestInitDuringBuild = true;
    MainInt_ControlBackground();
    expect_true("INIT arriving during table build prevents commit",
        MainInt_State == INIT && CommitCalls == 0U);
    expect_true("abandoned commit restores interrupt mask", Primask == 0U);
    BackgroundBuildSucceeds = RequestInitDuringBuild = false;
    MainInt_ControlBackground();
    expect_true("pending INIT reaches warm API next iteration",
        WarmInitCalls == 1U && ColdInitCalls == 0U && CommitCalls == 0U);
    reset_mocks();
    CheckInitContract = false;
    MainInt_State = RUNNING;
    Foc_Mode = IDLE;
    BackgroundBuildSucceeds = true;
    MainInt_ControlBackground();
    expect_true("normal table build still commits", CommitCalls == 1U);
}

static void run_control_tick(float estimated_speed)
{
    EstimatedSpeed = estimated_speed;
    MainIntTestDwt.CYCCNT += 24000U;
    Main_Int_Handler();
}

static void asynchronous_stop_survives_control_execution(void)
{
    const enum ControlInterruption cases[] = {
        CONTROL_EXTI4_STOP, CONTROL_HARDWARE_FAULT,
        CONTROL_MODE_IDLE, CONTROL_MODE_EXIT};
    for (unsigned index = 0U; index < sizeof(cases) / sizeof(cases[0]); ++index)
    {
        prepare_running_irq(1.0F / 51.0F);
        /* Reproduce the particularly important weak-bus EXTI case: BRK IRQ
         * not armed, so there is no protection flag to rescue a lost Stop. */
        BusVoltage = 100.0F;
        FixedBreakIrqArmed = BreakIrqEnabled = false;
        InjectControlInterruption = cases[index];
        run_control_tick(100.0F);
        expect_true("interruption occurs after a running input snapshot",
            ControlInterruptions == 1U && ControlInputCount == 1U
            && !ControlInputs[0].reset);
        expect_true("control completion never clears asynchronous Stop",
            StopRequested && ClearStopCalls == 0U);
        expect_true("control completion refreshes stop/protection gate",
            BreakCheckCalls >= 2U
            && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
        if (cases[index] == CONTROL_EXTI4_STOP)
            expect_true("EXTI Stop survives without fault latch or BRK IRQ",
                ProtectionFlags == 0U && !BreakIrqEnabled && !FixedBreakIrqArmed);
        if (cases[index] == CONTROL_HARDWARE_FAULT)
            expect_true("hardware fault remains latched after control completion",
                (ProtectionFlags & (uint32_t)Hardware_Fault) != 0U);
        /* Returning a mode to RUNNING/IF is not permission to clear Stop. */
        MainInt_State = RUNNING;
        Foc_Mode = IF_MODE;
        for (unsigned tick = 1U; tick < 5U; ++tick)
        {
            run_control_tick(100.0F);
            expect_true("subsequent control ticks receive reset after asynchronous Stop",
                ControlInputs[tick].reset && StopRequested && ClearStopCalls == 0U
                && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
        }
    }

    prepare_running_irq(1.0F / 51.0F);
    for (unsigned tick = 0U; tick < 5U; ++tick)
    {
        run_control_tick(100.0F);
        expect_true("uninterrupted running loop neither asserts nor clears Stop",
            !ControlInputs[tick].reset && !StopRequested && ClearStopCalls == 0U
            && (MainIntTestTimerCchp & TIMER_CCHP_POEN) != 0U);
    }

    prepare_running_irq(1.0F / 51.0F);
    StopRequested = true;
    run_control_tick(100.0F);
    expect_true("an existing Stop remains asserted through a complete tick",
        ControlInputs[0].reset && StopRequested && ClearStopCalls == 0U
        && (MainIntTestTimerCchp & TIMER_CCHP_POEN) == 0U);
}

static bool same_control_input(const FixedControlInput_t* left,
                               const FixedControlInput_t* right)
{
    return left->current_abc.a == right->current_abc.a
        && left->current_abc.b == right->current_abc.b
        && left->current_abc.c == right->current_abc.c
        && left->theta_rad == right->theta_rad && left->speed_rpm == right->speed_rpm
        && left->bus_voltage == right->bus_voltage && left->reset == right->reset;
}

static void display_filter_does_not_feed_control(void)
{
    FixedControlInput_t baseline_inputs[20];
    unsigned char baseline_state[sizeof(MainInt_ControlState)];
    float baseline_display = 0.0F;
    const float alphas[] = {1.0F / 51.0F, 0.5F};
    for (unsigned run = 0U; run < 2U; ++run)
    {
        prepare_running_irq(alphas[run]);
        run_control_tick(100.0F);
        expect_close("IRQ first display sample passes through", BufferValues[3], 100.0F, 0.0F);
        for (unsigned tick = 1U; tick < 20U; ++tick)
            run_control_tick(1000.0F);
        expect_true("IRQ captures and sends one frame request per tick",
            ControlInputCount == 20U && BufferSendCalls == 20U);
        if (run == 0U)
        {
            memcpy(baseline_inputs, ControlInputs, sizeof(baseline_inputs));
            memcpy(baseline_state, &MainInt_ControlState, sizeof(MainInt_ControlState));
            baseline_display = BufferValues[3];
        }
        else
        {
            expect_true("display alpha does not change complete control state",
                memcmp(baseline_state, &MainInt_ControlState, sizeof(MainInt_ControlState)) == 0);
            for (unsigned tick = 0U; tick < 20U; ++tick)
                expect_true("display alpha does not change controller input snapshot",
                    same_control_input(&baseline_inputs[tick], &ControlInputs[tick]));
            expect_true("display alpha actually changes serial channel 3",
                BufferValues[3] > baseline_display + 100.0F);
        }
        expect_close("real speed remains serial channel 2", BufferValues[2], 300.0F, 0.0F);
        expect_close("real sensor speed feeds control, not display estimate",
            ControlInputs[19].speed_rpm, 300.0F, 0.0F);
        expect_close("raw estimated angle remains channel 1", BufferValues[1], 1.0F, 0.0F);
        expect_close("final d voltage remains channel 15", BufferValues[15], 12.0F, 0.0F);
        expect_close("d feedback remains channel 16", BufferValues[16], 4.0F, 0.0F);
        expect_close("q feedback remains channel 17", BufferValues[17], -2.0F, 0.0F);
    }

    prepare_running_irq(1.0F / 51.0F);
    run_control_tick(0.0F);
    for (unsigned tick = 0U; tick < 151U; ++tick)
        run_control_tick(1000.0F);
    expect_true("10ms filter is below 95 percent after 30.2ms", BufferValues[3] < 950.0F);
    run_control_tick(1000.0F);
    expect_true("10ms filter reaches 95 percent by 30.4ms", BufferValues[3] >= 950.0F);
    expect_close("10ms IRQ filter matches first-order step response",
        BufferValues[3], 1000.0F * (1.0F - powf(50.0F / 51.0F, 152.0F)), 0.005F);
    expect_close("filtered value never changes measured control speed",
        ControlInputs[152].speed_rpm, 300.0F, 0.0F);
}

int main(void)
{
    init_failures_leave_output_disabled();
    failed_init_keeps_protection_and_requires_successful_retry();
    successful_init_recalibrates_and_revalidates();
    new_transient_fault_is_not_erased();
    background_init_uses_warm_path_and_cancels_candidate();
    asynchronous_stop_survives_control_execution();
    display_filter_does_not_feed_control();
    if (Failures != 0U)
        return 1;
    puts("PASS: real MainInt INIT success/failure ADC continuity, retry interlock, asynchronous Stop and display isolation");
    return 0;
}
