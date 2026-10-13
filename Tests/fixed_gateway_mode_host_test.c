#include <math.h>
#include <stdio.h>
#include <string.h>
#include "fixed_control.h"
#include "fixed_numeric_config.h"
#include "identification.h"
#include "main_int.h"
#include "parameters.h"
extern volatile A2lPidMirror_t Foc_Pid_CurD_Handler;

static int warm_init_regression(void)
{
    FixedControlState_t state;
    /* A cold API must never branch on poisoned/uninitialized instance data. */
    memset(&state, 0xA5, sizeof(state));
    FixedControl_Init(&state);
    MtpaIqParameters_t defaults = {0};
    MtpaIq_DefaultParameters(&defaults);
    if (Sensorless_Enabled || Sensorless_ThetAdj != 0.0F
        || state.foc.sensorless.angle_pu != MC_ZERO
        || !state.foc.mtpa.table_valid
        || memcmp(&defaults, &state.foc.mtpa.parameters, sizeof(defaults)) != 0)
        return 31;

    /* A non-default accepted model must survive byte-for-byte, including all
     * table validity flags. This is an actual successful table build. */
    state.foc.mtpa.parameters.ad0 = McMath_Mul(defaults.ad0, MC_CONST(1.1));
    if (!MtpaIq_BuildTable(&state.foc.mtpa, MC_ZERO, MC_ONE)) return 32;
    MtpaIqState_t accepted = state.foc.mtpa; /* Host only; no target stack copy. */
    state.foc.inductance_dq_pu = (ParkIq_t){MC_CONST(0.6), MC_CONST(0.2)};
    state.foc.sensorless.angle_pu = MC_CONST(0.375);
    state.foc.sensorless.speed_pu = MC_CONST(0.15);
    state.foc.sensorless.speed_sum = 12345;
    state.foc.sensorless.speed_count = 3U;
    state.foc.sensorless.angle_product_residual = 37;
    state.foc.sensorless.common_pid.integral = MC_CONST(0.3);
    state.foc.sensorless.integral_product_residual = 29;
    state.foc.sensorless.speed_filter.x1 = MC_CONST(0.4);
    state.foc.sensorless.speed_filter.y1 = MC_CONST(0.2);
    state.foc.leso.current_estimate.a = MC_CONST(0.2);
    state.foc.leso.disturbance.b = MC_CONST(-0.3);
    state.foc.leso.emf_estimate.a = MC_CONST(0.04);
    state.foc.hfi.previous_current.a = MC_CONST(0.08);
    state.foc.hfi.previous_high_frequency.b = MC_CONST(0.03);
    state.foc.hfi.injection_positive = true;
    state.foc.flying.count = 5U;
    state.foc.flying.completed = true;
    state.foc.identification.state = IDENTIFICATION_IQ_DONE;
    state.foc.identification.coefficients.ad0 = MC_CONST(0.75);
    state.foc.identification.sample_count = 19U;
    Experiment.Complete = true;
    Experiment.LLS.ad0 = 8.125F;
    Experiment.Id_buf[0] = 3.25F;
    Sensorless_Enabled = true;
    Sensorless_Method = HF_INJECTION;
    Leso_Enabled = Hfi_Enabled = true;
    MainInt_UseRealTheta = false;
    Sensorless_ThetAdj = 0.375F; /* Valid last CAN write not yet read by Step. */
    PLLKPMAX = 150.0F;
    PLLKIMAX = 4500.0F;
    Sensorless_Theta_PID.Kp = 99.0F;
    Leso_Gain = 9.0F;
    Hfi_InjVolt = 27.0F;
    for (unsigned pass = 0U; pass < 3U; ++pass)
    {
        state.foc.mtpa_rebuild_pending = true;
        state.foc.mtpa_rebuild_in_progress = true;
        state.foc.mtpa_staging_ready = true;
        state.foc.mtpa_rebuild_failed = true;
        state.foc.mtpa_staging.table_valid = true;
        state.foc.mtpa_staging.parameters.ad0 = MC_CONST(99.0);
        FixedControl_Reinitialize(&state);
        if (!Sensorless_Enabled || MainInt_UseRealTheta
            || Sensorless_ThetAdj != 0.375F
            || state.foc.config.sensorless.theta_adjustment_pu
                != McMath_FromFloat(0.375F * MC_INV_TWO_PI_F)
            || Sensorless_Method != HF_INJECTION || !Leso_Enabled || !Hfi_Enabled
            || PLLKPMAX != 150.0F || PLLKIMAX != 4500.0F)
            return 33;
        if (state.foc.sensorless.angle_pu != MC_CONST(0.375)
            || state.foc.sensorless.speed_pu != MC_CONST(0.15)
            || state.foc.sensorless.speed_sum != 12345
            || state.foc.sensorless.speed_count != 3U
            || state.foc.sensorless.angle_product_residual != 37
            || state.foc.sensorless.common_pid.integral != MC_ZERO
            || state.foc.sensorless.integral_product_residual != 0
            || state.foc.sensorless.speed_filter.x1 != MC_ZERO
            || state.foc.sensorless.speed_filter.y1 != MC_ZERO)
            return 34;
        if (state.foc.leso.current_estimate.a != MC_CONST(0.2)
            || state.foc.leso.disturbance.b != MC_CONST(-0.3)
            || state.foc.leso.emf_estimate.a != MC_CONST(0.04)
            || state.foc.hfi.previous_current.a != MC_CONST(0.08)
            || state.foc.hfi.previous_high_frequency.b != MC_CONST(0.03)
            || !state.foc.hfi.injection_positive
            || state.foc.flying.count != 5U || !state.foc.flying.completed)
            return 35;
        if (state.foc.mtpa_rebuild_pending || state.foc.mtpa_rebuild_in_progress
            || state.foc.mtpa_staging_ready || state.foc.mtpa_rebuild_failed
            || state.foc.mtpa_staging.table_valid
            || memcmp(&accepted, &state.foc.mtpa, sizeof(accepted)) != 0
            || state.foc.inductance_dq_pu.d != MC_CONST(0.6)
            || state.foc.inductance_dq_pu.q != MC_CONST(0.2))
            return 36;
        if (fabsf(Sensorless_Theta_PID.Kp - SENSORLESS_PLL_KP) > 0.001F
            || Leso_Gain != LESO_WC_GAIN || Hfi_InjVolt != HF_INJECTION_AMP
            || state.foc.identification.state != IDENTIFICATION_IQ_DONE
            || state.foc.identification.sample_count != 19U
            || !Experiment.Complete || Experiment.LLS.ad0 != 8.125F
            || Experiment.Id_buf[0] != 3.25F)
            return 37;
        /* Invalid pending writes must fall back to the prior accepted value,
         * not contaminate warm INIT's newly reset rollback cache. */
        Sensorless_ThetAdj = NAN;
        PLLKPMAX = INFINITY;
        PLLKIMAX = -1.0F;
    }
    Sensorless_ThetAdj = 0.375F;
    PLLKPMAX = 150.0F;
    PLLKIMAX = 4500.0F;
    FixedControlInput_t input = {.bus_voltage = 400.0F, .reset = true};
    MainInt_State = RUNNING;
    for (unsigned cycle = 0U; cycle < 4U; ++cycle)
        (void)FixedControl_Step(&state, &input);
    if (state.foc.identification.state != IDENTIFICATION_IQ_DONE
        || state.foc.identification.sample_count != 19U
        || !Experiment.Complete || Experiment.LLS.ad0 != 8.125F
        || memcmp(&accepted, &state.foc.mtpa, sizeof(accepted)) != 0)
        return 38;
    /* A retained experiment result must not force later non-IDENTIFY modes
     * back to IDLE. No HFI injection is needed for this mode-only check. */
    Sensorless_Enabled = false;
    MainInt_UseRealTheta = true;
    Foc_Mode = SPEED;
    input.reset = false;
    (void)FixedControl_Step(&state, &input);
    if (Foc_Mode != SPEED
        || state.foc.identification.state != IDENTIFICATION_IQ_DONE
        || memcmp(&accepted, &state.foc.mtpa, sizeof(accepted)) != 0)
        return 39;
    state.foc.identification.state = IDENTIFICATION_IQ_FAILED;
    (void)FixedControl_Step(&state, &input);
    if (Foc_Mode != SPEED) return 40;

    state.foc.mtpa.table_valid = false;
    FixedControl_Reinitialize(&state);
    if (state.foc.mtpa.table_valid) return 41; /* No hidden default fallback. */
    FixedControl_Init(&state);
    if (!state.foc.mtpa.table_valid || Sensorless_Enabled
        || Sensorless_ThetAdj != 0.0F
        || memcmp(&defaults, &state.foc.mtpa.parameters, sizeof(defaults)) != 0)
        return 42;
    puts("PASS: cold/warm INIT, accepted MTPA/results, observer history and invalid-write rollback");
    return 0;
}

static int gateway_scan_regression(void)
{
    const struct { int start, final, step, count, last; } cases[] = {
        {3, 12, 1, 10, 12}, {12, 3, -1, 10, 3},
        {3, 22, 1, 20, 22}, {3, 12, 2, 5, 11},
        {12, 3, -2, 5, 4}};
    for (unsigned n = 0U; n < sizeof(cases) / sizeof(cases[0]); ++n)
    {
        FixedControlState_t state;
        FixedControl_Init(&state);
        MainInt_State = RUNNING;
        Foc_Mode = IDENTIFY;
        FixedControlInput_t input = {.bus_voltage = 400.0F, .reset = true};
        (void)FixedControl_Step(&state, &input);
        Experiment.start_I = cases[n].start;
        Experiment.final_I = cases[n].final;
        Experiment.step_dir = cases[n].step;
        (void)FixedControl_Step(&state, &input);
        state.foc.identification.rs_complete = true;
        Experiment.state = NEXT_I;
        input.reset = false;
        (void)FixedControl_Step(&state, &input);
        unsigned count = 0U;
        float last = 0.0F;
        while (state.foc.identification.state == IDENTIFICATION_IQ_INJECT_COLLECT
               && count <= IDENTIFICATION_IQ_MAX_STEPS)
        {
            int expected = cases[n].start + (int)count * cases[n].step;
            last = McMath_ToFloat(state.foc.identification.current_target_pu)
                 * MC_CURRENT_BASE_A;
            if (fabsf(last - (float)expected) > 0.00002F
                || state.foc.identification.current_target_a != expected
                || Experiment.inj.State)
                return 50;
            count++;
            /* Only sample completion is injected: the real gateway and core
             * must schedule the next point and retain the manual pause. */
            state.foc.identification.state = IDENTIFICATION_IQ_NEXT_CURRENT;
            (void)FixedControl_Step(&state, &input);
        }
        if ((int)count != cases[n].count
            || fabsf(last - (float)cases[n].last) > 0.00002F
            || state.foc.identification.state != IDENTIFICATION_IQ_LLS)
            return 51;
    }
    puts("PASS: real A2L gateway scans, both directions, endpoints, capacity and non-divisible steps");
    return 0;
}

static int gateway_mtpa_fallback_regression(void)
{
    FixedControlState_t state;
    FixedControl_Init(&state);
    MainInt_State = RUNNING;
    Sensorless_Enabled = false;
    MainInt_UseRealTheta = true;
    Foc_Mode = SPEED;
    Foc_Speed_Ref = 600.0F;
    state.foc.mtpa.table_valid = false;
    FixedControlInput_t input = {.bus_voltage = 400.0F, .reset = false};
    unsigned cycles = state.foc.config.startup_hold_cycles
                    + 2U * state.foc.config.speed_prescaler;
    for (unsigned n = 0U; n < cycles; ++n)
        (void)FixedControl_Step(&state, &input);
    if (!Mtpa_LinearFallback || !state.foc.mtpa_linear_fallback
        || fabsf(Foc_Id_Ref - fabsf(Foc_Iq_Ref)) > 0.00002F)
        return 52;
    Mtpa_LinearFallback = false; /* This is an output, never a command. */
    (void)FixedControl_Step(&state, &input);
    if (!Mtpa_LinearFallback || !state.foc.mtpa_linear_fallback)
        return 53;
    Foc_Mode = IDLE;
    (void)FixedControl_Step(&state, &input);
    if (Mtpa_LinearFallback || state.foc.mtpa_linear_fallback)
        return 54;
    puts("PASS: MTPA fallback A2L mirror, output-only writes and IDLE clearing");
    return 0;
}

int main(void)
{
    Foc_Mode = IDLE;
    if (!FixedControl_ModeRequestsStop())
        return 1;
    for (int mode = VF_MODE; mode <= IDENTIFY; ++mode)
    {
        Foc_Mode = (FocMode_t)mode;
        if (FixedControl_ModeRequestsStop())
            return 2;
    }
    Foc_Mode = (FocMode_t)99;
    if (!FixedControl_ModeRequestsStop())
        return 3;
    Foc_Mode = IDLE;
    FixedControlState_t state;
    FixedControl_Init(&state);
    FixedControlInput_t input = {
        .current_abc = {2.3456F, -1.0F, -1.3456F},
        .theta_rad = 0.2F,
        .speed_rpm = 0.0F,
        .bus_voltage = 397.1234F,
        .reset = true};
    FixedControlOutput_t output = FixedControl_Step(&state, &input);
    Foc_Ramp_Speed_Handler.slope = 120.0F;
    (void)FixedControl_Step(&state, &input);
    if (state.foc.config.speed_ramp_step_pu
        != McMath_FromFloat(120.0F * SPEED_LOOP_TIME / MC_SPEED_BASE_RPM))
        return 12;
    Foc_Ramp_Speed_Handler.slope = NAN;
    (void)FixedControl_Step(&state, &input);
    if (Foc_Ramp_Speed_Handler.slope != 120.0F)
        return 13;
    if (output.bus_voltage_q24_raw != McMath_FromFloat(
            input.bus_voltage / MC_VOLTAGE_BASE_V)
        || output.phase_a_current_q24_raw != McMath_FromFloat(
            input.current_abc.a / MC_CURRENT_BASE_A))
        return 4;
    if (fabsf(output.bus_voltage_q24_v - input.bus_voltage) > 0.001F
        || fabsf(output.phase_a_current_q24_a - input.current_abc.a)
            > 0.0001F)
        return 5;

    Foc_Mode = IDENTIFY;
    input.reset = true;
    (void)FixedControl_Step(&state, &input);
    if (!Experiment.Initialized || Experiment.state != WAIT
        || Experiment.start_I != 3 || Experiment.final_I != 12
        || Experiment.inj.State)
        return 6;

    Experiment.start_I = 4;
    Experiment.final_I = 6;
    Experiment.step_dir = 1;
    Experiment.step_dir = 2;
    (void)FixedControl_Step(&state, &input);
    if (state.foc.identification.config.current_step_pu
        != McMath_FromFloat(2.0F / MC_CURRENT_BASE_A)
        || !state.foc.identification.config.integer_current_scan
        || state.foc.identification.config.current_start_a != 4
        || state.foc.identification.config.current_final_a != 6
        || state.foc.identification.config.current_step_a != 2) return 24;
    Experiment.step_dir = 1;
    Experiment.sample_capacity = 64;
    Experiment.repeat_times = 1;
    Experiment.inj.Ud_amp = 80.0F;
    Experiment.inj.Uq_amp = 0.0F;
    Experiment.state = EST_RS;
    input.reset = false;
    (void)FixedControl_Step(&state, &input);
    if (state.foc.identification.state != IDENTIFICATION_IQ_EST_RS
        || !state.foc.identification.config.manual_control
        || state.foc.identification.config.current_start_pu
           != McMath_FromFloat(4.0F / MC_CURRENT_BASE_A)
        || !state.foc.identification.config.integer_current_scan
        || state.foc.identification.config.current_start_a != 4
        || state.foc.identification.config.current_final_a != 6
        || state.foc.identification.config.current_step_a != 1
        || state.foc.identification.config.injection_voltage_d_pu
           != McMath_FromFloat(80.0F / MC_VOLTAGE_BASE_V))
        return 7;

    /* Bypass the slow Rs plant portion and verify the legacy NEXT_I +
     * inj.State gate at the physical-unit boundary. */
    state.foc.identification.state = IDENTIFICATION_IQ_WAIT;
    state.foc.identification.rs_complete = true;
    Experiment.state = NEXT_I;
    Experiment.inj.State = false;
    output = FixedControl_Step(&state, &input);
    if (state.foc.identification.state != IDENTIFICATION_IQ_INJECT_COLLECT
        || state.foc.identification.point_started
        || fabsf(output.voltage_dq.d) > 0.001F)
        return 8;

    Experiment.inj.Ud_amp = 80.0F;
    Experiment.inj.State = true;
    output = FixedControl_Step(&state, &input);
    if (!state.foc.identification.point_started
        || fabsf(output.voltage_dq.d - 80.0F) > 0.01F
        || fabsf(output.voltage_dq.q) > 0.01F)
        return 9;

    state.foc.identification.error = IDENTIFICATION_IQ_SINGULAR_LLS;
    state.foc.identification.state = IDENTIFICATION_IQ_FAILED;
    (void)FixedControl_Step(&state, &input);
    if (Foc_Mode != IDLE || !FixedControl_ModeRequestsStop()
        || Identification_Error != IDENTIFICATION_IQ_SINGULAR_LLS)
        return 10;
    (void)FixedControl_Step(&state, &input);
    if (Identification_Error != IDENTIFICATION_IQ_SINGULAR_LLS
        || Experiment.state != WAIT)
        return 11;

    /* Restore the old master flag, not the erroneous method bit mask. */
    FixedControl_Init(&state);
    MainInt_State = RUNNING;
    Foc_Mode = VF_MODE;
    input.reset = false;
    Sensorless_Enabled = false;
    Hfi_Enabled = true;
    output = FixedControl_Step(&state, &input);
    if (!state.foc.hfi.enabled || fabsf(output.voltage_dq.d) > 0.001F)
        return 14;
    bool sign_before = state.foc.hfi.injection_positive;
    Sensorless_Enabled = true;
    output = FixedControl_Step(&state, &input);
    if (state.foc.hfi.injection_positive == sign_before
        || fabsf(fabsf(output.voltage_dq.d) - HF_INJECTION_AMP) > 0.01F
        || state.foc.leso.pll.enabled || state.foc.hfi.pll.enabled)
        return 15;
    sign_before = state.foc.hfi.injection_positive;
    input.reset = true;
    output = FixedControl_Step(&state, &input);
    if (state.foc.hfi.injection_positive != sign_before
        || fabsf(output.voltage_dq.d) > 0.001F
        || fabsf(output.pwm_duty.a - 0.5F) > 0.00001F)
        return 16;
    Foc_Mode = SPEED;
    Foc_Speed_Ref = 600.0F;
    (void)FixedControl_Step(&state, &input);
    if (Foc_Speed_Ref != 0.0F) return 17;
    MainInt_State = SENSORLESS;
    MainInt_PrepareMode();
    if (Foc_Mode != SPEED || MainInt_ModeRequestsStop()) return 18;
    MainInt_State = EXIT;
    MainInt_PrepareMode();
    if (Foc_Mode != IDLE || !FixedControl_ModeRequestsStop()) return 19;
    MainInt_State = (DeviceStateEnum_t)99;
    MainInt_PrepareMode();
    if (MainInt_State != EXIT || !MainInt_ModeRequestsStop()) return 20;
    MainInt_State = INIT;
    MainInt_PrepareMode();
    if (Foc_Mode != IDLE || !MainInt_ModeRequestsStop()) return 21;
    MainInt_State = RUNNING;
    Foc_Mode = IF_MODE;
    Sensorless_Enabled = false;
    input.reset = true;
    Foc_Pid_CurD_Handler.MinOutput = 600.0F;
    Foc_Pid_CurD_Handler.MaxOutput = -600.0F;
    (void)FixedControl_Step(&state, &input);
    if (Foc_Pid_CurD_Handler.MinOutput != PID_CURRENT_D_LOOP_MIN_OUTPUT
        || Foc_Pid_CurD_Handler.MaxOutput != PID_CURRENT_D_LOOP_MAX_OUTPUT) return 22;
    Foc_Ramp_Speed_Handler.limit_min = -900.0F;
    Foc_Ramp_Speed_Handler.limit_max = 900.0F;
    (void)FixedControl_Step(&state, &input);
    if (state.foc.config.speed_ramp_maximum_pu != MC_CONST(0.5)
        || state.foc.config.speed_ramp_minimum_pu != MC_CONST(-0.5)) return 23;
    int warm_result = warm_init_regression();
    if (warm_result != 0)
    {
        fprintf(stderr, "FAIL warm INIT regression: %d\n", warm_result);
        return warm_result;
    }
    int scan_result = gateway_scan_regression();
    if (scan_result != 0)
    {
        fprintf(stderr, "FAIL gateway scan regression: %d\n", scan_result);
        return scan_result;
    }
    int mtpa_result = gateway_mtpa_fallback_regression();
    if (mtpa_result != 0)
    {
        fprintf(stderr, "FAIL MTPA gateway regression: %d\n", mtpa_result);
        return mtpa_result;
    }
    puts("PASS: A2L mode, ADC Q24, identification gateway and legacy sensorless master gating");
    return 0;
}
