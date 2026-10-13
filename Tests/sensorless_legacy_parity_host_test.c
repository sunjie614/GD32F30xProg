/* Execute the retained original float sources beside the explicit-state IQ
 * sources. The authorized bidirectional gain schedule intentionally differs
 * from the old negative-speed schedule: the test-only reference adapters below
 * apply abs ONLY to scheduling, and the authorized IQ Wc ceiling is 3990.
 * The old production sources (including their 4000 default) remain unchanged.
 * The host IQmath mock checks rounding/units/control sequencing; it does not
 * prove the target TI archive timing or motor-loop stability. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sensorless_iq.h"
#include "fixed_numeric_config.h"
#include "parameters.h"

/* Keep the reference's real CMSIS trig macros even though the other source
 * files in this executable select Q24. Preload its headers once as float. */
#undef MC_NUMERIC_IQMATH
#include "leso.h"
#include "hf_injection.h"
#include "filter.h"
#include "sensorless_interface.h"
#define MC_NUMERIC_IQMATH 1
#ifndef M_PI_4
#define M_PI_4 (M_PI * 0.25F)
#endif

/* Including the old sources exposes their A2L/private parameters to this
 * test only. Rename the two same-named internal phase-detector helpers. */
#define calculate_error legacy_leso_calculate_error
#include "../Sensorless/Src/leso.c"
#undef calculate_error
#define calculate_error legacy_hfi_calculate_error
#include "../Sensorless/Src/hf_injection.c"
#undef calculate_error
#include "../Sensorless/Src/flying.c"
#include "../Middleware/Sensorless_Interface/Src/sensorless_interface.c"

void Buffer_Put(float value, size_t index)
{
    (void)value;
    (void)index;
}

static float physical(mc_real_t value, float base)
{
    return McMath_ToFloat(value) * base;
}

static mc_real_t normalized(float value, float base)
{
    return McMath_FromFloat(value / base);
}

static float angular_error(float legacy_rad, mc_real_t angle_pu)
{
    return fabsf(wrap_theta_pi(legacy_rad
        - physical(angle_pu, M_2PI))) * 180.0F / M_PI;
}

static void maximum(float* value, float candidate)
{
    if (candidate > *value)
        *value = candidate;
}

static void initialize_legacy(void)
{
    SystemTimeConfig_t timing = {
        .current = {MAIN_LOOP_TIME, MAIN_LOOP_FREQ},
        .speed = {SPEED_LOOP_TIME, SPEED_LOOP_FREQ},
        .prescaler = SPEED_LOOP_PRESCALER};
    LESO_Param_t leso = {.wc_gain = LESO_WC_GAIN, .wc_max = LESO_WC_MAX,
        .wc_min = LESO_WC_MIN, .Rs = MOTOR_RS, .Ld = MOTOR_LD, .Lq = MOTOR_LQ};
    Sensorless_Param_t common = {.switch_speed = SENSORLESS_SWITCH_SPEED,
        .hysteresis = SENSORLESS_HYSTERESIS};
    MotorParam_t motor = {.inv_MotorPn = 1.0F / MOTOR_PN};
    PID_Handler_t pid = {.Kp = SENSORLESS_PLL_KP, .Ki = SENSORLESS_PLL_KI,
        .Kd = SENSORLESS_PLL_KD, .MaxOutput = SENSORLESS_PLL_MAX_OUTPUT,
        .MinOutput = SENSORLESS_PLL_MIN_OUTPUT,
        .IntegralLimit = SENSORLESS_PLL_INTEGRAL_LIMIT, .Ts = MAIN_LOOP_TIME};
    hf_injection_params_t hfi = {.injection_voltage = HF_INJECTION_AMP,
        .Ld = MOTOR_LD, .Lq = MOTOR_LQ, .inv_Pn = 1.0F / MOTOR_PN};
    assert(Leso_Set_SampleTime(&timing));
    assert(Leso_Initialization(&leso));
    Leso_Set_Pn(MOTOR_PN);
    Leso_Set_Inductor((Park_t){MOTOR_LD, MOTOR_LQ});
    assert(Hfi_Set_SampleTime(&timing));
    assert(Hfi_Initialization(&hfi));
    assert(Sensorless_Set_SampleTime(&timing));
    assert(Sensorless_Initialization(&common));
    assert(Sensorless_Set_MotorParams(&motor));
    assert(Sensorless_Set_PidParams(&pid));
    assert(Sensorless_Set_SpeedFilter(10.0F, SPEED_LOOP_FREQ));
}

static AngleResult_t legacy_position_with_symmetric_schedule(void)
{
    /* Execute the actual old PLL/filter/selection first. Its gain update is
     * after that update and affects the NEXT tick, exactly as in IQ. Positive
     * motion remains a direct comparison with the unmodified old function. */
    AngleResult_t result = Sensorless_Update_Position();
    if (Sensorless_Enabled && result.speed < -500.0F)
    {
        assert(Sensorless_Theta_PID.Kp == 50.0F);
        assert(Sensorless_Theta_PID.Ki == 625.0F);
        /* Explicit intentional negative-speed difference, not legacy parity. */
        Sensorless_PLL_Wc = SENSORLESS_PLL_WC + (fabsf(result.speed) - 500.0F) * 0.2F;
        Sensorless_Theta_PID.Kp = fminf(2.0F * Sensorless_PLL_Wc, 130.0F);
        Sensorless_Theta_PID.Ki = fminf(Sensorless_PLL_Wc * Sensorless_PLL_Wc, 4225.0F);
    }
    return result;
}

static void legacy_beta_with_symmetric_schedule(void)
{
    float signed_we = Leso_We;
    float original_maximum = Leso_Wc_Max;
    /* Explicit test-reference change: IQ now caps Wc at 3990, while the old
     * production float source/configuration remains at 4000. */
    Leso_Wc_Max = fminf(original_maximum, (float)LESO_IQ_WC_MAX_RAD_S);
    Leso_Update_Beta();
    if (signed_we < 0.0F)
    {
        assert(Leso_Wc == Leso_Wc_Min); /* the old reverse-only minimum */
        /* Only the bandwidth input changes. Restore signed We before the
         * original observer and phase detector execute. */
        Leso_We = fabsf(signed_we);
        Leso_Update_Beta();
        Leso_We = signed_we;
    }
    Leso_Wc_Max = original_maximum;
}

static void check_common_pll(SensorlessIqState_t* state,
    SensorlessIqConfig_t* config, LesoIqState_t* leso, HfiIqState_t* hfi)
{
    float angle_max = 0.0F, speed_max = 0.0F, omega_max = 0.0F;
    float real_angle = 0.0F;
    bool saw_adaptive = false, saw_reverse_adaptive = false;
    for (unsigned n = 0; n < 20000U; ++n)
    {
        bool enabled = !(n >= 15000U && n < 15100U);
        bool reset = n >= 12000U && n < 12100U;
        float actual_omega = n < 4000U ? 0.1F * n : n < 8000U ? 400.0F
            : n < 14000U ? 400.0F - 0.1F * (n - 8000U) : -200.0F;
        real_angle = wrap_theta_2pi(real_angle + actual_omega * MAIN_LOOP_TIME);
        /* Close the phase-detector feedback independently around each PLL.
         * Equal physical excitation, not an artificial held .9 error. */
        float le = sinf(real_angle - Sensorless_ThetaEst);
        float fixed_error = sinf(real_angle - physical(state->angle_pu, M_2PI));
        float he = le;
        float target = n < 6500U ? 900.0F : n < 12000U ? -900.0F : 400.0F;
        float reference = n < 6500U ? 700.0F : n < 12000U ? -700.0F : 300.0F;
        float feedback = n < 6500U ? 700.0F : n < 12000U ? -700.0F : 300.0F;
        Sensorless_Enabled = enabled;
        Sensorless_Set_ResetFlag(reset);
        Sensorless_Set_SpeedTarget(target);
        Sensorless_Set_SpeedRef(reference);
        Sensorless_Set_SpeedFdbk(feedback);
        Leso_Error = le;
        Hfi_Error = he;
        leso->phase_error_pu = McMath_FromFloat(fixed_error);
        hfi->phase_error_pu = McMath_FromFloat(fixed_error);
        SensorlessIqInput_t input = {.enabled = enabled, .reset = reset,
            .requested_method = state->method,
            .speed_target_pu = normalized(target, MC_SPEED_BASE_RPM),
            .speed_reference_pu = normalized(reference, MC_SPEED_BASE_RPM),
            .speed_feedback_pu = normalized(feedback, MC_SPEED_BASE_RPM)};
        AngleResult_t old = legacy_position_with_symmetric_schedule();
        SensorlessIqOutput_t out = {0};
        SensorlessIq_UpdatePosition(state, config, &input, leso, hfi, &out);
        assert((unsigned)Sensorless_Get_Method() == (unsigned)out.method);
        maximum(&angle_max, angular_error(old.theta, out.angle_pu));
        maximum(&speed_max, fabsf(old.speed - physical(out.speed_pu, MC_SPEED_BASE_RPM)));
        maximum(&omega_max, fabsf(Sensorless_Theta_PID.output
            - physical(out.omega_pu, SENSORLESS_IQ_OMEGA_BASE_RAD_S)));
        if (angular_error(Leso_Get_Result().theta, leso->pll.angle_pu) >= 0.5F)
            fprintf(stderr, "angle fail n=%u max=%f speedmax=%f omegamax=%f Ki=%f vs %f\n",
                n, angle_max, speed_max, omega_max, Sensorless_Theta_PID.Ki,
                physical(state->common_pid.ki_step, SENSORLESS_IQ_OMEGA_BASE_RAD_S / MAIN_LOOP_TIME));
        assert(angular_error(Leso_Get_Result().theta, leso->pll.angle_pu) < 0.5F);
        assert(angular_error(Hfi_Get_Result().theta, hfi->pll.angle_pu) < 0.5F);
        assert(hfi->pll.speed_pu == MC_ZERO); /* the local PLL is inactive */
        if (Sensorless_Theta_PID.Kp > 50.0F)
        {
            saw_adaptive = true;
            if (old.speed < -500.0F)
                saw_reverse_adaptive = true;
        }
        assert(fabsf(Sensorless_Theta_PID.Kp
            - physical(state->common_pid.kp, SENSORLESS_IQ_OMEGA_BASE_RAD_S)) < 0.3F);
        float ki_error = fabsf(Sensorless_Theta_PID.Ki - physical(state->common_pid.ki_step,
            SENSORLESS_IQ_OMEGA_BASE_RAD_S / MAIN_LOOP_TIME));
        if (ki_error >= 22.0F)
            fprintf(stderr, "n=%u speed=%f vs %f wc=%f vs %f Ki=%f vs %f\n", n,
                old.speed, physical(out.speed_pu, MC_SPEED_BASE_RPM), Sensorless_PLL_Wc,
                physical(state->pll_wc_pu, SENSORLESS_IQ_OMEGA_BASE_RAD_S),
                Sensorless_Theta_PID.Ki, physical(state->common_pid.ki_step,
                    SENSORLESS_IQ_OMEGA_BASE_RAD_S / MAIN_LOOP_TIME));
        assert(ki_error < 22.0F); /* 0.5% of the original Ki maximum */
    }
    assert(saw_adaptive);
    assert(saw_reverse_adaptive);
    assert(angle_max < 0.5F);
    assert(speed_max < MC_SPEED_BASE_RPM * 0.005F);
    assert(omega_max < 0.5F);
    printf("common PLL (reverse schedule corrected in test reference): angle %.6f deg, speed %.6f rpm, omega %.6f rad/s\n",
           angle_max, speed_max, omega_max);
}

static void check_common_pll_integral_limits(void)
{
    /* Exercise the active public PLL, not the inactive local HFI/LESO PLLs.
     * Compare against the real old Pid_Update: integral clamp, conditional
     * integration, output clamp, release and Reset are separate behaviors. */
    const struct {
        float kp, limit, integral, error;
        bool reset;
    } vectors[] = {
        {0.0F, 700.0F, 699.99F, 1.0F, false},
        {0.0F, 700.0F, -699.99F, -1.0F, false},
        {50.0F, 700.0F, 660.0F, 1.0F, false},
        {50.0F, 700.0F, -660.0F, -1.0F, false},
        {50.0F, 700.0F, 700.0F, -1.0F, false},
        {50.0F, 700.0F, -700.0F, 1.0F, false},
        {0.0F, 250.0F, 249.99F, 1.0F, false},
        {0.0F, 250.0F, -249.99F, -1.0F, false},
        {50.0F, 700.0F, 700.0F, 1.0F, true},
        {50.0F, 700.0F, -700.0F, -1.0F, true}
    };
    for (unsigned n = 0; n < sizeof(vectors) / sizeof(vectors[0]); ++n)
    {
        SensorlessIqConfig_t config;
        SensorlessIqState_t state;
        LesoIqState_t leso = {0};
        HfiIqState_t hfi = {0};
        SensorlessIq_DefaultConfig(&config);
        assert(fabsf(physical(config.integral_limit,
            SENSORLESS_IQ_OMEGA_BASE_RAD_S) - 700.0F) < 0.0001F);
        config.kp = normalized(vectors[n].kp, SENSORLESS_IQ_OMEGA_BASE_RAD_S);
        config.integral_limit = normalized(vectors[n].limit,
                                           SENSORLESS_IQ_OMEGA_BASE_RAD_S);
        SensorlessIq_Init(&state, &config);
        state.common_pid.integral = normalized(vectors[n].integral,
                                               SENSORLESS_IQ_OMEGA_BASE_RAD_S);
        leso.phase_error_pu = hfi.phase_error_pu = McMath_FromFloat(vectors[n].error);
        SensorlessIqInput_t input = {.enabled = true, .reset = vectors[n].reset,
            .requested_method = SENSORLESS_IQ_LES_OBSERVER,
            .speed_target_pu = normalized(900.0F, MC_SPEED_BASE_RPM),
            .speed_reference_pu = normalized(900.0F, MC_SPEED_BASE_RPM),
            .speed_feedback_pu = normalized(900.0F, MC_SPEED_BASE_RPM)};
        PID_Handler_t reference = {.Kp = vectors[n].kp,
            .Ki = SENSORLESS_PLL_KI, .Ts = MAIN_LOOP_TIME,
            .MinOutput = SENSORLESS_PLL_MIN_OUTPUT,
            .MaxOutput = SENSORLESS_PLL_MAX_OUTPUT,
            .IntegralLimit = vectors[n].limit, .integral = vectors[n].integral};
        float expected_output = Pid_Update(vectors[n].error, vectors[n].reset,
                                           &reference);
        SensorlessIqOutput_t output = {0};
        SensorlessIq_UpdatePosition(&state, &config, &input, &leso, &hfi, &output);
        assert(fabsf(physical(state.common_pid.integral,
            SENSORLESS_IQ_OMEGA_BASE_RAD_S) - reference.integral) < 0.0002F);
        assert(fabsf(physical(output.omega_pu, SENSORLESS_IQ_OMEGA_BASE_RAD_S)
            - expected_output) < 0.0002F);
        assert(McMath_Abs(state.common_pid.integral) <= config.integral_limit);
        assert(output.omega_pu >= config.minimum && output.omega_pu <= config.maximum);
    }
    puts("common PLL: +/-700 rad/s integral/output limits and legacy anti-windup PASS");
}

static void check_hfi(SensorlessIqState_t* common, HfiIqState_t* state,
                      const HfiIqConfig_t* config)
{
    float current_max = 0.0F, phase_max = 0.0F;
    for (unsigned n = 0; n < 4000U; ++n)
    {
        bool enabled = !(n >= 1000U && n < 1100U);
        bool reset = n >= 2000U && n < 2100U;
        float theta = wrap_theta_2pi(0.003F * n);
        Clark_t current = {2.0F * cosf(theta) + (n % 2U ? 0.5F : -0.5F),
                           3.0F * sinf(theta) + (n % 2U ? 0.3F : -0.3F)};
        ClarkIq_t raw = {normalized(current.a, MC_CURRENT_BASE_A),
                         normalized(current.b, MC_CURRENT_BASE_A)};
        Sensorless_Enabled = enabled;
        common->enabled = enabled;
        Hfi_Set_Enabled(true);
        state->enabled = true;
        Sensorless_Set_ResetFlag(reset);
        Hfi_Set_Theta(theta);
        state->pll.angle_pu = normalized(theta, M_2PI);
        Clark_t old = Sensorless_FilterCurrent(current);
        ClarkIq_t out = SensorlessIq_FilterCurrent(common, state, raw, enabled, reset);
        maximum(&current_max, fabsf(old.a - physical(out.a, MC_CURRENT_BASE_A)));
        maximum(&current_max, fabsf(old.b - physical(out.b, MC_CURRENT_BASE_A)));
        maximum(&current_max, fabsf(Hfi_IClarkResp.a
            - physical(state->response_current.a, MC_CURRENT_BASE_A)));
        maximum(&current_max, fabsf(Hfi_IClarkResp.b
            - physical(state->response_current.b, MC_CURRENT_BASE_A)));
        if (!reset)
            Hfi_Update();
        HfiIqInput_t input = {.reset = reset};
        HfiIqOutput_t result;
        HfiIq_Run(state, config, &input, &result);
        maximum(&phase_max, fabsf(Hfi_Get_PllErr() - physical(result.phase_error_pu, 1.0F)));
        Park_t old_voltage = Sensorless_Inject_Voltage((Park_t){2.0F, 3.0F});
        ParkIq_t voltage = SensorlessIq_InjectVoltage(common, config, state,
            (ParkIq_t){normalized(2.0F, MC_VOLTAGE_BASE_V),
                       normalized(3.0F, MC_VOLTAGE_BASE_V)}, reset);
        assert(fabsf(old_voltage.d - physical(voltage.d, MC_VOLTAGE_BASE_V)) < 0.0001F);
        assert(Hfi_InjectSign == state->injection_positive);
    }
    assert(current_max < MC_CURRENT_BASE_A * 0.001F);
    assert(phase_max < 0.001F);
    printf("HFI: base/response %.6f A, projection %.6f\n", current_max, phase_max);
}

static void check_leso(LesoIqState_t* state, const LesoIqConfig_t* config)
{
    float current_max = 0.0F, emf_max = 0.0F, phase_max = 0.0F;
    for (unsigned n = 0; n < 6000U; ++n)
    {
        bool enabled = !(n >= 1500U && n < 1550U);
        bool reset = n >= 3500U && n < 3550U;
        float theta = wrap_theta_2pi(0.025F * n);
        float speed = n < 2000U ? 300.0F : n < 4000U ? 900.0F : -300.0F;
        float lq = n < 2000U ? 0.016F : n < 4000U ? 0.022F
            : n < 5000U ? 0.001F : n < 5500U ? 0.0F : -0.002F;
        Clark_t current = {3.0F * cosf(theta), 4.0F * sinf(theta)};
        Clark_t voltage = {20.0F * cosf(theta + 0.15F), 30.0F * sinf(theta + 0.15F)};
        Leso_Set_Enabled(enabled);
        Leso_Set_Theta(theta);
        Leso_Set_Speed(speed);
        Leso_Set_Inductor((Park_t){MOTOR_LD, lq});
        Leso_Set_Voltage(voltage);
        Leso_Set_Current(current);
        if (enabled && !reset)
        {
            legacy_beta_with_symmetric_schedule();
            Leso_Update_EmfEstA();
            Leso_Update_EmfEstB();
            Leso_Update();
        }
        state->enabled = enabled;
        LesoIqInput_t input = {.reset = reset,
            .voltage_ab_pu = {normalized(voltage.a, MC_VOLTAGE_BASE_V),
                             normalized(voltage.b, MC_VOLTAGE_BASE_V)},
            .current_ab_pu = {normalized(current.a, MC_CURRENT_BASE_A),
                             normalized(current.b, MC_CURRENT_BASE_A)},
            .angle_pu = normalized(theta, M_2PI),
            .speed_pu = normalized(speed, MC_SPEED_BASE_RPM),
            .inductance_q_pu = normalized(lq, MC_FLUX_BASE_WB / MC_CURRENT_BASE_A)};
        LesoIqOutput_t out;
        LesoIq_Run(state, config, &input, &out);
        if (enabled && !reset)
            assert(fabsf(Leso_Lq - physical(state->last_inductance_q_pu,
                MC_FLUX_BASE_WB / MC_CURRENT_BASE_A)) < 0.000001F);
        maximum(&current_max, fabsf(Leso_CurEst.a - physical(state->current_estimate.a, MC_CURRENT_BASE_A)));
        maximum(&current_max, fabsf(Leso_CurEst.b - physical(state->current_estimate.b, MC_CURRENT_BASE_A)));
        maximum(&emf_max, fabsf(Leso_EmfEst.a - physical(out.emf_ab_pu.a, MC_VOLTAGE_BASE_V)));
        maximum(&emf_max, fabsf(Leso_EmfEst.b - physical(out.emf_ab_pu.b, MC_VOLTAGE_BASE_V)));
        maximum(&phase_max, fabsf(Leso_Get_PllErr() - physical(out.phase_error_pu, 1.0F)));
        if (enabled && !reset)
            assert(fabsf(Leso_Wc - physical(state->bandwidth_step, 1.0F / MAIN_LOOP_TIME)) < 0.01F);
    }
    assert(current_max < MC_CURRENT_BASE_A * 0.001F);
    assert(emf_max < MC_VOLTAGE_BASE_V * 0.001F);
    assert(phase_max < 0.001F);
    printf("LESO (reverse schedule corrected in test reference): current %.6f A, EMF %.6f V, projection %.6f\n", current_max, emf_max, phase_max);
}

static void check_bidirectional_gain_schedule(void)
{
    SensorlessIqConfig_t config;
    LesoIqConfig_t leso_config;
    SensorlessIq_DefaultConfig(&config);
    LesoIq_DefaultConfig(&leso_config);
    const mc_real_t magnitudes[] = {
        MC_ZERO, normalized(100.0F, MC_SPEED_BASE_RPM),
        config.dynamic_speed_threshold_pu - 1,
        config.dynamic_speed_threshold_pu,
        config.dynamic_speed_threshold_pu + 1,
        normalized(600.0F, MC_SPEED_BASE_RPM),
        normalized(1800.0F, MC_SPEED_BASE_RPM),
        normalized(4000.0F, MC_SPEED_BASE_RPM)};
    for (unsigned n = 0U; n < sizeof(magnitudes) / sizeof(magnitudes[0]); ++n)
    {
        SensorlessIqState_t positive, negative;
        LesoIqState_t leso[2];
        HfiIqState_t hfi[2] = {0};
        SensorlessIqOutput_t result[2];
        LesoIqOutput_t observer[2];
        SensorlessIq_Init(&positive, &config);
        SensorlessIq_Init(&negative, &config);
        SensorlessIqState_t* pair[] = {&positive, &negative};
        for (unsigned side = 0U; side < 2U; ++side)
        {
            mc_real_t speed = side == 0U ? magnitudes[n] : -magnitudes[n];
            LesoIq_Init(&leso[side], &leso_config);
            leso[side].enabled = true;
            /* Identical physical excitation/state: bandwidth and EMF must be
             * identical; the signed phase detector alone must reverse. */
            LesoIqInput_t observer_input = {
                .current_ab_pu = {MC_CONST(0.03), MC_CONST(-0.02)},
                .voltage_ab_pu = {MC_CONST(0.01), MC_CONST(0.02)},
                .angle_pu = MC_CONST(0.1), .speed_pu = speed,
                .inductance_q_pu = leso_config.inductance_q_pu};
            LesoIq_Run(&leso[side], &leso_config, &observer_input, &observer[side]);
            assert(leso[side].pll.speed_pu == speed);
            pair[side]->speed_pu = speed;
            pair[side]->pll_wc_pu = MC_CONST(0.031); /* stale Wc is retained below 500 */
            pair[side]->common_pid.integral = side == 0U ? MC_CONST(0.1) : MC_CONST(-0.1);
            leso[side].phase_error_pu = hfi[side].phase_error_pu =
                side == 0U ? MC_CONST(0.2) : MC_CONST(-0.2);
            SensorlessIqInput_t input = {.enabled = true,
                .requested_method = SENSORLESS_IQ_LES_OBSERVER,
                .speed_target_pu = speed, .speed_reference_pu = speed,
                .speed_feedback_pu = speed};
            /* count=0, so this tick schedules the held filtered speed. */
            SensorlessIq_UpdatePosition(pair[side], &config, &input,
                &leso[side], &hfi[side], &result[side]);
            assert(result[side].speed_pu == speed);
        }
        assert(positive.common_pid.kp == negative.common_pid.kp);
        assert(positive.common_pid.ki_step == negative.common_pid.ki_step);
        assert(positive.pll_wc_pu == negative.pll_wc_pu);
        assert(result[0].omega_pu == -result[1].omega_pu);
        assert(leso[0].bandwidth_step == leso[1].bandwidth_step);
        assert(observer[0].emf_ab_pu.a == observer[1].emf_ab_pu.a);
        assert(observer[0].emf_ab_pu.b == observer[1].emf_ab_pu.b);
        assert(observer[0].phase_error_pu != MC_ZERO);
        assert(observer[0].phase_error_pu == (magnitudes[n] == MC_ZERO
            ? observer[1].phase_error_pu : -observer[1].phase_error_pu));
        float rpm = physical(magnitudes[n], MC_SPEED_BASE_RPM);
        float expected_wc = fminf(LESO_IQ_WC_MAX_RAD_S, fmaxf(LESO_WC_MIN,
            LESO_WC_GAIN * 1.9F * rpm * MOTOR_PN * M_2PI / 60.0F));
        assert(fabsf(expected_wc - physical(leso[0].bandwidth_step,
            1.0F / MAIN_LOOP_TIME)) < 0.01F);
        if (magnitudes[n] <= config.dynamic_speed_threshold_pu)
        {
            assert(positive.common_pid.kp == config.low_speed_kp);
            assert(positive.common_pid.ki_step == config.low_speed_ki_step);
            assert(positive.pll_wc_pu == MC_CONST(0.031));
        }
        else
        {
            float wc = SENSORLESS_PLL_WC + (rpm - 500.0F) * 0.2F;
            assert(fabsf(physical(positive.pll_wc_pu,
                SENSORLESS_IQ_OMEGA_BASE_RAD_S) - wc) < 0.001F);
            assert(fabsf(physical(positive.common_pid.kp,
                SENSORLESS_IQ_OMEGA_BASE_RAD_S) - fminf(2.0F * wc, 130.0F)) < 0.002F);
            assert(fabsf(physical(positive.common_pid.ki_step,
                SENSORLESS_IQ_OMEGA_BASE_RAD_S / MAIN_LOOP_TIME)
                - fminf(wc * wc, 4225.0F)) < 0.4F);
        }
        if (rpm >= 1800.0F)
        {
            assert(positive.common_pid.kp == config.kp_maximum);
            assert(positive.common_pid.ki_step == config.ki_step_maximum);
            assert(leso[0].bandwidth_step == leso_config.bandwidth_maximum_step);
        }
    }

    /* Persistent paired states follow mirrored measured-speed trajectories.
     * Hold the speed-filter output for this scheduler unit test; the 20000-tick
     * legacy test above exercises the real ten-sample filter and reversal. */
    SensorlessIqState_t positive, negative;
    LesoIqState_t running_leso[2];
    SensorlessIq_Init(&positive, &config);
    SensorlessIq_Init(&negative, &config);
    for (unsigned side = 0U; side < 2U; ++side)
    {
        LesoIq_Init(&running_leso[side], &leso_config);
        running_leso[side].enabled = true;
    }
    const float trajectory[] = {1800.0F, 600.0F, 500.0F, 499.0F, 0.0F,
        -499.0F, -500.0F, -600.0F, -1800.0F, -600.0F, 0.0F, 600.0F};
    for (unsigned n = 0U; n < sizeof(trajectory) / sizeof(trajectory[0]); ++n)
    {
        SensorlessIqState_t* pair[] = {&positive, &negative};
        for (unsigned side = 0U; side < 2U; ++side)
        {
            mc_real_t speed = normalized(trajectory[n], MC_SPEED_BASE_RPM);
            mc_real_t error = MC_CONST(0.2);
            if (side != 0U) { speed = -speed; error = -error; }
            pair[side]->speed_pu = speed;
            pair[side]->speed_count = 0U;
            pair[side]->speed_sum = 0;
            mc_real_t previous_kp = pair[side]->common_pid.kp;
            mc_real_t previous_integral = pair[side]->common_pid.integral;
            LesoIqState_t leso = {.phase_error_pu = error};
            HfiIqState_t hfi = {.phase_error_pu = error};
            SensorlessIqInput_t input = {.enabled = true,
                .requested_method = pair[side]->method,
                .speed_target_pu = speed, .speed_reference_pu = speed,
                .speed_feedback_pu = speed};
            SensorlessIqOutput_t output;
            SensorlessIq_UpdatePosition(pair[side], &config, &input, &leso, &hfi, &output);
            /* The current output must still use the PREVIOUS gain. Changing
             * speed direction/magnitude must not move scheduling ahead of PI. */
            assert(output.omega_pu == McMath_Add(McMath_Mul(previous_kp, error),
                pair[side]->common_pid.integral));
            assert(side == 0U ? pair[side]->common_pid.integral >= previous_integral
                             : pair[side]->common_pid.integral <= previous_integral);
            assert(output.speed_pu == speed && leso.pll.speed_pu == speed);
        }
        assert(positive.common_pid.kp == negative.common_pid.kp);
        assert(positive.common_pid.ki_step == negative.common_pid.ki_step);
        assert(positive.pll_wc_pu == negative.pll_wc_pu);
        assert(positive.omega_pu == -negative.omega_pu);
        assert(positive.common_pid.integral == -negative.common_pid.integral);
        assert(positive.method == negative.method);
        for (unsigned side = 0U; side < 2U; ++side)
        {
            mc_real_t speed = normalized(trajectory[n], MC_SPEED_BASE_RPM);
            LesoIqInput_t input = {
                .current_ab_pu = {MC_CONST(0.03), MC_CONST(-0.02)},
                .voltage_ab_pu = {MC_CONST(0.01), MC_CONST(0.02)},
                .angle_pu = MC_CONST(0.1),
                .speed_pu = side == 0U ? speed : -speed,
                .inductance_q_pu = leso_config.inductance_q_pu};
            LesoIqOutput_t output;
            LesoIq_Run(&running_leso[side], &leso_config, &input, &output);
        }
        assert(running_leso[0].bandwidth_step == running_leso[1].bandwidth_step);
        assert(running_leso[0].current_estimate.a == running_leso[1].current_estimate.a);
        assert(running_leso[0].current_estimate.b == running_leso[1].current_estimate.b);
        assert(running_leso[0].disturbance.a == running_leso[1].disturbance.a);
        assert(running_leso[0].disturbance.b == running_leso[1].disturbance.b);
        assert(running_leso[0].emf_estimate.a == running_leso[1].emf_estimate.a);
        assert(running_leso[0].emf_estimate.b == running_leso[1].emf_estimate.b);
        assert(running_leso[0].phase_error_pu == (trajectory[n] == 0.0F
            ? running_leso[1].phase_error_pu : -running_leso[1].phase_error_pu));
    }
    puts("bidirectional schedules: 8 +/- boundary pairs, 12 mirrored running steps, phase sign and prior-gain timing PASS");
}

static void check_leso_hard_bandwidth_ceiling(void)
{
    LesoIqConfig_t config;
    LesoIq_DefaultConfig(&config);
    mc_real_t ceiling = LesoIq_MaximumBandwidthStep();
    /* Compare raw integers against the exact timer ratio, not a float display
     * that could round an excessive Q24 step back down to '3990.000'. */
    uint64_t ticks = (uint64_t)(MAIN_INT_TIMER_PRESCALER + 1U)
                     * MAIN_INT_TIMER_PERIOD * 2U;
    uint64_t allowed = (uint64_t)LESO_IQ_WC_MAX_RAD_S
        * ((uint64_t)1 << MC_Q_FRACTIONAL_BITS) * ticks;
    assert((uint64_t)ceiling * MCU_MAIN_FREQ <= allowed);
    assert(((uint64_t)ceiling + 1U) * MCU_MAIN_FREQ > allowed);
    assert(config.bandwidth_maximum_step == ceiling);
    /* Bypass the gateway deliberately, including min>max and an excessive
     * minimum: the core ceiling must still win over every configurable bound. */
    const struct { float minimum, maximum; } limits[] = {
        {600.0F, 3990.0F}, {600.0F, 4000.0F}, {600.0F, 4200.0F},
        {600.0F, 5000.0F}, {4500.0F, 5000.0F}, {5000.0F, 600.0F}};
    for (unsigned n = 0U; n < sizeof(limits) / sizeof(limits[0]); ++n)
    for (unsigned sign = 0U; sign < 2U; ++sign)
    {
        LesoIqState_t state;
        config.bandwidth_minimum_step = McMath_FromFloat(limits[n].minimum * MAIN_LOOP_TIME);
        config.bandwidth_maximum_step = McMath_FromFloat(limits[n].maximum * MAIN_LOOP_TIME);
        LesoIq_Init(&state, &config);
        state.enabled = true;
        state.current_estimate.a = normalized(1.0F, MC_CURRENT_BASE_A);
        LesoIqInput_t input = {.speed_pu = sign == 0U ? MC_ONE : -MC_ONE,
            .inductance_q_pu = config.inductance_q_pu};
        LesoIqOutput_t output;
        uint32_t old_saturation = McMath_Diagnostics.saturation_count;
        for (unsigned sample = 0U; sample < 200U; ++sample)
        {
            LesoIq_Run(&state, &config, &input, &output);
            assert(state.bandwidth_step == ceiling);
        }
        assert(fabsf(physical(state.current_estimate.a, MC_CURRENT_BASE_A)) < 0.001F);
        assert(McMath_Diagnostics.saturation_count == old_saturation);
    }
    puts("LESO 3990 ceiling: exact floor, 12 direct-config signed cases, zero-input decay PASS");
}

static void check_strategy_and_flying(SensorlessIqState_t* common,
    const SensorlessIqConfig_t* config, LesoIqState_t* leso,
    const LesoIqConfig_t* leso_config, HfiIqState_t* hfi,
    const HfiIqConfig_t* hfi_config)
{
    FlyingIqState_t flying;
    FlyingIq_Init(&flying, 8U);
    Flying_Set_Enabled(true);
    for (unsigned n = 0U; n < 8U; ++n)
    {
        Flying_Update(false);
        FlyingIq_Run(&flying, false);
        assert(Flying_Is_Completed() == flying.completed);
        assert(flying.completed == (n == 7U));
    }
    Flying_Set_Enabled(false);
    FlyingIq_SetEnabled(&flying, false);
    assert(Flying_Is_Completed() == flying.completed);
    Flying_Set_Enabled(true);
    FlyingIq_SetEnabled(&flying, true);
    assert(Flying_Count == flying.count && !flying.completed);
    const float levels[] = {0.0F, 330.0F, 340.0F, 400.0F, 460.0F, 470.0F, -500.0F};
    common->angle_pu = normalized(Hfi_Theta, M_2PI);
    Leso_Enabled = leso->enabled = false;
    Hfi_Enabled = hfi->enabled = false;
    for (unsigned r = 0U; r < sizeof(levels) / sizeof(levels[0]); ++r)
    for (unsigned f = 0U; f < sizeof(levels) / sizeof(levels[0]); ++f)
    {
        bool reset = r == 3U && f == 3U;
        Sensorless_Set_ResetFlag(reset);
        Sensorless_Set_SpeedRef(levels[r]);
        Sensorless_Set_SpeedFdbk(levels[f]);
        Sensorless_Set_Current((Clark_t){2.0F, 3.0F});
        Sensorless_Set_Voltage((Clark_t){4.0F, 5.0F});
        SensorlessIqInput_t input = {.enabled = false, .reset = reset,
            .speed_reference_pu = normalized(levels[r], MC_SPEED_BASE_RPM),
            .speed_feedback_pu = normalized(levels[f], MC_SPEED_BASE_RPM),
            .current_ab_pu = {normalized(2.0F, MC_CURRENT_BASE_A), normalized(3.0F, MC_CURRENT_BASE_A)},
            .voltage_ab_pu = {normalized(4.0F, MC_VOLTAGE_BASE_V), normalized(5.0F, MC_VOLTAGE_BASE_V)},
            .inductance_q_pu = leso_config->inductance_q_pu};
        Sensorless_Calculate();
        SensorlessIq_Calculate(common, config, &input, leso_config,
                              hfi_config, leso, hfi, &flying);
        assert(Leso_Get_Enabled() == leso->enabled);
        assert(Hfi_Get_Enabled() == hfi->enabled);
        assert(fabsf(Leso_Current.a - physical(leso->last_current_ab_pu.a, MC_CURRENT_BASE_A)) < 0.0001F);
        assert(fabsf(Leso_Voltage.a - physical(leso->last_voltage_ab_pu.a, MC_VOLTAGE_BASE_V)) < 0.0001F);
    }
    /* HFI initial-position alignment also happens while common PLL disabled. */
    Hfi_Enabled = hfi->enabled = false;
    Sensorless_Set_ResetFlag(false);
    Sensorless_Set_SpeedRef(0.0F);
    Sensorless_Set_SpeedFdbk(0.0F);
    Sensorless_ThetaEst = 3.0F;
    Hfi_Theta = 0.0F;
    common->angle_pu = normalized(3.0F, M_2PI);
    hfi->pll.angle_pu = MC_ZERO;
    SensorlessIqInput_t alignment = {.inductance_q_pu = leso_config->inductance_q_pu};
    Sensorless_Calculate();
    SensorlessIq_Calculate(common, config, &alignment, leso_config,
                          hfi_config, leso, hfi, &flying);
    assert(angular_error(Hfi_Theta, hfi->pll.angle_pu) < 0.0001F);
    puts("strategy/reset/capture gates and eight-cycle flying wait: PASS");
}

int main(void)
{
    SensorlessIqConfig_t config;
    LesoIqConfig_t leso_config;
    HfiIqConfig_t hfi_config;
    SensorlessIqState_t common;
    LesoIqState_t leso;
    HfiIqState_t hfi;
    SensorlessIq_DefaultConfig(&config);
    LesoIq_DefaultConfig(&leso_config);
    HfiIq_DefaultConfig(&hfi_config);
    SensorlessIq_Init(&common, &config);
    LesoIq_Init(&leso, &leso_config);
    HfiIq_Init(&hfi, &hfi_config);
    initialize_legacy();
    check_common_pll(&common, &config, &leso, &hfi);
    check_hfi(&common, &hfi, &hfi_config);
    check_leso(&leso, &leso_config);
    check_strategy_and_flying(&common, &config, &leso, &leso_config, &hfi, &hfi_config);
    check_common_pll_integral_limits();
    check_bidirectional_gain_schedule();
    check_leso_hard_bandwidth_ceiling();
    assert(McMath_Diagnostics.divide_by_zero_count == 0U);
    assert(McMath_Diagnostics.invalid_input_count == 0U);
    puts("PASS: sensorless positive legacy parity + intentional symmetric reverse schedules (host Q24 mock).");
    return 0;
}
