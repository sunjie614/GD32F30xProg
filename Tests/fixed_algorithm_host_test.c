#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "motor_iq.h"
#include "mtpa_iq.h"
#include "pid_iq.h"
#include "protect_iq.h"
#include "signal_iq.h"
#include "transformation_iq.h"

static unsigned Failures;
static float MtpaPoint25Id;
static float MtpaPoint25Iq;
static float MtpaPoint50Id;
static float MtpaPoint50Iq;

static float real_to_float(mc_real_t value)
{
    return McMath_ToFloat(value);
}

static void expect_near(const char* name, float actual, float expected,
                        float tolerance)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance)
    {
        fprintf(stderr, "FAIL %s: actual=%g expected=%g tolerance=%g\n",
                name, actual, expected, tolerance);
        Failures++;
    }
}

static void test_transforms(void)
{
    PhaseIq_t phase = {MC_ONE, MC_CONST(-0.5), MC_CONST(-0.5)};
    ClarkIq_t clarke = TransformIq_Clarke(phase);
    expect_near("clarke.a", real_to_float(clarke.a), 1.0F, 0.00001F);
    expect_near("clarke.b", real_to_float(clarke.b), 0.0F, 0.00001F);
    ParkIq_t park = TransformIq_Park(clarke, MC_CONST(0.25));
    expect_near("park.d", real_to_float(park.d), 0.0F, 0.00002F);
    expect_near("park.q", real_to_float(park.q), -1.0F, 0.00002F);
    ClarkIq_t inverse = TransformIq_InversePark(park, MC_CONST(0.25));
    expect_near("inverse.a", real_to_float(inverse.a),
                real_to_float(clarke.a), 0.00002F);
    expect_near("inverse.b", real_to_float(inverse.b),
                real_to_float(clarke.b), 0.00002F);
}

static void test_state_helpers(void)
{
    expect_near("division.negative", real_to_float(McMath_Div(
                    MC_CONST(-0.5), MC_CONST(0.25))),
                -2.0F, 0.000001F);
#if defined(MC_NUMERIC_IQMATH)
    if (McMath_FromFloat(NAN) != MC_ZERO
        || McMath_FromFloat(INFINITY) != MC_ZERO
        || McMath_Diagnostics.invalid_input_count != 2U)
    {
        fprintf(stderr, "FAIL invalid float boundary handling\n");
        Failures++;
    }
    McMath_ResetDiagnostics();
#endif

    RampIqState_t ramp;
    RampIq_Init(&ramp, MC_ZERO);
    expect_near("ramp.up", real_to_float(RampIq_Run(
                    &ramp, MC_ONE, MC_CONST(0.1))),
                0.1F, 0.000001F);
    RampIq_Reset(&ramp, MC_ZERO);
    expect_near("ramp.down", real_to_float(RampIq_Run(
                    &ramp, McMath_Neg(MC_ONE), MC_CONST(0.2))),
                -0.2F, 0.000001F);

    PhaseGeneratorIqState_t phase;
    PhaseGeneratorIq_Init(&phase, MC_CONST(0.95));
    expect_near("phase.wrap", real_to_float(PhaseGeneratorIq_Run(
        &phase, MC_CONST(0.10), MC_ZERO, true)), 0.05F, 0.000002F);

    PidIqState_t pid;
    PidIq_Init(&pid, MC_ONE, MC_CONST(0.1), MC_ZERO,
               MC_CONST(-0.5), MC_CONST(0.5), MC_CONST(0.5));
    mc_real_t output = PidIq_Run(&pid, MC_ONE);
    expect_near("pid.limit", real_to_float(output), 0.5F, 0.000001F);
    if (McMath_Diagnostics.saturation_count != 1U)
    {
        fprintf(stderr, "FAIL pid saturation diagnostic\n");
        Failures++;
    }
    McMath_ResetDiagnostics();
}

static void test_mtpa(void)
{
    MtpaIqState_t state;
    MtpaIq_Init(&state, NULL);
    if (!MtpaIq_BuildTable(&state, MC_ZERO, MC_ONE) || !state.table_valid)
    {
        fprintf(stderr, "FAIL mtpa table did not build\n");
        Failures++;
        return;
    }
    mc_real_t previous_iq = McMath_Neg(MC_ONE);
    for (unsigned index = 0U; index < MTPA_IQ_TABLE_POINTS; ++index)
    {
        const MtpaIqPoint_t* point = &state.table[index];
        if (!point->valid || !isfinite(real_to_float(point->id_pu))
            || !isfinite(real_to_float(point->iq_pu))
            || point->iq_pu < previous_iq)
        {
            fprintf(stderr,
                    "FAIL mtpa point %u: valid=%u id=%g iq=%g previous_iq=%g torque=%g\n",
                    index, point->valid ? 1U : 0U,
                    real_to_float(point->id_pu),
                    real_to_float(point->iq_pu),
                    real_to_float(previous_iq),
                    real_to_float(point->torque_pu));
            Failures++;
            return;
        }
        previous_iq = point->iq_pu;
    }
    mc_real_t id = MtpaIq_InterpolateIdByIq(
        &state, state.table[25].iq_pu);
    expect_near("mtpa.interpolate", real_to_float(id),
                real_to_float(state.table[25].id_pu), 0.002F);
    MtpaPoint25Id = real_to_float(state.table[25].id_pu);
    MtpaPoint25Iq = real_to_float(state.table[25].iq_pu);
    MtpaPoint50Id = real_to_float(state.table[50].id_pu);
    MtpaPoint50Iq = real_to_float(state.table[50].iq_pu);
    /* FLOAT_REF golden values; Q24 must remain inside the 0.1% full-scale
     * coordinate-transform/modulation acceptance band. */
    expect_near("mtpa.p25.id", MtpaPoint25Id, 0.23353294F, 0.001F);
    expect_near("mtpa.p25.iq", MtpaPoint25Iq, 0.39811099F, 0.001F);
    expect_near("mtpa.p50.id", MtpaPoint50Id, 0.34727588F, 0.001F);
    expect_near("mtpa.p50.iq", MtpaPoint50Iq, 0.70115435F, 0.001F);
}

static void test_motor_and_protect(void)
{
    MotorIqConfig_t motor_config = {
        .speed_prescaler = 1U,
        .position_modulus = 65536U,
        .position_offset = 0U,
        .pole_pairs = MC_CONST(2.0),
        .speed_filter_alpha = MC_ZERO,
        .speed_scale = MC_CONST(2.0)};
    MotorIqState_t motor;
    MotorIq_Init(&motor, &motor_config);
    MotorIq_RunPosition(&motor, 16384U);
    expect_near("motor.theta_mechanical",
                real_to_float(MotorIq_GetMechanicalAngle(&motor)),
                0.25F, 0.000001F);
    expect_near("motor.theta_electrical",
                real_to_float(MotorIq_GetElectricalAngle(&motor)),
                0.50F, 0.000001F);
    expect_near("motor.speed", real_to_float(MotorIq_GetSpeed(&motor)),
                0.50F, 0.000001F);

    ProtectIqConfig_t protect_config = {
        .bus_rate_pu = MC_CONST(0.5),
        .bus_fluctuation_pu = MC_CONST(0.1),
        .bus_low_pu = MC_CONST(0.05),
        .current_max_pu = MC_CONST(0.8),
        .temperature_max_pu = MC_CONST(0.5),
        .average_current_ratio = MC_CONST(0.9),
        .fan_on_ratio = MC_CONST(0.4),
        .fan_off_ratio = MC_CONST(0.3),
        .average_current_cycles = 2U};
    ProtectIqState_t protect;
    ProtectIq_Init(&protect, &protect_config, No_Protect);
    PhaseIq_t warning_current = {MC_CONST(0.75), MC_ZERO, MC_ZERO};
    if (ProtectIq_RunPhaseCurrent(&protect, warning_current)
        || ProtectIq_RunPhaseCurrent(&protect, warning_current)
        || !ProtectIq_RunPhaseCurrent(&protect, warning_current)
        || (protect.flags & Over_Avg_Current) == 0U)
        Failures++;
    ProtectIq_Reset(&protect);
    if (!ProtectIq_RunPhaseCurrent(&protect,
            (PhaseIq_t){MC_CONST(0.81), MC_ZERO, MC_ZERO})
        || (protect.flags & Over_Max_Current) == 0U)
        Failures++;
    ProtectIq_Reset(&protect);
    if (!ProtectIq_RunBusVoltage(&protect, MC_CONST(0.7))
        || (protect.flags & Over_Voltage) == 0U)
        Failures++;
    ProtectIq_Reset(&protect);
    if (!ProtectIq_RunBusVoltage(&protect, MC_CONST(0.01))
        || (protect.flags & Low_Voltage) == 0U)
        Failures++;
    ProtectIq_Reset(&protect);
    if (!ProtectIq_RunTemperature(&protect, MC_CONST(0.6))
        || (protect.flags & Over_Heat) == 0U)
        Failures++;
    ProtectIq_Reset(&protect);
    ProtectIq_RunHardwareFault(&protect, false);
    if ((protect.flags & Hardware_Fault) == 0U
        || !ProtectIq_RunFan(&protect, MC_CONST(0.25))
        || ProtectIq_RunFan(&protect, MC_CONST(0.10)))
        Failures++;
}

int main(void)
{
    McMath_ResetDiagnostics();
    test_transforms();
    test_state_helpers();
    test_mtpa();
    test_motor_and_protect();
    if (McMath_Diagnostics.saturation_count != 0U
        || McMath_Diagnostics.divide_by_zero_count != 0U
        || McMath_Diagnostics.invalid_input_count != 0U)
    {
        fprintf(stderr, "FAIL unexpected numeric diagnostics: %u/%u/%u\n",
                McMath_Diagnostics.saturation_count,
                McMath_Diagnostics.divide_by_zero_count,
                McMath_Diagnostics.invalid_input_count);
        Failures++;
    }
    if (Failures != 0U)
        return 1;
    printf("PASS: fixed algorithm host regression "
           "p25=(%.8f,%.8f) p50=(%.8f,%.8f)\n",
           MtpaPoint25Id, MtpaPoint25Iq, MtpaPoint50Id, MtpaPoint50Iq);
    return 0;
}
