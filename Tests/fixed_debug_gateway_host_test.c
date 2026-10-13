#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fixed_debug_gateway.h"
#include "fixed_numeric_config.h"

static unsigned Failures;

static void expect_true(const char* name, bool condition)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL %s\n", name);
        Failures++;
    }
}

static void expect_close(const char* name, float actual, float expected)
{
    if (!isfinite(actual) || fabsf(actual - expected) > 0.0002F)
    {
        fprintf(stderr, "FAIL %s: actual=%.8f expected=%.8f\n",
                name, actual, expected);
        Failures++;
    }
}

static void observer_mirrors_are_physical_and_read_only(void)
{
    FocIqState_t state = {0};
    FocIqInput_t input = {
        .measured_angle_pu = MC_CONST(0.99),
        .measured_speed_pu = MC_CONST(-0.2)};
    FocIqOutput_t output = {
        .estimated_angle_pu = MC_CONST(0.01),
        .estimated_speed_pu = MC_CONST(0.5),
        .inductance_dq_pu = {MC_CONST(0.3), MC_CONST(0.6)}};
    state.sensorless.angle_pu = output.estimated_angle_pu;
    state.current_ab_pu = (ClarkIq_t){MC_CONST(0.2), MC_CONST(-0.15)};
    state.voltage_reference_ab_pu = (ClarkIq_t){MC_CONST(0.25), MC_CONST(-0.05)};
    state.leso.last_current_ab_pu = state.current_ab_pu;
    state.leso.last_voltage_ab_pu = state.voltage_reference_ab_pu;
    state.leso.pll.angle_pu = MC_CONST(0.02);
    state.leso.pll.speed_pu = MC_CONST(0.25);
    state.leso.current_estimate = (ClarkIq_t){MC_CONST(0.12), MC_CONST(-0.1)};
    state.leso.emf_estimate = (ClarkIq_t){MC_CONST(0.04), MC_CONST(-0.025)};
    state.leso.emf_dq = (ParkIq_t){MC_CONST(0.06), MC_CONST(-0.07)};
    state.leso.emf_filtered = (ParkIq_t){MC_CONST(0.02), MC_CONST(0.015)};
    state.leso.emf_slow_filtered = (ParkIq_t){MC_CONST(0.01), MC_CONST(-0.005)};
    state.hfi.pll.angle_pu = MC_CONST(0.97);
    state.hfi.pll.speed_pu = MC_CONST(-0.75);
    state.hfi.response_current = (ClarkIq_t){MC_CONST(0.03), MC_CONST(-0.02)};
    state.hfi.base_current = (ClarkIq_t){MC_CONST(0.1), MC_CONST(-0.08)};
    /* Different snapshots ensure the gateway uses its documented sources:
     * observer current/voltage mirrors are state, MTPA inductance is output. */
    output.current_ab_pu = (ClarkIq_t){MC_CONST(0.8), MC_CONST(0.9)};
    output.voltage_reference_ab_pu = (ClarkIq_t){MC_CONST(0.7), MC_CONST(0.6)};
    unsigned char saved_state[sizeof(state)];
    unsigned char saved_input[sizeof(input)];
    unsigned char saved_output[sizeof(output)];
    memcpy(saved_state, &state, sizeof(state));
    memcpy(saved_input, &input, sizeof(input));
    memcpy(saved_output, &output, sizeof(output));

    for (unsigned publication = 0U; publication < 2U; ++publication)
    {
        /* Simulate old watch-list writes: publication refreshes the mirrors,
         * never imports them into a controller, observer or input snapshot. */
        Sensorless_ThetaEst = 123.0F;
        Leso_Current.a = 456.0F;
        Hfi_IClarkResp.b = 789.0F;
        MTPA_Inductor.d = 12.0F;
        FixedDebug_Publish(&state, &input, &output);

        expect_close("Sensorless theta rad", Sensorless_ThetaEst,
                     0.01F * MC_TWO_PI_F);
        expect_close("Sensorless speed rpm", Sensorless_SpeedEst, 900.0F);
        expect_close("Sensorless short angle error deg", Sensorless_ThetaErr, -7.2F);
        expect_close("Sensorless speed error rpm", Sensorless_SpeedErr, -1260.0F);
        expect_close("LESO theta rad", Leso_Theta, 0.02F * MC_TWO_PI_F);
        expect_close("LESO speed rpm", Leso_Speed, 450.0F);
        expect_close("LESO short angle error deg", Leso_Theta_Err, -10.8F);
        expect_close("LESO speed error rpm", Leso_Speed_Err, -810.0F);
        expect_close("HFI theta rad", Hfi_Theta, 0.97F * MC_TWO_PI_F);
        expect_close("HFI speed rpm", Hfi_Speed, -1350.0F);
        expect_close("HFI short angle error deg", Hfi_Theta_Err, 7.2F);
        expect_close("HFI speed error rpm", Hfi_Speed_Err, 990.0F);
        expect_close("LESO estimated current alpha A", Leso_CurEst.a, 3.6F);
        expect_close("LESO estimated current beta A", Leso_CurEst.b, -3.0F);
        expect_close("LESO current alpha A", Leso_Current.a, 6.0F);
        expect_close("LESO current beta A", Leso_Current.b, -4.5F);
        expect_close("LESO voltage alpha V", Leso_Voltage.a, 200.0F);
        expect_close("LESO voltage beta V", Leso_Voltage.b, -40.0F);
        expect_close("LESO emf alpha V", Leso_EmfEst.a, 32.0F);
        expect_close("LESO emf beta V", Leso_EmfEst.b, -20.0F);
        expect_close("LESO emf d V", Leso_EmfEst_dq.d, 48.0F);
        expect_close("LESO emf q V", Leso_EmfEst_dq.q, -56.0F);
        expect_close("LESO filtered emf d V", Leso_Emf_Filtered.d, 16.0F);
        expect_close("LESO filtered emf q V", Leso_Emf_Filtered.q, 12.0F);
        expect_close("LESO slow filtered emf d V", Leso_Emf_Slow_Filtered.d, 8.0F);
        expect_close("LESO slow filtered emf q V", Leso_Emf_Slow_Filtered.q, -4.0F);
        expect_close("HFI current feedback alpha A", Hfi_IClarkFdbk.a, 6.0F);
        expect_close("HFI current feedback beta A", Hfi_IClarkFdbk.b, -4.5F);
        expect_close("HFI response current alpha A", Hfi_IClarkResp.a, 0.9F);
        expect_close("HFI response current beta A", Hfi_IClarkResp.b, -0.6F);
        expect_close("HFI filtered current alpha A", Hfi_IClarkFilt.a, 3.0F);
        expect_close("HFI filtered current beta A", Hfi_IClarkFilt.b, -2.4F);
        expect_close("MTPA inductance d H", MTPA_Inductor.d, 0.01F);
        expect_close("MTPA inductance q H", MTPA_Inductor.q, 0.02F);
        expect_true("debug publish preserves complete core state",
            memcmp(saved_state, &state, sizeof(state)) == 0);
        expect_true("debug publish preserves input snapshot",
            memcmp(saved_input, &input, sizeof(input)) == 0);
        expect_true("debug publish preserves output snapshot",
            memcmp(saved_output, &output, sizeof(output)) == 0);
    }

    /* Reverse the zero crossing: do not show a nearly full-revolution error. */
    input.measured_angle_pu = MC_CONST(0.01);
    output.estimated_angle_pu = MC_CONST(0.99);
    state.sensorless.angle_pu = output.estimated_angle_pu;
    FixedDebug_Publish(&state, &input, &output);
    expect_close("positive zero-crossing angle error", Sensorless_ThetaErr, 7.2F);

    Sensorless_ThetaEst = 123.0F;
    FixedDebug_Publish(NULL, &input, &output);
    FixedDebug_Publish(&state, NULL, &output);
    FixedDebug_Publish(&state, &input, NULL);
    expect_close("null snapshot publish leaves mirrors untouched",
                 Sensorless_ThetaEst, 123.0F);
}

static void mtpa_mirror_is_torque_sorted_without_reordering_core(void)
{
    MtpaIqState_t state = {0};
    state.table_valid = true;
    for (unsigned index = 0U; index < MTPA_IQ_TABLE_POINTS; ++index)
    {
        state.table[index] = (MtpaIqPoint_t){
            .torque_pu = McMath_FromFloat(
                (float)(MTPA_IQ_TABLE_POINTS - 1U - index) * 0.01F),
            .flux_pu = McMath_FromFloat(0.2F + (float)index * 0.002F),
            .gamma_pu = McMath_FromFloat(0.1F + (float)index * 0.001F),
            .id_pu = McMath_FromFloat(-0.1F - (float)index * 0.002F),
            .iq_pu = McMath_FromFloat((float)index * 0.01F),
            .ld_pu = McMath_FromFloat(0.3F + (float)index * 0.002F),
            .lq_pu = McMath_FromFloat(0.6F + (float)index * 0.003F),
            .valid = index != 17U};
    }
    unsigned char saved_state[sizeof(state)];
    memcpy(saved_state, &state, sizeof(state));
    for (unsigned publication = 0U; publication < 2U; ++publication)
    {
        mtpa_table[0].T_req = 999.0F;
        FixedDebug_PublishMtpa(&state);
        expect_true("MTPA publish preserves core and Iq lookup ordering",
            memcmp(saved_state, &state, sizeof(state)) == 0);
        for (unsigned index = 0U; index < MTPA_IQ_TABLE_POINTS; ++index)
        {
            const MtpaIqPoint_t* source =
                &state.table[MTPA_IQ_TABLE_POINTS - 1U - index];
            expect_close("MTPA table requested torque Nm", mtpa_table[index].T_req,
                McMath_ToFloat(source->torque_pu) * MC_TORQUE_BASE_NM);
            expect_close("MTPA table flux Wb", mtpa_table[index].Psi_s,
                McMath_ToFloat(source->flux_pu) * MC_FLUX_BASE_WB);
            expect_close("MTPA table angle rad", mtpa_table[index].gamma,
                McMath_ToFloat(source->gamma_pu) * MC_TWO_PI_F);
            expect_close("MTPA table d current A", mtpa_table[index].Id,
                McMath_ToFloat(source->id_pu) * MC_CURRENT_BASE_A);
            expect_close("MTPA table q current A", mtpa_table[index].Iq,
                McMath_ToFloat(source->iq_pu) * MC_CURRENT_BASE_A);
            expect_close("MTPA table d inductance H", mtpa_table[index].Ld,
                McMath_ToFloat(source->ld_pu) * MC_FLUX_BASE_WB / MC_CURRENT_BASE_A);
            expect_close("MTPA table q inductance H", mtpa_table[index].Lq,
                McMath_ToFloat(source->lq_pu) * MC_FLUX_BASE_WB / MC_CURRENT_BASE_A);
            expect_true("MTPA table retains invalid point flag",
                mtpa_table[index].valid == source->valid);
            if (index != 0U)
                expect_true("MTPA mirror is increasing requested torque",
                    mtpa_table[index - 1U].T_req <= mtpa_table[index].T_req);
        }
    }
    expect_close("MTPA first mirror requested torque Nm", mtpa_table[0].T_req, 0.0F);
    expect_close("MTPA last mirror requested torque Nm",
        mtpa_table[MTPA_IQ_TABLE_POINTS - 1U].T_req, 25.0F);
    mtpa_table[0].T_req = 999.0F;
    FixedDebug_PublishMtpa(NULL);
    expect_close("null MTPA publish leaves mirror untouched", mtpa_table[0].T_req, 999.0F);
}

int main(void)
{
    observer_mirrors_are_physical_and_read_only();
    mtpa_mirror_is_torque_sorted_without_reordering_core();
    expect_true("normal debug publication has no arithmetic fault",
        McMath_Diagnostics.saturation_count == 0U
        && McMath_Diagnostics.divide_by_zero_count == 0U
        && McMath_Diagnostics.invalid_input_count == 0U);
    if (Failures != 0U)
        return 1;
    puts("PASS: legacy observer/MTPA physical debug mirrors are read-only");
    return 0;
}
