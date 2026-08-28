#include <stdio.h>
#include "foc_iq.h"

static unsigned Failures;

static void expect_true(const char* name, bool condition)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL %s\n", name);
        Failures++;
    }
}

static FocIqInput_t zero_input(void)
{
    return (FocIqInput_t){
        .current_abc_pu = {MC_ZERO, MC_ZERO, MC_ZERO},
        .measured_angle_pu = MC_CONST(0.125),
        .measured_speed_pu = MC_ZERO,
        .bus_voltage_pu = MC_CONST(0.5)};
}

int main(void)
{
    FocIqState_t state;
    FocIqConfig_t config;
    FocIqParameters_t parameters;
    FocIqOutput_t output;
    FocIqInput_t input = zero_input();
    FocIq_DefaultConfig(&config);
    FocIq_DefaultParameters(&parameters);
    FocIq_Init(&state, &config);
    expect_true("init", state.initialized && state.mtpa.table_valid);

    parameters.mode = SPEED;
    parameters.reset = true;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("speed held in reset", !output.startup_active);
    parameters.reset = false;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("startup on reset release", output.startup_active);
    expect_true("startup d voltage",
                output.voltage_reference_dq_pu.d
                    == config.startup_voltage_d_pu);

    parameters.mode = VF_MODE;
    parameters.vf_voltage_reference_pu.d = MC_CONST(0.05);
    parameters.vf_frequency_step_pu = MC_CONST(0.001);
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("vf duty a", output.pwm_duty.a >= MC_ZERO
                              && output.pwm_duty.a <= MC_ONE);
    expect_true("vf duty b", output.pwm_duty.b >= MC_ZERO
                              && output.pwm_duty.b <= MC_ONE);
    expect_true("vf duty c", output.pwm_duty.c >= MC_ZERO
                              && output.pwm_duty.c <= MC_ONE);

    parameters.mode = IDENTIFY;
    parameters.reset = true;
    FocIq_Run(&state, &parameters, &input, &output);
    parameters.reset = false;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("identification on reset release",
                output.identification_state == IDENTIFICATION_IQ_EST_RS);

    parameters.mode = (FocMode_t)99;
    FocIq_Run(&state, &parameters, &input, &output);
    expect_true("invalid mode fallback", output.requested_mode == IDLE);
    expect_true("invalid mode neutral pwm",
                output.pwm_duty.a == MC_HALF
                && output.pwm_duty.b == MC_HALF
                && output.pwm_duty.c == MC_HALF);

    MtpaIq_DefaultParameters(&state.pending_mtpa_parameters);
    state.mtpa_rebuild_pending = true;
    state.mode = IDLE;
    expect_true("background build", FocIq_BackgroundBuildMtpa(&state));
    expect_true("background forces idle", state.mtpa_rebuild_in_progress);
    FocIq_CommitBackgroundMtpa(&state);
    expect_true("background commit", state.mtpa.table_valid
                                     && !state.mtpa_rebuild_in_progress);

    if (Failures != 0U)
        return 1;
    puts("PASS: FOC IQ state regression");
    return 0;
}
