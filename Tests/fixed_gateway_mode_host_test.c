#include <math.h>
#include <stdio.h>
#include "fixed_control.h"
#include "fixed_numeric_config.h"
#include "identification.h"

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

    puts("PASS: A2L mode, ADC Q24, and legacy identification gateway");
    return 0;
}
