#include "fixed_debug_gateway.h"

#include <stddef.h>
#include "fixed_numeric_config.h"

volatile float Sensorless_ThetaEst = 0.0F, Sensorless_SpeedEst = 0.0F;
volatile float Sensorless_ThetaErr = 0.0F, Sensorless_SpeedErr = 0.0F;
volatile float Leso_Theta = 0.0F, Leso_Speed = 0.0F;
volatile float Leso_Theta_Err = 0.0F, Leso_Speed_Err = 0.0F;
volatile float Hfi_Theta = 0.0F, Hfi_Speed = 0.0F;
volatile float Hfi_Theta_Err = 0.0F, Hfi_Speed_Err = 0.0F;
volatile Clark_t Leso_CurEst = {0}, Leso_Current = {0};
volatile Clark_t Leso_Voltage = {0}, Leso_EmfEst = {0};
volatile Park_t Leso_EmfEst_dq = {0}, Leso_Emf_Filtered = {0};
volatile Park_t Leso_Emf_Slow_Filtered = {0};
volatile Clark_t Hfi_IClarkFdbk = {0}, Hfi_IClarkResp = {0};
volatile Clark_t Hfi_IClarkFilt = {0};
volatile Park_t MTPA_Inductor = {0};
volatile A2lMtpaPoint_t mtpa_table[MTPA_IQ_TABLE_POINTS] = {0};
volatile float a_d = 0.0F, b_d = 0.0F, a_q = 0.0F, b_q = 0.0F, c_coeff = 0.0F;

static float debug_physical(mc_real_t value, float base)
{
    return McMath_ToFloat(value) * base;
}

static Clark_t debug_clark(ClarkIq_t value, float base)
{
    return (Clark_t){debug_physical(value.a, base),
                     debug_physical(value.b, base)};
}

static Park_t debug_park(ParkIq_t value, float base)
{
    return (Park_t){debug_physical(value.d, base),
                    debug_physical(value.q, base)};
}

static float debug_angle_error(mc_real_t measured, mc_real_t estimated)
{
    return debug_physical(AngleIq_ErrorPu(measured, estimated), 360.0F);
}

void FixedDebug_Publish(const FocIqState_t* state,
                        const FocIqInput_t* input,
                        const FocIqOutput_t* output)
{
    if (state == NULL || input == NULL || output == NULL)
        return;
    Sensorless_ThetaEst = debug_physical(state->sensorless.angle_pu, MC_TWO_PI_F);
    Sensorless_SpeedEst = debug_physical(output->estimated_speed_pu, MC_SPEED_BASE_RPM);
    Sensorless_ThetaErr = debug_angle_error(input->measured_angle_pu,
                                           state->sensorless.angle_pu);
    Sensorless_SpeedErr = debug_physical(McMath_Sub(input->measured_speed_pu,
        output->estimated_speed_pu), MC_SPEED_BASE_RPM);
    Leso_Theta = debug_physical(state->leso.pll.angle_pu, MC_TWO_PI_F);
    Leso_Speed = debug_physical(state->leso.pll.speed_pu, MC_SPEED_BASE_RPM);
    Leso_Theta_Err = debug_angle_error(input->measured_angle_pu,
                                      state->leso.pll.angle_pu);
    Leso_Speed_Err = debug_physical(McMath_Sub(input->measured_speed_pu,
        state->leso.pll.speed_pu), MC_SPEED_BASE_RPM);
    Hfi_Theta = debug_physical(state->hfi.pll.angle_pu, MC_TWO_PI_F);
    Hfi_Speed = debug_physical(state->hfi.pll.speed_pu, MC_SPEED_BASE_RPM);
    Hfi_Theta_Err = debug_angle_error(input->measured_angle_pu,
                                     state->hfi.pll.angle_pu);
    Hfi_Speed_Err = debug_physical(McMath_Sub(input->measured_speed_pu,
        state->hfi.pll.speed_pu), MC_SPEED_BASE_RPM);
    Leso_CurEst = debug_clark(state->leso.current_estimate, MC_CURRENT_BASE_A);
    Leso_Current = debug_clark(state->leso.last_current_ab_pu, MC_CURRENT_BASE_A);
    Leso_Voltage = debug_clark(state->leso.last_voltage_ab_pu, MC_VOLTAGE_BASE_V);
    Leso_EmfEst = debug_clark(state->leso.emf_estimate, MC_VOLTAGE_BASE_V);
    Leso_EmfEst_dq = debug_park(state->leso.emf_dq, MC_VOLTAGE_BASE_V);
    Leso_Emf_Filtered = debug_park(state->leso.emf_filtered, MC_VOLTAGE_BASE_V);
    Leso_Emf_Slow_Filtered = debug_park(state->leso.emf_slow_filtered, MC_VOLTAGE_BASE_V);
    Hfi_IClarkFdbk = debug_clark(state->current_ab_pu, MC_CURRENT_BASE_A);
    Hfi_IClarkResp = debug_clark(state->hfi.response_current, MC_CURRENT_BASE_A);
    Hfi_IClarkFilt = debug_clark(state->hfi.base_current, MC_CURRENT_BASE_A);
    MTPA_Inductor = debug_park(output->inductance_dq_pu,
                              MC_FLUX_BASE_WB / MC_CURRENT_BASE_A);
}

void FixedDebug_PublishMtpa(const MtpaIqState_t* state)
{
    if (state == NULL)
        return;
    /* Legacy fitted model names are accepted-model observations, not writes
     * into a background build which is already in progress. Flux base=1 Wb. */
    a_d = debug_physical(state->parameters.ad0, MC_CURRENT_BASE_A);
    b_d = debug_physical(state->parameters.add, MC_CURRENT_BASE_A);
    a_q = debug_physical(state->parameters.aq0, MC_CURRENT_BASE_A);
    b_q = debug_physical(state->parameters.aqq, MC_CURRENT_BASE_A);
    c_coeff = debug_physical(state->parameters.adq, MC_CURRENT_BASE_A);
    for (uint16_t index = 0U; index < MTPA_IQ_TABLE_POINTS; ++index)
    {
        const MtpaIqPoint_t* point = &state->table[index];
        mtpa_table[index] = (A2lMtpaPoint_t){
            .T_req = debug_physical(point->torque_pu, MC_TORQUE_BASE_NM),
            .Psi_s = debug_physical(point->flux_pu, MC_FLUX_BASE_WB),
            .gamma = debug_physical(point->gamma_pu, MC_TWO_PI_F),
            .Id = debug_physical(point->id_pu, MC_CURRENT_BASE_A),
            .Iq = debug_physical(point->iq_pu, MC_CURRENT_BASE_A),
            .Ld = debug_physical(point->ld_pu, MC_FLUX_BASE_WB / MC_CURRENT_BASE_A),
            .Lq = debug_physical(point->lq_pu, MC_FLUX_BASE_WB / MC_CURRENT_BASE_A),
            .valid = point->valid};
    }
    /* Preserve the old watch-list index meaning (increasing requested torque)
     * without changing the core's Iq-sorted lookup table. */
    for (uint16_t index = 1U; index < MTPA_IQ_TABLE_POINTS; ++index)
    {
        A2lMtpaPoint_t point = mtpa_table[index];
        uint16_t position = index;
        while (position > 0U && mtpa_table[position - 1U].T_req > point.T_req)
        {
            mtpa_table[position] = mtpa_table[position - 1U];
            position--;
        }
        mtpa_table[position] = point;
    }
}
