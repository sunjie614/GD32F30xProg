#include <math.h>
#include <stdio.h>
#include "sensorless_iq_gateway.h"
#include "fixed_numeric_config.h"
#include "parameters.h"

static int near(mc_real_t value, float expected)
{
    return fabsf(McMath_ToFloat(value) - expected) < 0.000002F;
}

static int check_bandwidth_ceiling(void)
{
    SensorlessGatewayState_t gateway;
    SensorlessIqConfig_t config;
    LesoIqConfig_t lc; HfiIqConfig_t hc;
    SensorlessIqState_t state; LesoIqState_t leso; HfiIqState_t hfi;
    SensorlessGateway_Init(&gateway, &config);
    SensorlessIq_Init(&state, &config);
    LesoIq_DefaultConfig(&lc); LesoIq_Init(&leso, &lc);
    HfiIq_DefaultConfig(&hc); HfiIq_Init(&hfi, &hc);
    if (Leso_Wc_Max != (float)LESO_IQ_WC_MAX_RAD_S
        || lc.bandwidth_maximum_step != LesoIq_MaximumBandwidthStep())
        return 10;
    /* Invalid pairs roll back, including a changed valid minimum paired with
     * an invalid maximum. They must NOT silently change to 3990. */
    const struct { float minimum, maximum; bool valid; } vectors[] = {
        {700.0F, 3800.0F, true},
        {800.0F, 4000.0F, false},
        {900.0F, 4200.0F, false},
        {1000.0F, 5000.0F, false},
        {NAN, 3990.0F, false},
        {700.0F, NAN, false},
        {4000.0F, 3990.0F, false},
        {3991.0F, 4500.0F, false},
        {3801.0F, 3800.0F, false},
        {0.0F, 3800.0F, false},
        {700.0F, INFINITY, false},
        {600.0F, 3990.0F, true},
        {3990.0F, 3990.0F, true}};
    float accepted_minimum = LESO_WC_MIN;
    float accepted_maximum = LESO_IQ_WC_MAX_RAD_S;
    for (unsigned n = 0U; n < sizeof(vectors) / sizeof(vectors[0]); ++n)
    {
        mc_real_t old_minimum = lc.bandwidth_minimum_step;
        mc_real_t old_maximum = lc.bandwidth_maximum_step;
        uint32_t old_invalid = McMath_Diagnostics.invalid_input_count;
        Leso_Wc_Min = vectors[n].minimum;
        Leso_Wc_Max = vectors[n].maximum;
        SensorlessGateway_Read(&gateway, &config, &lc, &hc, &state, &leso, &hfi);
        if (vectors[n].valid)
        {
            accepted_minimum = vectors[n].minimum;
            accepted_maximum = vectors[n].maximum;
            if (McMath_Diagnostics.invalid_input_count != old_invalid) return 11;
        }
        else if (McMath_Diagnostics.invalid_input_count != old_invalid + 1U
            || lc.bandwidth_minimum_step != old_minimum
            || lc.bandwidth_maximum_step != old_maximum)
            return 12;
        if (Leso_Wc_Min != accepted_minimum || Leso_Wc_Max != accepted_maximum
            || lc.bandwidth_minimum_step > lc.bandwidth_maximum_step
            || lc.bandwidth_maximum_step > LesoIq_MaximumBandwidthStep())
            return 13;
    }
    Leso_Wc_Max = nextafterf((float)LESO_IQ_WC_MAX_RAD_S, INFINITY);
    SensorlessGateway_Read(&gateway, &config, &lc, &hc, &state, &leso, &hfi);
    if (Leso_Wc_Max != (float)LESO_IQ_WC_MAX_RAD_S
        || lc.bandwidth_minimum_step != LesoIq_MaximumBandwidthStep()
        || lc.bandwidth_maximum_step != LesoIq_MaximumBandwidthStep())
        return 14;
    puts("PASS: Wc default 3990, 14 pair/boundary writes, invalid pairs roll back");
    return 0;
}

int main(void)
{
    SensorlessGatewayState_t gateway;
    SensorlessIqConfig_t config;
    LesoIqConfig_t lc; HfiIqConfig_t hc;
    SensorlessIqState_t state; LesoIqState_t leso; HfiIqState_t hfi;
    FlyingIqState_t flying;
    SensorlessGateway_Init(&gateway, &config);
    SensorlessIq_Init(&state, &config);
    LesoIq_DefaultConfig(&lc); LesoIq_Init(&leso, &lc);
    HfiIq_DefaultConfig(&hc); HfiIq_Init(&hfi, &hc);
    FlyingIq_Init(&flying, 8U);
    Sensorless_Theta_PID.Kp = 80.0F;
    Sensorless_Theta_PID.Ki = 1000.0F;
    Sensorless_Theta_PID.MinOutput = -900.0F;
    Sensorless_Theta_PID.MaxOutput = 900.0F;
    Sensorless_ThetAdj = 0.5F;
    Hfi_InjVolt = 25.0F;
    Leso_Gain = 9.0F;
    Leso_Wc_Min = 700.0F; Leso_Wc_Max = 3800.0F;
    Leso_Rs = 0.75F;
    Sensorless_Method = HF_INJECTION;
    Hfi_Enabled = Leso_Enabled = true;
    SensorlessGateway_Read(&gateway, &config, &lc, &hc, &state, &leso, &hfi);
    if (!near(state.common_pid.kp, 0.08F)
        || !near(state.common_pid.ki_step, 1000.0F * MAIN_LOOP_TIME / 1000.0F)
        || !near(config.minimum, -0.9F) || !near(config.maximum, 0.9F)
        || !near(config.theta_adjustment_pu, 0.5F * MC_INV_TWO_PI_F)
        || !near(hc.injection_voltage_pu, 25.0F / MC_VOLTAGE_BASE_V)
        || !near(lc.bandwidth_gain, 9.0F)
        || !near(lc.bandwidth_minimum_step, 700.0F * MAIN_LOOP_TIME)
        || !near(lc.resistance_pu, 0.75F * MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V)
        || !leso.enabled || !hfi.enabled
        || state.method != SENSORLESS_IQ_HF_INJECTION)
        return 1;
    SensorlessGateway_Publish(&gateway, &state, &leso, &hfi, &flying, (ParkIq_t){0}, false);
    state.common_pid.kp = MC_CONST(0.1);
    SensorlessGateway_Publish(&gateway, &state, &leso, &hfi, &flying, (ParkIq_t){0}, false);
    SensorlessGateway_Read(&gateway, &config, &lc, &hc, &state, &leso, &hfi);
    if (!near(state.common_pid.kp, 0.1F)) return 2;
    Sensorless_Theta_PID.Kp = NAN;
    Sensorless_Theta_PID.MinOutput = 2000.0F;
    Sensorless_Theta_PID.MaxOutput = 1e9F;
    Leso_Wc_Max = 6000.0F;
    Sensorless_Method = (sensorless_method_t)4;
    SensorlessGateway_Read(&gateway, &config, &lc, &hc, &state, &leso, &hfi);
    if (!isfinite(Sensorless_Theta_PID.Kp)
        || Sensorless_Theta_PID.MinOutput != -900.0F
        || Sensorless_Theta_PID.MaxOutput != 900.0F
        || Leso_Wc_Max != 3800.0F
        || Sensorless_Method != HF_INJECTION)
        return 3;
    Sensorless_Theta_PID.Ts = 0.5F;
    SensorlessGateway_Read(&gateway, &config, &lc, &hc, &state, &leso, &hfi);
    if (Sensorless_Theta_PID.Ts != MAIN_LOOP_TIME) return 4;
    int bandwidth_result = check_bandwidth_ceiling();
    if (bandwidth_result != 0) return bandwidth_result;
    puts("PASS: sensorless physical gateway, write-through, rollback and adaptive-gain cache");
    return 0;
}
