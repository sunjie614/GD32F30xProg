#include "sensorless_iq_gateway.h"

#include <math.h>
#include <stddef.h>
#include "fixed_numeric_config.h"
#include "parameters.h"

volatile bool Sensorless_Enabled = false;
volatile bool Sensorless_Reset = true;
volatile sensorless_method_t Sensorless_Method = FLYING;
volatile bool Leso_Enabled = false, Hfi_Enabled = false;
volatile float Sensorless_ThetAdj = 0.0F;
volatile float Sensorless_Switch_Speed = SENSORLESS_SWITCH_SPEED;
volatile float Sensorless_Threshold_Hfi = SENSORLESS_SWITCH_SPEED + SENSORLESS_HYSTERESIS;
volatile float Sensorless_Threshold_Leso = SENSORLESS_SWITCH_SPEED - SENSORLESS_HYSTERESIS;
volatile float PLLKPMAX = 130.0F, PLLKIMAX = 4225.0F;
volatile float Sensorless_PLL_Wc = SENSORLESS_PLL_WC;
volatile A2lPidMirror_t Sensorless_Theta_PID = {
    .Kp = SENSORLESS_PLL_KP, .Ki = SENSORLESS_PLL_KI, .Kd = SENSORLESS_PLL_KD,
    .MaxOutput = SENSORLESS_PLL_MAX_OUTPUT, .MinOutput = SENSORLESS_PLL_MIN_OUTPUT,
    .IntegralLimit = SENSORLESS_PLL_INTEGRAL_LIMIT, .Ts = MAIN_LOOP_TIME};
volatile A2lPidMirror_t Leso_Theta_PID = {
    .Kp = SMO_PLL_KP, .Ki = SMO_PLL_KI, .Kd = SMO_PLL_KD,
    .MaxOutput = SMO_PLL_MAX_OUTPUT, .MinOutput = SMO_PLL_MIN_OUTPUT,
    .IntegralLimit = SMO_PLL_INTEGRAL_LIMIT, .Ts = MAIN_LOOP_TIME};
volatile A2lPidMirror_t Hfi_Theta_Pid = {
    .Kp = HFI_PLL_KP, .Ki = HFI_PLL_KI, .Kd = HFI_PLL_KD,
    .MaxOutput = HFI_PLL_MAX_OUTPUT, .MinOutput = HFI_PLL_MIN_OUTPUT,
    .IntegralLimit = HFI_PLL_INTEGRAL_LIMIT, .Ts = MAIN_LOOP_TIME};
volatile float Leso_Gain = LESO_WC_GAIN, Leso_Factor = 1.9F;
volatile float Leso_Wc_Min = LESO_WC_MIN, Leso_Wc_Max = LESO_IQ_WC_MAX_RAD_S;
volatile float Leso_Rs = MOTOR_RS;
volatile float Leso_Wc = 0.0F, Leso_We = 0.0F;
volatile float Leso_Beta1 = 0.0F, Leso_Beta2 = 0.0F;
volatile float Leso_Ld = MOTOR_LD, Leso_Lq = MOTOR_LQ;
volatile float Leso_InvLd = 1.0F / MOTOR_LD, Leso_InvLq = 1.0F / MOTOR_LQ;
/* Legacy 1e25 is not a representable fixed limit. This reports the effective
 * Q24 limit in A for current_estimate, not a silently editable fake limit. */
volatile float Leso_Int_limit = 128.0F * MC_CURRENT_BASE_A;
volatile float Leso_Error = 0.0F, Hfi_Error = 0.0F;
volatile float Hfi_InjVolt = HF_INJECTION_AMP;
volatile float Hfi_InjFreq = HF_INJECTION_FREQ;
volatile bool Hfi_InjectSign = false;
volatile bool Flying_Enabled = false, Flying_Completed = false;
volatile uint16_t Flying_Count = 0U;
volatile float Sensorless_SpeedRef = 0.0F, Sensorless_SpeedTarget = 0.0F;
volatile float Sensorless_SpeedFdbk = 0.0F;

enum { GW_KP, GW_KI, GW_KD, GW_MIN, GW_MAX, GW_LIMIT, GW_ADJ,
       GW_SWITCH, GW_HFI, GW_LESO, GW_KPMAX, GW_KIMAX, GW_GAIN,
       GW_WCMIN, GW_WCMAX, GW_RS };

/* Reject an invalid physical write and show the accepted value again. */
static bool gateway_changed(SensorlessGatewayState_t* cache, unsigned index,
                             volatile float* mirror, float scale,
                             bool nonnegative, mc_real_t* destination)
{
    float value = *mirror;
    if (value == cache->accepted[index])
        return false;
    float normalized = value * scale;
    if (!isfinite(value) || !isfinite(normalized)
        || normalized <= -128.0F || normalized >= 128.0F
        || (nonnegative && value < 0.0F))
    {
        McMath_Diagnostics.invalid_input_count++;
        *mirror = cache->accepted[index];
        return false;
    }
    *destination = McMath_FromFloat(normalized);
    cache->accepted[index] = value;
    return true;
}

void SensorlessGateway_Init(SensorlessGatewayState_t* cache,
                            SensorlessIqConfig_t* config)
{
    if (cache == NULL || config == NULL)
        return;
    SensorlessIq_DefaultConfig(config);
    Sensorless_Enabled = false;
    Leso_Enabled = Hfi_Enabled = false;
    Sensorless_Method = FLYING;
    Sensorless_ThetAdj = 0.0F;
    Sensorless_Switch_Speed = SENSORLESS_SWITCH_SPEED;
    Sensorless_Threshold_Hfi = SENSORLESS_SWITCH_SPEED + SENSORLESS_HYSTERESIS;
    Sensorless_Threshold_Leso = SENSORLESS_SWITCH_SPEED - SENSORLESS_HYSTERESIS;
    Sensorless_Theta_PID = (A2lPidMirror_t){
        .Kp = SENSORLESS_PLL_KP, .Ki = SENSORLESS_PLL_KI, .Kd = SENSORLESS_PLL_KD,
        .MaxOutput = SENSORLESS_PLL_MAX_OUTPUT, .MinOutput = SENSORLESS_PLL_MIN_OUTPUT,
        .IntegralLimit = SENSORLESS_PLL_INTEGRAL_LIMIT, .Ts = MAIN_LOOP_TIME};
    PLLKPMAX = 130.0F; PLLKIMAX = 4225.0F;
    Leso_Gain = LESO_WC_GAIN; Leso_Factor = 1.9F;
    Leso_Wc_Min = LESO_WC_MIN; Leso_Wc_Max = LESO_IQ_WC_MAX_RAD_S;
    Leso_Rs = MOTOR_RS; Hfi_InjVolt = HF_INJECTION_AMP;
    *cache = (SensorlessGatewayState_t){.accepted = {
        SENSORLESS_PLL_KP, SENSORLESS_PLL_KI, SENSORLESS_PLL_KD,
        SENSORLESS_PLL_MIN_OUTPUT, SENSORLESS_PLL_MAX_OUTPUT,
        SENSORLESS_PLL_INTEGRAL_LIMIT, 0.0F, SENSORLESS_SWITCH_SPEED,
        SENSORLESS_SWITCH_SPEED + SENSORLESS_HYSTERESIS,
        SENSORLESS_SWITCH_SPEED - SENSORLESS_HYSTERESIS,
        130.0F, 4225.0F, LESO_WC_GAIN, LESO_WC_MIN, LESO_IQ_WC_MAX_RAD_S, MOTOR_RS},
        .method = FLYING};
}

void SensorlessGateway_Reinitialize(SensorlessGatewayState_t* cache,
                                    SensorlessIqConfig_t* config)
{
    if (cache == NULL || config == NULL)
        return;
    /* Read has already validated pending writes using the old accepted
     * cache. Preserve precisely the controls that float module INIT did not
     * reset; copying the public mirror blindly could retain NaN/overflow. */
    bool enabled = Sensorless_Enabled;
    bool leso_enabled = Leso_Enabled, hfi_enabled = Hfi_Enabled;
    sensorless_method_t method = Sensorless_Method;
    float adjustment = cache->accepted[GW_ADJ];
    float kp_maximum = cache->accepted[GW_KPMAX];
    float ki_maximum = cache->accepted[GW_KIMAX];
    SensorlessGateway_Init(cache, config);
    Sensorless_Enabled = enabled;
    Leso_Enabled = cache->leso_enabled = leso_enabled;
    Hfi_Enabled = cache->hfi_enabled = hfi_enabled;
    Sensorless_Method = cache->method = method;
    Sensorless_ThetAdj = cache->accepted[GW_ADJ] = adjustment;
    PLLKPMAX = cache->accepted[GW_KPMAX] = kp_maximum;
    PLLKIMAX = cache->accepted[GW_KIMAX] = ki_maximum;
    config->theta_adjustment_pu = McMath_FromFloat(adjustment * MC_INV_TWO_PI_F);
    config->kp_maximum = McMath_FromFloat(kp_maximum / SENSORLESS_IQ_OMEGA_BASE_RAD_S);
    config->ki_step_maximum = McMath_FromFloat(ki_maximum * MAIN_LOOP_TIME
        / SENSORLESS_IQ_OMEGA_BASE_RAD_S);
    Leso_Theta_PID = (A2lPidMirror_t){
        .Kp = SMO_PLL_KP, .Ki = SMO_PLL_KI, .Kd = SMO_PLL_KD,
        .MaxOutput = SMO_PLL_MAX_OUTPUT, .MinOutput = SMO_PLL_MIN_OUTPUT,
        .IntegralLimit = SMO_PLL_INTEGRAL_LIMIT, .Ts = MAIN_LOOP_TIME};
    Hfi_Theta_Pid = (A2lPidMirror_t){
        .Kp = HFI_PLL_KP, .Ki = HFI_PLL_KI, .Kd = HFI_PLL_KD,
        .MaxOutput = HFI_PLL_MAX_OUTPUT, .MinOutput = HFI_PLL_MIN_OUTPUT,
        .IntegralLimit = HFI_PLL_INTEGRAL_LIMIT, .Ts = MAIN_LOOP_TIME};
    Hfi_InjFreq = HF_INJECTION_FREQ;
}

void SensorlessGateway_Read(SensorlessGatewayState_t* cache,
                            SensorlessIqConfig_t* config,
                            LesoIqConfig_t* lc, HfiIqConfig_t* hc,
                            SensorlessIqState_t* state,
                            LesoIqState_t* leso, HfiIqState_t* hfi)
{
    if (cache == NULL || config == NULL || lc == NULL || hc == NULL
        || state == NULL || leso == NULL || hfi == NULL)
        return;
    const float omega_inv = 1.0F / SENSORLESS_IQ_OMEGA_BASE_RAD_S;
    if (gateway_changed(cache, GW_KP, &Sensorless_Theta_PID.Kp, omega_inv,
                         true, &config->kp))
        state->common_pid.kp = config->kp;
    if (gateway_changed(cache, GW_KI, &Sensorless_Theta_PID.Ki,
                         MAIN_LOOP_TIME * omega_inv, true, &config->ki_step))
        state->common_pid.ki_step = config->ki_step;
    (void)gateway_changed(cache, GW_KD, &Sensorless_Theta_PID.Kd,
                          omega_inv / MAIN_LOOP_TIME, true, &config->kd_step);
    /* A pair is validated atomically: never send minimum > maximum to PID. */
    float minimum = Sensorless_Theta_PID.MinOutput;
    float maximum = Sensorless_Theta_PID.MaxOutput;
    if (!isfinite(minimum) || !isfinite(maximum) || minimum > maximum
        || minimum * omega_inv <= -128.0F || maximum * omega_inv >= 128.0F
        || minimum * omega_inv >= 128.0F || maximum * omega_inv <= -128.0F)
    {
        Sensorless_Theta_PID.MinOutput = cache->accepted[GW_MIN];
        Sensorless_Theta_PID.MaxOutput = cache->accepted[GW_MAX];
        McMath_Diagnostics.invalid_input_count++;
    }
    (void)gateway_changed(cache, GW_MIN, &Sensorless_Theta_PID.MinOutput,
                          omega_inv, false, &config->minimum);
    (void)gateway_changed(cache, GW_MAX, &Sensorless_Theta_PID.MaxOutput,
                          omega_inv, false, &config->maximum);
    (void)gateway_changed(cache, GW_LIMIT, &Sensorless_Theta_PID.IntegralLimit,
                          omega_inv, true, &config->integral_limit);
    (void)gateway_changed(cache, GW_ADJ, &Sensorless_ThetAdj,
                          MC_INV_TWO_PI_F, false, &config->theta_adjustment_pu);
    (void)gateway_changed(cache, GW_SWITCH, &Sensorless_Switch_Speed,
                          1.0F / MC_SPEED_BASE_RPM, true, &config->switch_speed_pu);
    (void)gateway_changed(cache, GW_HFI, &Sensorless_Threshold_Hfi,
                          1.0F / MC_SPEED_BASE_RPM, true, &config->threshold_hfi_pu);
    (void)gateway_changed(cache, GW_LESO, &Sensorless_Threshold_Leso,
                          1.0F / MC_SPEED_BASE_RPM, true, &config->threshold_leso_pu);
    (void)gateway_changed(cache, GW_KPMAX, &PLLKPMAX, omega_inv,
                          true, &config->kp_maximum);
    (void)gateway_changed(cache, GW_KIMAX, &PLLKIMAX,
                          MAIN_LOOP_TIME * omega_inv, true, &config->ki_step_maximum);
    (void)gateway_changed(cache, GW_GAIN, &Leso_Gain, 1.0F,
                          true, &lc->bandwidth_gain);
    /* Keep the existing pair-rollback contract. An excessive/invalid write
     * restores BOTH last accepted values; it is not silently clamped. */
    float wc_minimum = Leso_Wc_Min;
    float wc_maximum = Leso_Wc_Max;
    if (!isfinite(wc_minimum) || !isfinite(wc_maximum)
        || wc_minimum <= 0.0F || wc_minimum > wc_maximum
        || wc_maximum > (float)LESO_IQ_WC_MAX_RAD_S)
    {
        Leso_Wc_Min = cache->accepted[GW_WCMIN];
        Leso_Wc_Max = cache->accepted[GW_WCMAX];
        McMath_Diagnostics.invalid_input_count++;
    }
    (void)gateway_changed(cache, GW_WCMIN, &Leso_Wc_Min,
                          MAIN_LOOP_TIME, true, &lc->bandwidth_minimum_step);
    (void)gateway_changed(cache, GW_WCMAX, &Leso_Wc_Max,
                          MAIN_LOOP_TIME, true, &lc->bandwidth_maximum_step);
    /* Physical 3990 is valid, but its Q24 conversion must never round above
     * the exact ceiling. This also covers min=max=3990. */
    mc_real_t bandwidth_limit = LesoIq_MaximumBandwidthStep();
    if (lc->bandwidth_minimum_step > bandwidth_limit)
        lc->bandwidth_minimum_step = bandwidth_limit;
    if (lc->bandwidth_maximum_step > bandwidth_limit)
        lc->bandwidth_maximum_step = bandwidth_limit;
    (void)gateway_changed(cache, GW_RS, &Leso_Rs,
                          MC_CURRENT_BASE_A / MC_VOLTAGE_BASE_V,
                          true, &lc->resistance_pu);
    /* This register is read every cycle because injection amplitude is an
     * intentional live experiment input; invalid writes are rolled back. */
    if (isfinite(Hfi_InjVolt) && fabsf(Hfi_InjVolt / MC_VOLTAGE_BASE_V) < 128.0F)
        hc->injection_voltage_pu = McMath_FromFloat(Hfi_InjVolt / MC_VOLTAGE_BASE_V);
    else
    {
        Hfi_InjVolt = McMath_ToFloat(hc->injection_voltage_pu) * MC_VOLTAGE_BASE_V;
        McMath_Diagnostics.invalid_input_count++;
    }
    if ((unsigned)Sensorless_Method > HF_INJECTION)
    {
        Sensorless_Method = cache->method;
        McMath_Diagnostics.invalid_input_count++;
    }
    if (Sensorless_Method != cache->method)
        state->method = (SensorlessIqMethod_t)Sensorless_Method;
    if (Leso_Enabled != cache->leso_enabled)
        leso->enabled = Leso_Enabled;
    if (Hfi_Enabled != cache->hfi_enabled)
        hfi->enabled = Hfi_Enabled;
    Sensorless_Theta_PID.Ts = MAIN_LOOP_TIME;
    Leso_Factor = 1.9F; /* Old Leso_Set_Inductor overwrote this each cycle. */
}

void SensorlessGateway_Publish(SensorlessGatewayState_t* cache,
                               const SensorlessIqState_t* state,
                               const LesoIqState_t* leso,
                               const HfiIqState_t* hfi,
                               const FlyingIqState_t* flying,
                               ParkIq_t inductance, bool reset)
{
    if (cache == NULL || state == NULL || leso == NULL || hfi == NULL || flying == NULL)
        return;
    Sensorless_Method = cache->method = (sensorless_method_t)state->method;
    Leso_Enabled = cache->leso_enabled = leso->enabled;
    Hfi_Enabled = cache->hfi_enabled = hfi->enabled;
    Sensorless_Theta_PID.Kp = cache->accepted[GW_KP]
        = McMath_ToFloat(state->common_pid.kp) * SENSORLESS_IQ_OMEGA_BASE_RAD_S;
    Sensorless_Theta_PID.Ki = cache->accepted[GW_KI]
        = McMath_ToFloat(state->common_pid.ki_step)
          * SENSORLESS_IQ_OMEGA_BASE_RAD_S / MAIN_LOOP_TIME;
    Sensorless_Theta_PID.integral = McMath_ToFloat(state->common_pid.integral)
        * SENSORLESS_IQ_OMEGA_BASE_RAD_S;
    Sensorless_Theta_PID.output = McMath_ToFloat(state->omega_pu)
        * SENSORLESS_IQ_OMEGA_BASE_RAD_S;
    Sensorless_Theta_PID.previous_error = McMath_ToFloat(state->common_pid.last_error);
    Sensorless_Theta_PID.Reset = reset;
    Sensorless_Reset = reset;
    Sensorless_SpeedRef = McMath_ToFloat(state->speed_reference_pu) * MC_SPEED_BASE_RPM;
    Sensorless_SpeedTarget = McMath_ToFloat(state->speed_target_pu) * MC_SPEED_BASE_RPM;
    Sensorless_SpeedFdbk = McMath_ToFloat(state->speed_feedback_pu) * MC_SPEED_BASE_RPM;
    Sensorless_PLL_Wc = McMath_ToFloat(state->pll_wc_pu) * SENSORLESS_IQ_OMEGA_BASE_RAD_S;
    Leso_Wc = McMath_ToFloat(leso->bandwidth_step) / MAIN_LOOP_TIME;
    Leso_Beta1 = 2.0F * Leso_Wc;
    Leso_Beta2 = Leso_Wc * Leso_Wc;
    Leso_We = McMath_ToFloat(leso->pll.speed_pu) * MC_SPEED_BASE_RPM
        * MC_TWO_PI_F * MOTOR_PN / 60.0F;
    Leso_Ld = McMath_ToFloat(inductance.d) * MC_FLUX_BASE_WB / MC_CURRENT_BASE_A;
    Leso_Lq = McMath_ToFloat(inductance.q) * MC_FLUX_BASE_WB / MC_CURRENT_BASE_A;
    if (Leso_Ld < 0.001F) Leso_Ld = 0.001F;
    if (Leso_Lq < 0.001F) Leso_Lq = 0.001F;
    Leso_InvLd = 1.0F / Leso_Ld;
    Leso_InvLq = 1.0F / Leso_Lq;
    Leso_Error = McMath_ToFloat(leso->phase_error_pu);
    Hfi_Error = McMath_ToFloat(hfi->phase_error_pu);
    Hfi_InjectSign = hfi->injection_positive;
    Flying_Enabled = flying->enabled; Flying_Completed = flying->completed;
    Flying_Count = flying->count;
}
