/* LEGACY IQMATH PROTOTYPE - intentionally retained for comparison only.
 * The IQMATH target now compiles fixed_control_gateway.c + foc_iq.c instead. */
#include "fixed_control.h"

#include <stddef.h>
#include "identification.h"
#include "mc_math.h"
#include "parameters.h"

#define CURRENT_BASE_A     (30.0F)
#define VOLTAGE_BASE_V     (800.0F)
#define SPEED_BASE_RPM     (1800.0F)
#define TWO_PI_F           (6.28318530717958647692F)
#define INV_TWO_PI_F       (0.15915494309189533577F)
#define SPEED_DIVIDER      (10U)

enum
{
    FIXED_MODE_IDLE = 0,
    FIXED_MODE_VF = 1,
    FIXED_MODE_IF = 2,
    FIXED_MODE_SPEED = 3,
    FIXED_MODE_STARTUP = 4,
    FIXED_MODE_IDENTIFY = 5
};

enum
{
    SENSORLESS_LESO = 1U << 0,
    SENSORLESS_HFI = 1U << 1,
    SENSORLESS_FLYING = 1U << 2
};

typedef struct
{
    mc_real_t kp;
    mc_real_t ki_step;
    mc_real_t integral;
    mc_real_t minimum;
    mc_real_t maximum;
} FixedPi_t;

typedef struct
{
    mc_real_t a;
    mc_real_t b;
    mc_real_t c;
} FixedPhase_t;

typedef struct
{
    mc_real_t a;
    mc_real_t b;
} FixedClark_t;

typedef struct
{
    mc_real_t d;
    mc_real_t q;
} FixedPark_t;

typedef struct
{
    mc_real_t theta_pu;
    mc_real_t omega_step_pu;
    mc_real_t speed_pu;
    mc_real_t integral;
    FixedClark_t current_previous;
    FixedClark_t voltage_previous;
} FixedObserver_t;

typedef struct
{
    int64_t sum_voltage_current;
    int64_t sum_current_squared;
    uint32_t sample_count;
    mc_real_t resistance_pu;
} FixedIdentification_t;

typedef struct
{
    bool initialized;
    uint16_t speed_counter;
    uint16_t previous_mode;
    mc_real_t speed_reference_pu;
    mc_real_t speed_ramp_pu;
    mc_real_t speed_feedback_pu;
    mc_real_t theta_pu;
    mc_real_t open_loop_theta_pu;
    mc_real_t bus_pu;
    FixedClark_t current_ab;
    FixedPark_t current_dq;
    FixedPark_t current_reference;
    FixedPark_t voltage_dq;
    FixedClark_t voltage_ab;
    FixedPi_t speed_pi;
    FixedPi_t current_d_pi;
    FixedPi_t current_q_pi;
    FixedObserver_t observer;
    FixedIdentification_t identification;
    mc_real_t hfi_phase_pu;
} FixedControlContext_t;

/* These structs intentionally remain float: they are the compatibility
   registers presented to A2L/CCP, never algorithm state. */
typedef struct
{
    volatile float Kp;
    volatile float Ki;
    volatile float Kd;
    volatile float integral;
    volatile float previous_error;
    volatile float MaxOutput;
    volatile float MinOutput;
    volatile float output;
    volatile float IntegralLimit;
    volatile float Ts;
    volatile bool Reset;
} A2lPidMirror_t;

typedef struct
{
    volatile Park_t vol_ref;
    volatile float freq;
    volatile float offset;
} A2lVfMirror_t;

typedef struct
{
    volatile Park_t cur_ref;
    volatile float freq;
    volatile float offset;
    volatile bool use_sensor;
} A2lIfMirror_t;

static FixedControlContext_t FixedControl_Context;

volatile uint16_t Foc_Mode = FIXED_MODE_IDLE;
volatile float Foc_Speed_Ref = 0.0F;
volatile float Foc_Speed_Ramp = 0.0F;
volatile float Foc_Speed_Fdbk = 0.0F;
volatile float Foc_Theta = 0.0F;
volatile float Foc_BusVoltage = 0.0F;
volatile float Foc_Id_Ref = 0.0F;
volatile float Foc_Iq_Ref = 0.0F;
volatile float Foc_Id_Fdbk = 0.0F;
volatile float Foc_Iq_Fdbk = 0.0F;
volatile float Foc_Ud_Ref = 0.0F;
volatile float Foc_Uq_Ref = 0.0F;
volatile bool MainInt_UseRealTheta = true;
volatile uint16_t Sensorless_Method = 0U;
volatile uint32_t FixedControl_CycleCount = 0U;
volatile FixedControlSelfTest_t FixedControl_SelfTest = {0};
volatile bool Foc_Reset = true;
volatile float Foc_Current_Ts = MAIN_LOOP_TIME;
volatile float Foc_Current_Freq = MAIN_LOOP_FREQ;
volatile float Foc_Speed_Ts = SPEED_LOOP_TIME;
volatile float Foc_Speed_Freq = SPEED_LOOP_FREQ;
volatile float Foc_BusVoltage_Inv = 0.0F;
volatile Park_t Foc_Idq_Ref = {0};
volatile Park_t Foc_Idq_Fdbk = {0};
volatile Park_t Foc_Udq_Ref = {0};
volatile Clark_t Foc_Iclark_Fdbk = {0};
volatile Clark_t Foc_Uclark_Ref = {0};
volatile Phase_t Foc_Tcm = {0.5F, 0.5F, 0.5F};
volatile A2lPidMirror_t Foc_Pid_Speed_Handler = {
    .Kp = PID_SPEED_LOOP_KP, .Ki = PID_SPEED_LOOP_KI,
    .Kd = PID_SPEED_LOOP_KD, .MaxOutput = PID_SPEED_LOOP_MAX_OUTPUT,
    .MinOutput = PID_SPEED_LOOP_MIN_OUTPUT,
    .IntegralLimit = PID_SPEED_LOOP_INTEGRAL_LIMIT,
    .Ts = SPEED_LOOP_TIME, .Reset = true};
volatile A2lPidMirror_t Foc_Pid_CurD_Handler = {
    .Kp = PID_CURRENT_D_LOOP_KP, .Ki = PID_CURRENT_D_LOOP_KI,
    .Kd = PID_CURRENT_D_LOOP_KD, .MaxOutput = PID_CURRENT_D_LOOP_MAX_OUTPUT,
    .MinOutput = PID_CURRENT_D_LOOP_MIN_OUTPUT,
    .IntegralLimit = PID_CURRENT_D_LOOP_INTEGRAL_LIMIT,
    .Ts = MAIN_LOOP_TIME, .Reset = true};
volatile A2lPidMirror_t Foc_Pid_CurQ_Handler = {
    .Kp = PID_CURRENT_Q_LOOP_KP, .Ki = PID_CURRENT_Q_LOOP_KI,
    .Kd = PID_CURRENT_Q_LOOP_KD, .MaxOutput = PID_CURRENT_Q_LOOP_MAX_OUTPUT,
    .MinOutput = PID_CURRENT_Q_LOOP_MIN_OUTPUT,
    .IntegralLimit = PID_CURRENT_Q_LOOP_INTEGRAL_LIMIT,
    .Ts = MAIN_LOOP_TIME, .Reset = true};
volatile A2lVfMirror_t Foc_VfParam = {0};
volatile A2lIfMirror_t Foc_IfParam = {0};
FluxExperiment_t Experiment = {0};

/* Extra physical-unit A2L inputs for open-loop and identification modes. */
volatile float Foc_Vf_Vd = 0.0F;
volatile float Foc_Vf_Vq = 0.0F;
volatile float Foc_Vf_Frequency = 0.0F;
volatile float Foc_Vf_Offset = 0.0F;
volatile float Foc_If_Id = 0.0F;
volatile float Foc_If_Iq = 0.0F;
volatile float Foc_If_Frequency = 0.0F;
volatile float Foc_If_Offset = 0.0F;
volatile bool Foc_If_UseSensor = false;
volatile float Identification_Voltage = 1.0F;
volatile float Identification_Rs = 0.0F;

static mc_real_t fixed_wrap_pu(mc_real_t value)
{
    while (value >= MC_ONE)
    {
        value = McMath_Sub(value, MC_ONE);
    }
    while (value < MC_ZERO)
    {
        value = McMath_Add(value, MC_ONE);
    }
    return value;
}

static mc_real_t fixed_angle_error(mc_real_t reference, mc_real_t feedback)
{
    mc_real_t error = McMath_Sub(reference, feedback);
    if (error > MC_HALF)
    {
        error = McMath_Sub(error, MC_ONE);
    }
    else if (error < -MC_HALF)
    {
        error = McMath_Add(error, MC_ONE);
    }
    return error;
}

static void fixed_pi_init(FixedPi_t* pi,
                          float kp,
                          float ki_step,
                          float minimum,
                          float maximum)
{
    pi->kp = McMath_FromFloat(kp);
    pi->ki_step = McMath_FromFloat(ki_step);
    pi->integral = MC_ZERO;
    pi->minimum = McMath_FromFloat(minimum);
    pi->maximum = McMath_FromFloat(maximum);
}

static void fixed_pi_reset(FixedPi_t* pi)
{
    pi->integral = MC_ZERO;
}

static void fixed_sync_a2l_parameters(void)
{
    /* A2L writes are sampled once at the deterministic ISR boundary. */
    FixedControl_Context.speed_pi.kp = McMath_FromFloat(
        Foc_Pid_Speed_Handler.Kp * SPEED_BASE_RPM / CURRENT_BASE_A);
    FixedControl_Context.speed_pi.ki_step = McMath_FromFloat(
        Foc_Pid_Speed_Handler.Ki * SPEED_LOOP_TIME
        * SPEED_BASE_RPM / CURRENT_BASE_A);
    FixedControl_Context.speed_pi.minimum = McMath_FromFloat(
        Foc_Pid_Speed_Handler.MinOutput / CURRENT_BASE_A);
    FixedControl_Context.speed_pi.maximum = McMath_FromFloat(
        Foc_Pid_Speed_Handler.MaxOutput / CURRENT_BASE_A);

    FixedControl_Context.current_d_pi.kp = McMath_FromFloat(
        Foc_Pid_CurD_Handler.Kp * CURRENT_BASE_A / VOLTAGE_BASE_V);
    FixedControl_Context.current_d_pi.ki_step = McMath_FromFloat(
        Foc_Pid_CurD_Handler.Ki * MAIN_LOOP_TIME
        * CURRENT_BASE_A / VOLTAGE_BASE_V);
    FixedControl_Context.current_d_pi.minimum = McMath_FromFloat(
        Foc_Pid_CurD_Handler.MinOutput / VOLTAGE_BASE_V);
    FixedControl_Context.current_d_pi.maximum = McMath_FromFloat(
        Foc_Pid_CurD_Handler.MaxOutput / VOLTAGE_BASE_V);

    FixedControl_Context.current_q_pi.kp = McMath_FromFloat(
        Foc_Pid_CurQ_Handler.Kp * CURRENT_BASE_A / VOLTAGE_BASE_V);
    FixedControl_Context.current_q_pi.ki_step = McMath_FromFloat(
        Foc_Pid_CurQ_Handler.Ki * MAIN_LOOP_TIME
        * CURRENT_BASE_A / VOLTAGE_BASE_V);
    FixedControl_Context.current_q_pi.minimum = McMath_FromFloat(
        Foc_Pid_CurQ_Handler.MinOutput / VOLTAGE_BASE_V);
    FixedControl_Context.current_q_pi.maximum = McMath_FromFloat(
        Foc_Pid_CurQ_Handler.MaxOutput / VOLTAGE_BASE_V);
}

static mc_real_t fixed_pi_step(FixedPi_t* pi, mc_real_t error)
{
    mc_real_t proportional = McMath_Mul(pi->kp, error);
    mc_real_t unclamped = McMath_Add(proportional, pi->integral);
    if (unclamped <= pi->maximum && unclamped >= pi->minimum)
    {
        pi->integral = McMath_Clamp(
            McMath_Add(pi->integral, McMath_Mul(pi->ki_step, error)),
            pi->minimum,
            pi->maximum);
    }
    return McMath_Clamp(McMath_Add(proportional, pi->integral),
                        pi->minimum,
                        pi->maximum);
}

static FixedClark_t fixed_clarke(FixedPhase_t input)
{
    FixedClark_t output;
    output.a = McMath_Sub(McMath_Mul(MC_CONST(0.6666666666667), input.a),
                          McMath_Mul(MC_CONST(0.3333333333333),
                                     McMath_Add(input.b, input.c)));
    output.b = McMath_Mul(MC_CONST(0.5773502691896),
                          McMath_Sub(input.b, input.c));
    return output;
}

static FixedPark_t fixed_park(FixedClark_t input, mc_real_t theta_pu)
{
    mc_real_t cosine = McMath_CosPu(theta_pu);
    mc_real_t sine = McMath_SinPu(theta_pu);
    FixedPark_t output;
    output.d = McMath_Add(McMath_Mul(input.a, cosine),
                          McMath_Mul(input.b, sine));
    output.q = McMath_Sub(McMath_Mul(input.b, cosine),
                          McMath_Mul(input.a, sine));
    return output;
}

static FixedClark_t fixed_inverse_park(FixedPark_t input, mc_real_t theta_pu)
{
    mc_real_t cosine = McMath_CosPu(theta_pu);
    mc_real_t sine = McMath_SinPu(theta_pu);
    FixedClark_t output;
    output.a = McMath_Sub(McMath_Mul(input.d, cosine),
                          McMath_Mul(input.q, sine));
    output.b = McMath_Add(McMath_Mul(input.d, sine),
                          McMath_Mul(input.q, cosine));
    return output;
}

static FixedPhase_t fixed_svpwm(FixedClark_t voltage, mc_real_t bus_pu)
{
    FixedPhase_t duty = {MC_HALF, MC_HALF, MC_HALF};
    if (bus_pu <= MC_CONST(0.025))
    {
        return duty;
    }

    mc_real_t inverse_bus = McMath_Div(MC_ONE, bus_pu);
    mc_real_t x = McMath_Mul(McMath_Mul(MC_CONST(1.7320508075689), voltage.b),
                             inverse_bus);
    mc_real_t y = McMath_Mul(
        McMath_Add(McMath_Mul(MC_CONST(1.5), voltage.a),
                   McMath_Mul(MC_CONST(0.8660254037844), voltage.b)),
        inverse_bus);
    mc_real_t z = McMath_Mul(
        McMath_Add(McMath_Mul(MC_CONST(-1.5), voltage.a),
                   McMath_Mul(MC_CONST(0.8660254037844), voltage.b)),
        inverse_bus);
    mc_real_t v1 = voltage.b;
    mc_real_t v2 = McMath_Mul(
        McMath_Sub(McMath_Mul(MC_CONST(1.7320508075689), voltage.a), voltage.b),
        MC_HALF);
    mc_real_t v3 = McMath_Mul(
        McMath_Sub(McMath_Mul(MC_CONST(-1.7320508075689), voltage.a), voltage.b),
        MC_HALF);
    uint8_t sector = (v1 > MC_ZERO ? 1U : 0U)
                   + (v2 > MC_ZERO ? 2U : 0U)
                   + (v3 > MC_ZERO ? 4U : 0U);
    mc_real_t t1 = MC_ZERO;
    mc_real_t t2 = MC_ZERO;
    switch (sector)
    {
    case 1: t1 = z; t2 = y; break;
    case 2: t1 = y; t2 = McMath_Neg(x); break;
    case 3: t1 = McMath_Neg(z); t2 = x; break;
    case 4: t1 = McMath_Neg(x); t2 = z; break;
    case 5: t1 = x; t2 = McMath_Neg(y); break;
    case 6: t1 = McMath_Neg(y); t2 = McMath_Neg(z); break;
    default: return duty;
    }
    mc_real_t sum = McMath_Add(t1, t2);
    if (sum > MC_ONE)
    {
        t1 = McMath_Div(t1, sum);
        t2 = McMath_Div(t2, sum);
    }
    mc_real_t ta = McMath_Mul(McMath_Sub(MC_ONE, McMath_Add(t1, t2)), MC_HALF);
    mc_real_t tb = McMath_Add(ta, t1);
    mc_real_t tc = McMath_Add(tb, t2);
    switch (sector)
    {
    case 1: duty.a = tb; duty.b = ta; duty.c = tc; break;
    case 2: duty.a = ta; duty.b = tc; duty.c = tb; break;
    case 3: duty.a = ta; duty.b = tb; duty.c = tc; break;
    case 4: duty.a = tc; duty.b = tb; duty.c = ta; break;
    case 5: duty.a = tc; duty.b = ta; duty.c = tb; break;
    case 6: duty.a = tb; duty.b = tc; duty.c = ta; break;
    default: break;
    }
    duty.a = McMath_Clamp(duty.a, MC_ZERO, MC_ONE);
    duty.b = McMath_Clamp(duty.b, MC_ZERO, MC_ONE);
    duty.c = McMath_Clamp(duty.c, MC_ZERO, MC_ONE);
    return duty;
}

static void fixed_observer_step(FixedObserver_t* observer,
                                FixedClark_t voltage,
                                FixedClark_t current)
{
    /* Normalized LESO/back-EMF approximation. All state and arithmetic are Q24. */
    FixedClark_t delta_i = {
        McMath_Sub(current.a, observer->current_previous.a),
        McMath_Sub(current.b, observer->current_previous.b)};
    FixedClark_t emf = {
        McMath_Sub(observer->voltage_previous.a,
                   McMath_Add(McMath_Mul(MC_CONST(0.024375), current.a),
                              McMath_Mul(MC_CONST(0.625), delta_i.a))),
        McMath_Sub(observer->voltage_previous.b,
                   McMath_Add(McMath_Mul(MC_CONST(0.024375), current.b),
                              McMath_Mul(MC_CONST(0.625), delta_i.b)))};
    mc_real_t measured = McMath_Atan2Pu(McMath_Neg(emf.a), emf.b);
    mc_real_t error = fixed_angle_error(measured, observer->theta_pu);
    observer->integral = McMath_Clamp(
        McMath_Add(observer->integral, McMath_Mul(MC_CONST(0.025), error)),
        MC_CONST(-0.02), MC_CONST(0.02));
    observer->omega_step_pu = McMath_Clamp(
        McMath_Add(McMath_Mul(MC_CONST(0.20), error), observer->integral),
        MC_CONST(-0.02), MC_CONST(0.02));
    observer->theta_pu = fixed_wrap_pu(
        McMath_Add(observer->theta_pu, observer->omega_step_pu));
    observer->speed_pu = McMath_Div(observer->omega_step_pu, MC_CONST(0.006));
    observer->current_previous = current;
    observer->voltage_previous = voltage;
}

static mc_real_t fixed_mtpa_id(mc_real_t iq_reference)
{
    /* Fixed runtime interpolation law; zero at low torque and bounded saliency
       current at high torque. This is replaced by identified coefficients via
       the A2L gateway after an identification run. */
    mc_real_t magnitude = McMath_Abs(iq_reference);
    mc_real_t result = McMath_Neg(
        McMath_Mul(MC_CONST(0.15), McMath_Mul(magnitude, magnitude)));
    return McMath_Clamp(result, MC_CONST(-0.20), MC_ZERO);
}

static void fixed_identification_step(FixedIdentification_t* identification,
                                      mc_real_t current_d,
                                      mc_real_t voltage_d)
{
    identification->sum_voltage_current +=
        ((int64_t)voltage_d * (int64_t)current_d) >> 24;
    identification->sum_current_squared +=
        ((int64_t)current_d * (int64_t)current_d) >> 24;
    identification->sample_count++;
    if (identification->sample_count >= 512U)
    {
        if (identification->sum_current_squared != 0)
        {
            int64_t raw = (identification->sum_voltage_current << 24)
                        / identification->sum_current_squared;
            if (raw > INT32_MAX) raw = INT32_MAX;
            if (raw < INT32_MIN) raw = INT32_MIN;
            identification->resistance_pu = (mc_real_t)raw;
        }
        identification->sum_voltage_current = 0;
        identification->sum_current_squared = 0;
        identification->sample_count = 0U;
    }
}

void FixedControl_Reset(void)
{
    FixedControl_Context.speed_counter = 0U;
    FixedControl_Context.speed_ramp_pu = MC_ZERO;
    FixedControl_Context.current_reference.d = MC_ZERO;
    FixedControl_Context.current_reference.q = MC_ZERO;
    FixedControl_Context.voltage_dq.d = MC_ZERO;
    FixedControl_Context.voltage_dq.q = MC_ZERO;
    fixed_pi_reset(&FixedControl_Context.speed_pi);
    fixed_pi_reset(&FixedControl_Context.current_d_pi);
    fixed_pi_reset(&FixedControl_Context.current_q_pi);
}

void FixedControl_Init(void)
{
    FixedControl_Context = (FixedControlContext_t){0};
    fixed_pi_init(&FixedControl_Context.speed_pi,
                  PID_SPEED_LOOP_KP * SPEED_BASE_RPM / CURRENT_BASE_A,
                  PID_SPEED_LOOP_KI * SPEED_LOOP_TIME * SPEED_BASE_RPM / CURRENT_BASE_A,
                  PID_SPEED_LOOP_MIN_OUTPUT / CURRENT_BASE_A,
                  PID_SPEED_LOOP_MAX_OUTPUT / CURRENT_BASE_A);
    fixed_pi_init(&FixedControl_Context.current_d_pi,
                  PID_CURRENT_D_LOOP_KP * CURRENT_BASE_A / VOLTAGE_BASE_V,
                  PID_CURRENT_D_LOOP_KI * MAIN_LOOP_TIME * CURRENT_BASE_A / VOLTAGE_BASE_V,
                  PID_CURRENT_D_LOOP_MIN_OUTPUT / VOLTAGE_BASE_V,
                  PID_CURRENT_D_LOOP_MAX_OUTPUT / VOLTAGE_BASE_V);
    fixed_pi_init(&FixedControl_Context.current_q_pi,
                  PID_CURRENT_Q_LOOP_KP * CURRENT_BASE_A / VOLTAGE_BASE_V,
                  PID_CURRENT_Q_LOOP_KI * MAIN_LOOP_TIME * CURRENT_BASE_A / VOLTAGE_BASE_V,
                  PID_CURRENT_Q_LOOP_MIN_OUTPUT / VOLTAGE_BASE_V,
                  PID_CURRENT_Q_LOOP_MAX_OUTPUT / VOLTAGE_BASE_V);
    McMath_ResetDiagnostics();
    FixedControl_Context.initialized = true;
    Experiment.Initialized = true;
    Experiment.sample_capacity = SAMPLE_CAPACITY;
    FixedControl_SelfTest = FixedControl_RunSelfTest();
}

FixedControlOutput_t FixedControl_Step(const FixedControlInput_t* input)
{
    FixedControlOutput_t output = {0};
    if (input == NULL)
    {
        McMath_Diagnostics.invalid_input_count++;
        output.pwm_duty = (Phase_t){0.5F, 0.5F, 0.5F};
        return output;
    }
    if (!FixedControl_Context.initialized)
    {
        FixedControl_Init();
    }
    FixedControl_CycleCount++;
    fixed_sync_a2l_parameters();
    Foc_Vf_Vd = Foc_VfParam.vol_ref.d;
    Foc_Vf_Vq = Foc_VfParam.vol_ref.q;
    Foc_Vf_Frequency = Foc_VfParam.freq;
    Foc_Vf_Offset = Foc_VfParam.offset;
    Foc_If_Id = Foc_IfParam.cur_ref.d;
    Foc_If_Iq = Foc_IfParam.cur_ref.q;
    Foc_If_Frequency = Foc_IfParam.freq;
    Foc_If_Offset = Foc_IfParam.offset;
    Foc_If_UseSensor = Foc_IfParam.use_sensor;

    FixedPhase_t current = {
        McMath_FromFloat(input->current_abc.a / CURRENT_BASE_A),
        McMath_FromFloat(input->current_abc.b / CURRENT_BASE_A),
        McMath_FromFloat(input->current_abc.c / CURRENT_BASE_A)};
    FixedControl_Context.current_ab = fixed_clarke(current);
    FixedControl_Context.speed_feedback_pu =
        McMath_FromFloat(input->speed_rpm / SPEED_BASE_RPM);
    FixedControl_Context.theta_pu = fixed_wrap_pu(
        McMath_FromFloat(input->theta_rad * INV_TWO_PI_F));
    if (Foc_Mode != FixedControl_Context.previous_mode)
    {
        FixedControl_Context.open_loop_theta_pu = FixedControl_Context.theta_pu;
        FixedControl_Context.previous_mode = Foc_Mode;
        fixed_pi_reset(&FixedControl_Context.speed_pi);
        fixed_pi_reset(&FixedControl_Context.current_d_pi);
        fixed_pi_reset(&FixedControl_Context.current_q_pi);
    }
    if (Foc_Mode == FIXED_MODE_VF)
    {
        FixedControl_Context.open_loop_theta_pu = fixed_wrap_pu(McMath_Add(
            FixedControl_Context.open_loop_theta_pu,
            McMath_FromFloat(Foc_Vf_Frequency * MAIN_LOOP_TIME)));
        FixedControl_Context.theta_pu = fixed_wrap_pu(McMath_Add(
            FixedControl_Context.open_loop_theta_pu,
            McMath_FromFloat(Foc_Vf_Offset * INV_TWO_PI_F)));
    }
    else if (Foc_Mode == FIXED_MODE_IF && !Foc_If_UseSensor)
    {
        FixedControl_Context.open_loop_theta_pu = fixed_wrap_pu(McMath_Add(
            FixedControl_Context.open_loop_theta_pu,
            McMath_FromFloat(Foc_If_Frequency * MAIN_LOOP_TIME)));
        FixedControl_Context.theta_pu = fixed_wrap_pu(McMath_Add(
            FixedControl_Context.open_loop_theta_pu,
            McMath_FromFloat(Foc_If_Offset * INV_TWO_PI_F)));
    }
    FixedControl_Context.bus_pu =
        McMath_FromFloat(input->bus_voltage / VOLTAGE_BASE_V);
    FixedControl_Context.speed_reference_pu = McMath_Clamp(
        McMath_FromFloat(Foc_Speed_Ref / SPEED_BASE_RPM), -MC_ONE, MC_ONE);

    fixed_observer_step(&FixedControl_Context.observer,
                        FixedControl_Context.voltage_ab,
                        FixedControl_Context.current_ab);
    if (!MainInt_UseRealTheta && Sensorless_Method != 0U)
    {
        FixedControl_Context.theta_pu = FixedControl_Context.observer.theta_pu;
        FixedControl_Context.speed_feedback_pu = FixedControl_Context.observer.speed_pu;
    }
    FixedControl_Context.current_dq = fixed_park(
        FixedControl_Context.current_ab, FixedControl_Context.theta_pu);

    Foc_Reset = input->reset || Foc_Mode == FIXED_MODE_IDLE;
    bool reset = Foc_Reset;
    if (reset)
    {
        FixedControl_Reset();
    }
    else if (Foc_Mode == FIXED_MODE_SPEED || Foc_Mode == FIXED_MODE_STARTUP)
    {
        FixedControl_Context.speed_counter++;
        if (FixedControl_Context.speed_counter >= SPEED_DIVIDER)
        {
            FixedControl_Context.speed_counter = 0U;
            mc_real_t ramp_step = MC_CONST(
                RAMP_SPEED_SLOPE * SPEED_LOOP_TIME / SPEED_BASE_RPM);
            mc_real_t delta = McMath_Sub(FixedControl_Context.speed_reference_pu,
                                         FixedControl_Context.speed_ramp_pu);
            if (delta > ramp_step)
                FixedControl_Context.speed_ramp_pu = McMath_Add(
                    FixedControl_Context.speed_ramp_pu, ramp_step);
            else if (delta < -ramp_step)
                FixedControl_Context.speed_ramp_pu = McMath_Sub(
                    FixedControl_Context.speed_ramp_pu, ramp_step);
            else
                FixedControl_Context.speed_ramp_pu =
                    FixedControl_Context.speed_reference_pu;
            FixedControl_Context.current_reference.q = fixed_pi_step(
                &FixedControl_Context.speed_pi,
                McMath_Sub(FixedControl_Context.speed_ramp_pu,
                           FixedControl_Context.speed_feedback_pu));
            FixedControl_Context.current_reference.d = fixed_mtpa_id(
                FixedControl_Context.current_reference.q);
        }
    }
    else if (Foc_Mode == FIXED_MODE_IF)
    {
        FixedControl_Context.current_reference.d =
            McMath_FromFloat(Foc_If_Id / CURRENT_BASE_A);
        FixedControl_Context.current_reference.q =
            McMath_FromFloat(Foc_If_Iq / CURRENT_BASE_A);
    }

    if (Foc_Mode == FIXED_MODE_VF)
    {
        FixedControl_Context.voltage_dq.d =
            McMath_FromFloat(Foc_Vf_Vd / VOLTAGE_BASE_V);
        FixedControl_Context.voltage_dq.q =
            McMath_FromFloat(Foc_Vf_Vq / VOLTAGE_BASE_V);
    }
    else
    {
        FixedControl_Context.voltage_dq.d = fixed_pi_step(
            &FixedControl_Context.current_d_pi,
            McMath_Sub(FixedControl_Context.current_reference.d,
                       FixedControl_Context.current_dq.d));
        FixedControl_Context.voltage_dq.q = fixed_pi_step(
            &FixedControl_Context.current_q_pi,
            McMath_Sub(FixedControl_Context.current_reference.q,
                       FixedControl_Context.current_dq.q));
    }

    if ((Sensorless_Method & SENSORLESS_HFI) != 0U)
    {
        FixedControl_Context.hfi_phase_pu = fixed_wrap_pu(McMath_Add(
            FixedControl_Context.hfi_phase_pu,
            MC_CONST(HF_INJECTION_FREQ * MAIN_LOOP_TIME)));
        FixedControl_Context.voltage_dq.d = McMath_Add(
            FixedControl_Context.voltage_dq.d,
            McMath_Mul(MC_CONST(HF_INJECTION_AMP / VOLTAGE_BASE_V),
                       McMath_SinPu(FixedControl_Context.hfi_phase_pu)));
    }

    if (Foc_Mode == FIXED_MODE_IDENTIFY)
    {
        mc_real_t injection = ((FixedControl_CycleCount / 250U) & 1U)
                            ? McMath_FromFloat(Identification_Voltage / VOLTAGE_BASE_V)
                            : McMath_FromFloat(-Identification_Voltage / VOLTAGE_BASE_V);
        FixedControl_Context.voltage_dq.d = injection;
        FixedControl_Context.voltage_dq.q = MC_ZERO;
        uint32_t sample_index = FixedControl_Context.identification.sample_count;
        fixed_identification_step(&FixedControl_Context.identification,
                                  FixedControl_Context.current_dq.d,
                                  injection);
        Identification_Rs = McMath_ToFloat(
            FixedControl_Context.identification.resistance_pu)
                          * VOLTAGE_BASE_V / CURRENT_BASE_A;
        Experiment.Complete = false;
        Experiment.state = INJECT_COLLECT;
        Experiment.Rs_est = Identification_Rs;
        if (sample_index < SAMPLE_CAPACITY)
        {
            Experiment.Id_buf[sample_index] = McMath_ToFloat(
                FixedControl_Context.current_dq.d) * CURRENT_BASE_A;
            Experiment.Iq_buf[sample_index] = McMath_ToFloat(
                FixedControl_Context.current_dq.q) * CURRENT_BASE_A;
            Experiment.Ud_buf[sample_index] =
                McMath_ToFloat(injection) * VOLTAGE_BASE_V;
            Experiment.Uq_buf[sample_index] = 0.0F;
            Experiment.pos = (int)sample_index;
        }
    }
    else
    {
        Experiment.Complete = true;
        Experiment.state = WAIT;
    }

    FixedControl_Context.voltage_ab = fixed_inverse_park(
        FixedControl_Context.voltage_dq, FixedControl_Context.theta_pu);
    FixedPhase_t pwm = reset
        ? (FixedPhase_t){MC_HALF, MC_HALF, MC_HALF}
        : fixed_svpwm(FixedControl_Context.voltage_ab,
                      FixedControl_Context.bus_pu);

    Foc_Speed_Ramp = McMath_ToFloat(FixedControl_Context.speed_ramp_pu)
                   * SPEED_BASE_RPM;
    Foc_Speed_Fdbk = McMath_ToFloat(FixedControl_Context.speed_feedback_pu)
                   * SPEED_BASE_RPM;
    Foc_Theta = McMath_ToFloat(FixedControl_Context.theta_pu) * TWO_PI_F;
    Foc_BusVoltage = input->bus_voltage;
    Foc_BusVoltage_Inv = input->bus_voltage > 0.0F
                       ? 1.0F / input->bus_voltage : 0.0F;
    Foc_Id_Ref = McMath_ToFloat(FixedControl_Context.current_reference.d)
               * CURRENT_BASE_A;
    Foc_Iq_Ref = McMath_ToFloat(FixedControl_Context.current_reference.q)
               * CURRENT_BASE_A;
    Foc_Id_Fdbk = McMath_ToFloat(FixedControl_Context.current_dq.d)
                * CURRENT_BASE_A;
    Foc_Iq_Fdbk = McMath_ToFloat(FixedControl_Context.current_dq.q)
                * CURRENT_BASE_A;
    Foc_Ud_Ref = McMath_ToFloat(FixedControl_Context.voltage_dq.d)
               * VOLTAGE_BASE_V;
    Foc_Uq_Ref = McMath_ToFloat(FixedControl_Context.voltage_dq.q)
               * VOLTAGE_BASE_V;
    Foc_Idq_Ref = (Park_t){Foc_Id_Ref, Foc_Iq_Ref};
    Foc_Idq_Fdbk = (Park_t){Foc_Id_Fdbk, Foc_Iq_Fdbk};
    Foc_Udq_Ref = (Park_t){Foc_Ud_Ref, Foc_Uq_Ref};
    Foc_Iclark_Fdbk = (Clark_t){
        McMath_ToFloat(FixedControl_Context.current_ab.a) * CURRENT_BASE_A,
        McMath_ToFloat(FixedControl_Context.current_ab.b) * CURRENT_BASE_A};
    Foc_Uclark_Ref = (Clark_t){
        McMath_ToFloat(FixedControl_Context.voltage_ab.a) * VOLTAGE_BASE_V,
        McMath_ToFloat(FixedControl_Context.voltage_ab.b) * VOLTAGE_BASE_V};

    output.pwm_duty = (Phase_t){McMath_ToFloat(pwm.a),
                                McMath_ToFloat(pwm.b),
                                McMath_ToFloat(pwm.c)};
    Foc_Tcm = output.pwm_duty;
    output.voltage_dq = (Park_t){Foc_Ud_Ref, Foc_Uq_Ref};
    output.current_dq = (Park_t){Foc_Id_Fdbk, Foc_Iq_Fdbk};
    output.estimated_theta_rad =
        McMath_ToFloat(FixedControl_Context.observer.theta_pu) * TWO_PI_F;
    output.estimated_speed_rpm =
        McMath_ToFloat(FixedControl_Context.observer.speed_pu) * SPEED_BASE_RPM;
    Foc_Pid_Speed_Handler.integral =
        McMath_ToFloat(FixedControl_Context.speed_pi.integral) * CURRENT_BASE_A;
    Foc_Pid_Speed_Handler.output = Foc_Iq_Ref;
    Foc_Pid_CurD_Handler.integral =
        McMath_ToFloat(FixedControl_Context.current_d_pi.integral) * VOLTAGE_BASE_V;
    Foc_Pid_CurD_Handler.output = Foc_Ud_Ref;
    Foc_Pid_CurQ_Handler.integral =
        McMath_ToFloat(FixedControl_Context.current_q_pi.integral) * VOLTAGE_BASE_V;
    Foc_Pid_CurQ_Handler.output = Foc_Uq_Ref;
    return output;
}

FixedControlSelfTest_t FixedControl_RunSelfTest(void)
{
    FixedControlSelfTest_t result = {0};
    const mc_real_t tolerance = MC_CONST(0.00002);
    mc_real_t half = McMath_FromFloat(0.5F);
    if (McMath_Abs(McMath_Sub(half, MC_CONST(0.5))) > tolerance)
        result.conversion_failures++;
    if (McMath_Abs(McMath_Sub(McMath_Mul(half, half), MC_CONST(0.25))) > tolerance)
        result.arithmetic_failures++;
    if (McMath_Abs(McMath_Sub(McMath_Div(half, MC_CONST(0.25)), MC_CONST(2.0))) > tolerance)
        result.arithmetic_failures++;
    if (McMath_Abs(McMath_Sub(McMath_SinPu(MC_CONST(0.25)), MC_ONE)) > tolerance)
        result.trigonometric_failures++;
    if (McMath_Abs(McMath_Sub(McMath_CosPu(MC_CONST(0.5)), -MC_ONE)) > tolerance)
        result.trigonometric_failures++;
    if (McMath_Abs(McMath_Sub(McMath_Sqrt(MC_CONST(0.25)), half)) > tolerance)
        result.sqrt_failures++;
    return result;
}
