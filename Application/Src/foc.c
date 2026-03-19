#include "foc.h"
#include "buffer.h"
#include "leso.h"
#include "parameters.h"
#include "pid.h"
#include "signal.h"
#include "stdint.h"
#include "transformation.h"

#include "MTPA.h"
#include "identification.h"

static FocMode_t     Foc_Mode            = IDLE;   // 当前FOC模式
static bool          Foc_Reset           = false;  // FOC复位标志
static float         Foc_Current_Ts      = 0.0F;   // 电流环采样周期
static float         Foc_Current_Freq    = 0.0F;   // 电流环频率
static uint16_t      Foc_Speed_Prescaler = 0U;     // 电流环分频数
static float         Foc_Speed_Ts        = 0.0F;   // 转速环采样周期
static float         Foc_Speed_Freq      = 0.0F;   // 转速环频率
static float         Foc_Speed_Ref       = 0.0F;   // 参考速度
static float         Foc_Speed_Fdbk      = 0.0F;   // 实际转速反馈
static float         Foc_Theta           = 0.0F;
static float         Foc_BusVoltage      = 0.0F;
static float         Foc_BusVoltage_Inv  = 0.0F;
static float         Foc_Speed_Ramp      = 0.0F;  // 实际指令转速
static volatile bool Foc_Sweep           = true;  // FOC扫频标志
static volatile bool Foc_StartupPrepareRequest = false;

static VF_Parameter_t  Foc_VfParam            = {0};
static IF_Parameter_t  Foc_IfParam            = {0};
static Clark_t         Foc_Iclark_Fdbk        = {0};
static Park_t          Foc_Idq_Ref            = {0};
static Park_t          Foc_Idq_Fdbk           = {0};
static Clark_t         Foc_Uclark_Ref         = {0};
static Park_t          Foc_Udq_Ref            = {0};
static PID_Handler_t   Foc_Pid_Speed_Handler  = {0};
static PID_Handler_t   Foc_Pid_CurD_Handler   = {0};
static PID_Handler_t   Foc_Pid_CurQ_Handler   = {0};
static RampGenerator_t Foc_Ramp_Speed_Handler = {0};
static SawtoothWave_t  Foc_Sawtooth_Handler   = {0};

typedef enum
{
    SPEED_PI_KP_LOW = 0,
    SPEED_PI_KP_RAMP_UP,
    SPEED_PI_KP_HIGH,
    SPEED_PI_KP_RAMP_DOWN
} SpeedPiKpState_t;

typedef struct
{
    float            kp_mul;
    float            ki_mul;
    float            kp_mul_max;
    float            ki_mul_max;
    float            kp_up_start_err;
    float            kp_up_end_err;
    float            kp_down_start_err;
    float            kp_down_end_err;
    float            ki_switch_err;
    uint16_t         ki_confirm_cycles;
    uint16_t         ki_high_count;
    uint16_t         ki_low_count;
    bool             ki_boosted;
    SpeedPiKpState_t kp_state;
} SpeedPiScheduler_t;

static float Foc_Speed_Kp_Base = PID_SPEED_LOOP_KP;
static float Foc_Speed_Ki_Base = PID_SPEED_LOOP_KI;

static SpeedPiScheduler_t Foc_SpeedPi
    = {.kp_mul            = 1.0F,
       .ki_mul            = 1.0F,
       .kp_mul_max        = SPEED_PI_KP_MUL_MAX,
       .ki_mul_max        = SPEED_PI_KI_MUL_MAX,
       .kp_up_start_err   = SPEED_PI_KP_UP_START_ERR,
       .kp_up_end_err     = SPEED_PI_KP_UP_END_ERR,
       .kp_down_start_err = SPEED_PI_KP_DOWN_START_ERR,
       .kp_down_end_err   = SPEED_PI_KP_DOWN_END_ERR,
       .ki_switch_err     = SPEED_PI_KI_SWITCH_ERR,
       .ki_confirm_cycles = SPEED_PI_KI_CONFIRM_CYCLES,
       .ki_high_count     = 0U,
       .ki_low_count      = 0U,
       .ki_boosted        = false,
       .kp_state          = SPEED_PI_KP_LOW};

FluxExperiment_t Experiment = {0};

void Foc_Set_SampleTime(const SystemTimeConfig_t* config)
{
    Foc_Current_Ts      = config->current.val;  // 电流环采样周期
    Foc_Current_Freq    = config->current.inv;  // 电流环频率
    Foc_Speed_Ts        = config->speed.val;    // 转速环采样周期
    Foc_Speed_Freq      = config->speed.inv;    // 转速环频率
    Foc_Speed_Prescaler = (uint16_t)config->prescaler;  // 转速环分频数
}

void Foc_Set_Mode(FocMode_t mode)
{
    Foc_Mode = mode;  // 设置FOC模式
}

FocMode_t Foc_Get_Mode(void)
{
    return Foc_Mode;  // 获取FOC模式
}

void Foc_Set_ResetFlag(bool reset)
{
    Foc_Reset = reset;  // 设置复位标志
}

bool Foc_Get_ResetFlag(void)
{
    if (Foc_Mode == IDLE)
    {
        return true;  // 在IDLE模式下始终返回true
    }
    return Foc_Reset;  // 获取复位标志状态
}

// bool Foc_Get_ResetFlag(void) {
//     if (Foc_Mode_Prev != Foc_Mode) {
//         // 防止意外切换模式
//         Foc_Mode_Prev = Foc_Mode;
//         return true;
//     }
//     if (Foc_Mode == VF_MODE || Foc_Mode == IF_MODE
//         || Foc_Mode == SPEED) {
//         return Foc_Reset;  // 获取复位标志状态
//     }
//     return true;  // 在IDLE模式下始终返回true
// }

void Foc_Set_Angle(float angle)
{
    Foc_Theta = wrap_theta_2pi(angle);  // 确保角度在 [0, 2π) 范围内
}

void Foc_Set_BusVoltage(float voltage)
{
    Foc_BusVoltage = voltage;  // 设置母线电压
}

float Foc_Get_BusVoltage(void)
{
    return Foc_BusVoltage;  // 获取母线电压
}

Park_t Foc_Get_Inductor(void)
{
    return Mtpa_Get_LPark();  // 获取电感
}

void Foc_Set_BusVoltageInv(float voltage)
{
    Foc_BusVoltage_Inv = voltage;  // 设置母线电压倒数
}

void Foc_Set_Speed(float speed)
{
    Foc_Speed_Fdbk = speed;  // 设置参考速度
}

void Foc_Request_StartupPrepare(void)
{
    Foc_StartupPrepareRequest = true;
}

float Foc_Get_SpeedRamp(void)
{
    return Foc_Speed_Ramp;
}

float Foc_Get_SpeedTarget(void)
{
    return Foc_Speed_Ref;
}

void Foc_Set_Speed_and_Angle(AngleResult_t* angle_speed)
{
    Foc_Theta      = wrap_theta_2pi(angle_speed->theta);
    Foc_Speed_Fdbk = angle_speed->speed;
}

void Foc_Set_Iclark_Fdbk(Clark_t current)
{
    Foc_Iclark_Fdbk = current;  // 设置电流反馈
}

Clark_t Foc_Get_Iclark_Fdbk(void)
{
    return Foc_Iclark_Fdbk;  // 获取αβ轴电流反馈
}

void Foc_Set_Idq_Ref(Park_t idq_ref)
{
    Foc_Idq_Ref = idq_ref;  // 设置DQ轴电流参考
}

Park_t Foc_Get_Idq_Ref(void)
{
    return Foc_Idq_Ref;  // 获取DQ轴电流参考
}

void Foc_Set_Idq_Fdbk(Park_t idq_fdbk)
{
    Foc_Idq_Fdbk = idq_fdbk;  // 设置DQ轴电流反馈
}

Park_t Foc_Get_Idq_Fdbk(void)
{
    return Foc_Idq_Fdbk;  // 获取DQ轴电流反馈
}

void Foc_Set_Udq_Ref(Park_t udq_ref)
{
    Foc_Udq_Ref = udq_ref;  // 设置DQ轴电压参考

    // 将DQ轴电压参考转换为αβ轴电压参考
    Foc_Uclark_Ref = InvParkTransform(Foc_Udq_Ref, Foc_Theta);
}

Park_t Foc_Get_Udq_Ref(void)
{
    return Foc_Udq_Ref;  // 获取DQ轴电压参考
}

void Foc_Set_Uclark_Ref(Clark_t uclark_ref)
{
    Foc_Uclark_Ref = uclark_ref;  // 设置αβ轴电压参考
}

Clark_t Foc_Get_Uclark_Ref(void)
{
    return Foc_Uclark_Ref;  // 获取αβ轴电压参考
}

void Foc_Set_Vf_Param(VF_Parameter_t* vf_param)
{
    Foc_VfParam = *vf_param;  // 设置VF参数
}

void Foc_Set_If_Param(IF_Parameter_t* if_param)
{
    Foc_IfParam = *if_param;  // 设置IF参数
}

void Foc_Set_Pid_Speed_Handler(PID_Handler_t* handler)
{
    Foc_Pid_Speed_Handler = *handler;  // 设置速度环PID控制器
    Foc_Speed_Kp_Base     = handler->Kp;
    Foc_Speed_Ki_Base     = handler->Ki;

    Foc_SpeedPi.kp_mul        = 1.0F;
    Foc_SpeedPi.ki_mul        = 1.0F;
    Foc_SpeedPi.kp_state      = SPEED_PI_KP_LOW;
    Foc_SpeedPi.ki_boosted    = false;
    Foc_SpeedPi.ki_high_count = 0U;
    Foc_SpeedPi.ki_low_count  = 0U;
}

void Foc_Set_Pid_CurD_Handler(PID_Handler_t* handler)
{
    Foc_Pid_CurD_Handler = *handler;  // 设置D轴电流环PID控制器
}

void Foc_Set_Pid_CurQ_Handler(PID_Handler_t* handler)
{
    Foc_Pid_CurQ_Handler = *handler;  // 设置Q轴电流环PID控制器
}

void Foc_Set_Ramp_Speed_Handler(RampGenerator_t* handler)
{
    Foc_Ramp_Speed_Handler = *handler;  // 设置速度环斜坡生成器
}

static inline Phase_t calculate_SVPWM_Tcm(Clark_t u_ref, float inv_Vdc)
{
    float   alpha  = u_ref.a;
    float   beta   = u_ref.b;
    uint8_t sector = 0;
    float   v_ref1 = beta;
    float   v_ref2 = (+SQRT3 * alpha - beta) * 0.5F;
    float   v_ref3 = (-SQRT3 * alpha - beta) * 0.5F;

    // 判断扇区（1~6）
    if (v_ref1 > 0)
    {
        sector += 1;
    }
    if (v_ref2 > 0)
    {
        sector += 2;
    }
    if (v_ref3 > 0)
    {
        sector += 4;
    }

    // Clarke to t1/t2 projection
    float X = SQRT3 * beta * inv_Vdc;
    float Y = (+1.5F * alpha + SQRT3_2 * beta) * inv_Vdc;
    float Z = (-1.5F * alpha + SQRT3_2 * beta) * inv_Vdc;

    float t1 = 0.0F, t2 = 0.0F;

    switch (sector)
    {
    case 1:
        t1 = Z;
        t2 = Y;
        break;
    case 2:
        t1 = Y;
        t2 = -X;
        break;
    case 3:
        t1 = -Z;
        t2 = X;
        break;
    case 4:
        t1 = -X;
        t2 = Z;
        break;
    case 5:
        t1 = X;
        t2 = -Y;
        break;
    case 6:
        t1 = -Y;
        t2 = -Z;
        break;
    default:
        t1 = 0.0F;
        t2 = 0.0F;
        break;
    }

    // 过调制处理
    float T_sum = t1 + t2;
    if (T_sum > 1.0F)
    {
        t1 /= T_sum;
        t2 /= T_sum;
    }

    // 中心对称调制时间计算
    float t0 = (1.0F - t1 - t2) * 0.5F;
    float ta = t0;
    float tb = t0 + t1;
    float tc = tb + t2;

    Phase_t tcm = {0.0F, 0.0F, 0.0F};

    // 扇区映射到ABC换相点
    switch (sector)
    {
    case 1:
        tcm.a = tb;
        tcm.b = ta;
        tcm.c = tc;
        break;
    case 2:
        tcm.a = ta;
        tcm.b = tc;
        tcm.c = tb;
        break;
    case 3:
        tcm.a = ta;
        tcm.b = tb;
        tcm.c = tc;
        break;
    case 4:
        tcm.a = tc;
        tcm.b = tb;
        tcm.c = ta;
        break;
    case 5:
        tcm.a = tc;
        tcm.b = ta;
        tcm.c = tb;
        break;
    case 6:
        tcm.a = tb;
        tcm.b = tc;
        tcm.c = ta;
        break;
    default:
        tcm.a = 0.5F;
        tcm.b = 0.5F;
        tcm.c = 0.5F;
        break;
    }

    return tcm;
}

Phase_t Foc_Get_Tcm(void)
{
    Phase_t tcm = {.a = 0.5F, .b = 0.5F, .c = 0.5F};
    if (Foc_Reset)
    {
        return tcm;  // 如果复位标志为真，直接返回零值
    }
    // 生成三相PWM时间
    tcm = calculate_SVPWM_Tcm(Foc_Uclark_Ref, Foc_BusVoltage_Inv);

    return tcm;  // 返回三相PWM时间
}

static inline float dispatch_current(float cur_ref)
{
    if (cur_ref < 0.0F)
    {
        cur_ref = -cur_ref;
    }
    // float out = 0.0003497F * cur_ref * cur_ref * cur_ref - 0.02016F * cur_ref * cur_ref +
    //             0.7335F * cur_ref + 0.6032F;  // 三次函数拟合
    float out = 0.0F;
    MTPA_interp_by_Iq(
        mtpa_table, MTPA_TABLE_POINTS, cur_ref, &out, NULL);
    return out;
}

static inline Park_t Foc_Update_SpeedLoop(float ref,
                                          float fdbk,
                                          bool  reset)
{
    static uint16_t counter = 0x0000U;
    if (reset)
    {
        counter                        = 0x0000U;
        Foc_Ramp_Speed_Handler.value  = 0.0F;
        Foc_Ramp_Speed_Handler.target = 0.0F;
        Foc_Speed_Ramp                = 0.0F;
        Foc_Idq_Ref.d                 = 0.0F;
        Foc_Idq_Ref.q                 = 0.0F;
        Pid_Update(0.0F, true, &Foc_Pid_Speed_Handler);
        return Foc_Idq_Ref;
    }

    counter++;
    if (counter < Foc_Speed_Prescaler)
    {
        return Foc_Idq_Ref;  // 如果未到达分频点，直接返回参考值
    }
    counter                       = 0x0000U;
    Foc_Ramp_Speed_Handler.target = ref;  // 更新目标速度
    Park_t output                 = {0};
    float  ramp    = RampGenerator(&Foc_Ramp_Speed_Handler, reset);
    Foc_Speed_Ramp = ramp;
    output.q = Pid_Update(ramp - fdbk, reset, &Foc_Pid_Speed_Handler);
#if defined(FOC_DEBUG_IQ)
    static float iqtest = 0;
    iqtest              = iqtest + 0.0002F;
    if (iqtest > 21.0F)
    {
        iqtest = 0.0F;
    }
    output.q = iqtest;
#endif
    output.d = dispatch_current(output.q);

    return output;  // 返回DQ轴电流参考
}

static inline uint16_t Foc_Get_StartupHoldCycles(void)
{
    float cycles = SENSORLESS_STARTUP_HOLD_TIME * Foc_Current_Freq;
    if (cycles < 1.0F)
    {
        cycles = 1.0F;
    }
    if (cycles > (float)UINT16_MAX)
    {
        cycles = (float)UINT16_MAX;
    }
    return (uint16_t)(cycles + 0.5F);
}

static inline Park_t Foc_Update_CurrentLoop(Park_t ref,
                                            Park_t fdbk,
                                            bool   reset)
{
    Park_t output = {0};

    Pid_Update(ref.d - fdbk.d, reset, &Foc_Pid_CurD_Handler);
    Pid_Update(ref.q - fdbk.q, reset, &Foc_Pid_CurQ_Handler);

    output.d = Foc_Pid_CurD_Handler.output;
    output.q = Foc_Pid_CurQ_Handler.output;

    return output;
}

static inline float speed_pi_clampf(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

static inline float speed_pi_get_kp_mul(float abs_err)
{
    float span = 0.0F;
    float gain = 1.0F;
    float t    = 0.0F;

    switch (Foc_SpeedPi.kp_state)
    {
    case SPEED_PI_KP_HIGH:
        return Foc_SpeedPi.kp_mul_max;
    case SPEED_PI_KP_RAMP_UP:
        span = Foc_SpeedPi.kp_up_end_err - Foc_SpeedPi.kp_up_start_err;
        if (span <= 0.0F)
        {
            return Foc_SpeedPi.kp_mul_max;
        }
        t = (speed_pi_clampf(abs_err,
                             Foc_SpeedPi.kp_up_start_err,
                             Foc_SpeedPi.kp_up_end_err)
             - Foc_SpeedPi.kp_up_start_err)
            / span;
        gain = 1.0F + t * (Foc_SpeedPi.kp_mul_max - 1.0F);
        return gain;
    case SPEED_PI_KP_RAMP_DOWN:
        span = Foc_SpeedPi.kp_down_start_err
               - Foc_SpeedPi.kp_down_end_err;
        if (span <= 0.0F)
        {
            return 1.0F;
        }
        t = (speed_pi_clampf(abs_err,
                             Foc_SpeedPi.kp_down_end_err,
                             Foc_SpeedPi.kp_down_start_err)
             - Foc_SpeedPi.kp_down_end_err)
            / span;
        gain = 1.0F + t * (Foc_SpeedPi.kp_mul_max - 1.0F);
        return gain;
    case SPEED_PI_KP_LOW:
    default:
        return 1.0F;
    }
}

static inline uint16_t speed_pi_get_confirm_ticks(void)
{
    uint32_t ticks = (uint32_t)Foc_SpeedPi.ki_confirm_cycles;

    if (Foc_Speed_Prescaler == 0U)
    {
        return (ticks == 0U) ? 1U : (uint16_t)ticks;
    }

    ticks *= (uint32_t)Foc_Speed_Prescaler;
    if (ticks == 0U)
    {
        ticks = 1U;
    }
    if (ticks > UINT16_MAX)
    {
        ticks = UINT16_MAX;
    }
    return (uint16_t)ticks;
}

static inline void speed_pi_update_gain(float speed_err, bool reset)
{
    uint16_t confirm_ticks = 0U;
    float    abs_speed_err = fabsf(speed_err);

    if (reset)
    {
        Foc_SpeedPi.kp_mul        = 1.0F;
        Foc_SpeedPi.ki_mul        = 1.0F;
        Foc_SpeedPi.kp_state      = SPEED_PI_KP_LOW;
        Foc_SpeedPi.ki_boosted    = false;
        Foc_SpeedPi.ki_high_count = 0U;
        Foc_SpeedPi.ki_low_count  = 0U;

        Foc_Pid_Speed_Handler.Kp = Foc_Speed_Kp_Base;
        Foc_Pid_Speed_Handler.Ki = Foc_Speed_Ki_Base;
        return;
    }

    if (fabsf(Foc_Speed_Ref) <= SPEED_PI_ENABLE_REF_MIN)
    {
        Foc_SpeedPi.kp_mul        = 1.0F;
        Foc_SpeedPi.ki_mul        = 1.0F;
        Foc_SpeedPi.kp_state      = SPEED_PI_KP_LOW;
        Foc_SpeedPi.ki_boosted    = false;
        Foc_SpeedPi.ki_high_count = 0U;
        Foc_SpeedPi.ki_low_count  = 0U;

        Foc_Pid_Speed_Handler.Kp = Foc_Speed_Kp_Base;
        Foc_Pid_Speed_Handler.Ki = Foc_Speed_Ki_Base;
        return;
    }

    switch (Foc_SpeedPi.kp_state)
    {
    case SPEED_PI_KP_LOW:
        if (abs_speed_err >= Foc_SpeedPi.kp_up_start_err)
        {
            Foc_SpeedPi.kp_state = SPEED_PI_KP_RAMP_UP;
        }
        break;
    case SPEED_PI_KP_RAMP_UP:
        if (abs_speed_err >= Foc_SpeedPi.kp_up_end_err)
        {
            Foc_SpeedPi.kp_state = SPEED_PI_KP_HIGH;
        }
        else if (abs_speed_err < Foc_SpeedPi.kp_up_start_err)
        {
            Foc_SpeedPi.kp_state = SPEED_PI_KP_LOW;
        }
        break;
    case SPEED_PI_KP_HIGH:
        if (abs_speed_err <= Foc_SpeedPi.kp_down_start_err)
        {
            Foc_SpeedPi.kp_state = SPEED_PI_KP_RAMP_DOWN;
        }
        break;
    case SPEED_PI_KP_RAMP_DOWN:
        if (abs_speed_err > Foc_SpeedPi.kp_down_start_err)
        {
            Foc_SpeedPi.kp_state = SPEED_PI_KP_HIGH;
        }
        else if (abs_speed_err <= Foc_SpeedPi.kp_down_end_err)
        {
            Foc_SpeedPi.kp_state = SPEED_PI_KP_LOW;
        }
        break;
    default:
        Foc_SpeedPi.kp_state = SPEED_PI_KP_LOW;
        break;
    }

    Foc_SpeedPi.kp_mul = speed_pi_get_kp_mul(abs_speed_err);

    confirm_ticks = speed_pi_get_confirm_ticks();
    if (abs_speed_err > Foc_SpeedPi.ki_switch_err)
    {
        if (Foc_SpeedPi.ki_high_count < confirm_ticks)
        {
            Foc_SpeedPi.ki_high_count++;
        }
        Foc_SpeedPi.ki_low_count = 0U;
        if (Foc_SpeedPi.ki_high_count >= confirm_ticks)
        {
            Foc_SpeedPi.ki_boosted = true;
        }
    }
    else
    {
        if (Foc_SpeedPi.ki_low_count < confirm_ticks)
        {
            Foc_SpeedPi.ki_low_count++;
        }
        Foc_SpeedPi.ki_high_count = 0U;
        if (Foc_SpeedPi.ki_low_count >= confirm_ticks)
        {
            Foc_SpeedPi.ki_boosted = false;
        }
    }

    Foc_SpeedPi.ki_mul
        = Foc_SpeedPi.ki_boosted ? Foc_SpeedPi.ki_mul_max : 1.0F;

    Foc_Pid_Speed_Handler.Kp = Foc_Speed_Kp_Base * Foc_SpeedPi.kp_mul;
    Foc_Pid_Speed_Handler.Ki = Foc_Speed_Ki_Base * Foc_SpeedPi.ki_mul;
}

static inline Park_t Foc_Update_VfMode(bool reset)
{
    static bool  reset_prev = true;
    static bool  sweep_flag = false;
    static float phase_prev = 0.0F;
    Park_t       output     = {0};
    if (reset_prev && !reset)
    {
        SawtoothWave_Init(&Foc_Sawtooth_Handler,
                          M_2PI,
                          Foc_VfParam.freq,
                          0.0F,
                          Foc_Current_Ts);
    }
    Foc_Sawtooth_Handler.frequency = Foc_VfParam.freq;
    output = Foc_VfParam.vol_ref;  // 获取电压参考

    float phase = 0.0F;
    phase       = SawtoothWaveGenerator(&Foc_Sawtooth_Handler,
                                  reset);  // 更新电压环角度
    if (fabsf(phase - phase_prev) > M_PI_2)
    {
        sweep_flag = false;
    }
    if (Foc_Sweep)
    {
        sweep_flag = true;
    }
    if (sweep_flag)
    {
        phase_prev = phase;
    }
    Foc_Theta = wrap_theta_2pi(phase_prev + Foc_VfParam.offset);

    Foc_Idq_Fdbk = ParkTransform(Foc_Iclark_Fdbk, Foc_Theta);
    reset_prev   = reset;
    return output;
}

static inline Park_t Foc_Update_IfMode(bool reset)
{
    static bool  reset_prev = true;
    static bool  sweep_flag = false;
    static float phase_prev = 0.0F;
    Park_t       output     = {0};
    // 如果传感器状态为启用，则直接使用参考值，否则使用正弦波生成器
    if (Foc_IfParam.use_sensor == true)
    {
        Foc_IfParam.offset = 0.0F;
    }
    else
    {
        if (reset_prev && !reset)
        {
            SawtoothWave_Init(&Foc_Sawtooth_Handler,
                              M_2PI,
                              Foc_IfParam.freq,
                              0.0F,
                              Foc_Current_Ts);
        }
        Foc_Sawtooth_Handler.frequency = Foc_IfParam.freq;

        float phase = 0.0F;
        phase       = SawtoothWaveGenerator(&Foc_Sawtooth_Handler,
                                      reset);  // 更新电流环角度
        if (fabsf(phase - phase_prev) > M_PI_2)
        {
            sweep_flag = false;
        }
        if (Foc_Sweep)
        {
            sweep_flag = true;
        }
        if (sweep_flag)
        {
            phase_prev = phase;
        }

        Foc_Theta = wrap_theta_2pi(phase_prev + Foc_IfParam.offset);
    }

    Foc_Idq_Fdbk = ParkTransform(Foc_Iclark_Fdbk, Foc_Theta);

    output = Foc_Update_CurrentLoop(
        Foc_IfParam.cur_ref, Foc_Idq_Fdbk, reset);

    reset_prev = reset;
    return output;
}

static inline Park_t Foc_Update_SpeedMode(bool reset)
{
    static bool     reset_prev     = true;
    static bool     startup_active = false;
    static uint16_t startup_count  = 0x0000U;
    float           speed_err_abs  = 0.0F;
    Park_t          output         = {0};

    if (Foc_StartupPrepareRequest)
    {
        startup_active            = true;
        startup_count             = 0x0000U;
        Foc_StartupPrepareRequest = false;
    }

    if (reset_prev && !reset)
    {
        startup_active = true;
        startup_count  = 0x0000U;
    }

    if (reset)
    {
        Foc_Speed_Ref  = 0.0F;
        startup_active = false;
        startup_count  = 0x0000U;
        Foc_StartupPrepareRequest = false;
    }

    Foc_Idq_Fdbk = ParkTransform(Foc_Iclark_Fdbk, Foc_Theta);

    if (startup_active)
    {
        speed_pi_update_gain(0.0F, true);
        (void)Foc_Update_SpeedLoop(0.0F, Foc_Speed_Fdbk, true);

        output.d = SENSORLESS_STARTUP_UD;
        output.q = 0.0F;

        startup_count++;
        if (startup_count >= Foc_Get_StartupHoldCycles())
        {
            startup_active = false;
        }

        reset_prev = reset;
        return output;
    }

    speed_err_abs = fabsf(Foc_Speed_Ramp - Foc_Speed_Fdbk);
    speed_pi_update_gain(speed_err_abs, reset);

    Foc_Idq_Ref
        = Foc_Update_SpeedLoop(Foc_Speed_Ref, Foc_Speed_Fdbk, reset);

    output = Foc_Update_CurrentLoop(Foc_Idq_Ref, Foc_Idq_Fdbk, reset);

    if (output.q > PID_CURRENT_Q_LOOP_MAX_OUTPUT)
    {
        output.q = PID_CURRENT_Q_LOOP_MAX_OUTPUT;
    }
    if (output.q < -PID_CURRENT_Q_LOOP_MAX_OUTPUT)
    {
        output.q = -PID_CURRENT_Q_LOOP_MAX_OUTPUT;
    }

    reset_prev = reset;
    return output;
}

Park_t Foc_Update_Main(void)
{
    Park_t output = {0};

    switch (Foc_Mode)
    {
    case VF_MODE:
    {
        output = Foc_Update_VfMode(Foc_Reset);  // VF模式
        break;
    }
    case IF_MODE:
    {
        output = Foc_Update_IfMode(Foc_Reset);  // IF模式
        break;
    }
    case STARTUP:
    {
        output.d = SENSORLESS_STARTUP_UD;  // D轴电压参考为5
        output.q = 0.0F;  // Q轴电压参考为0
        break;
    }
    case SPEED:
    {
        output = Foc_Update_SpeedMode(Foc_Reset);  // 转速模式
        break;
    }
    case IDENTIFY:
    {
        if (Experiment.Initialized == false)
        {
            Experiment_Init(&Experiment,
                            Foc_Current_Ts,
                            SAMPLE_CAPACITY,
                            REPEAT_TIMES,
                            MAX_STEPS,
                            3,
                            12,
                            1,
                            100);
            MTPA_build_table(
                mtpa_table, MTPA_TABLE_POINTS, 0.0f, 50.0f);
        }
        else if (Experiment.Complete == true)
        {
            float ad0 = 0.0F, add = 0.0F, aq0 = 0.0F, aqq = 0.0F,
                  adq = 0.0F;
            Get_Identification_Results(
                &Experiment, &ad0, &add, &aq0, &aqq, &adq);
            MTPA_Get_Parameter(ad0, add, aq0, aqq, adq);
            MTPA_build_table(
                mtpa_table, MTPA_TABLE_POINTS, 0.0f, 50.0f);
            Foc_Mode = IDLE;
            break;
        }
        Foc_Idq_Fdbk = ParkTransform(Foc_Iclark_Fdbk, Foc_Theta);
        Experiment_Step(&Experiment,
                        Foc_Idq_Fdbk.d,
                        Foc_Idq_Fdbk.q,
                        &output.d,
                        &output.q);
        break;
    }
    default:
    {
        output.d = 0.0F;  // 默认情况下，D轴电压参考为0
        output.q = 0.0F;  // Q轴电压参考为0

        break;
    }
    }
    Foc_Udq_Ref    = output;
    Foc_Uclark_Ref = InvParkTransform(Foc_Udq_Ref, Foc_Theta);
    return output;  // 返回DQ轴电压参考
}
