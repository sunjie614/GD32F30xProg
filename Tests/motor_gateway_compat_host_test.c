#include <math.h>
#include <stdio.h>
#include "fixed_numeric_config.h"
#include "mc_math.h"
#include "motor.h"

extern volatile MotorParam_t motor_param;
extern volatile bool Motor_Initialized;
extern volatile float Motor_Position;
extern volatile float Motor_Position_Scale;
extern volatile float Motor_Position_Offset;
extern volatile float Motor_Pn;
extern volatile float Motor_Theta_Elec;
extern volatile float Motor_Theta_Mech;
extern volatile float Motor_Speed;
extern volatile float Motor_SampleFreq;
extern volatile uint16_t Motor_Speed_Prescaler;

static unsigned failures;
static void near(const char* name, float actual, float expected, float tolerance)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance)
    {
        fprintf(stderr, "FAIL %s: actual=%g expected=%g\n", name, actual, expected);
        ++failures;
    }
}
static void check(const char* name, bool value)
{
    if (!value) { fprintf(stderr, "FAIL %s\n", name); ++failures; }
}
static MotorParam_t parameters(void)
{
    return (MotorParam_t){.Rs = 0.65F, .Ld = 0.1F, .Lq = 0.016F,
        .Flux = 0.0F, .Pn = 2.0F, .Resolver_Pn = 1.0F,
        .inv_MotorPn = 0.5F, .Position_Scale = 9999.0F,
        .Position_Offset = 6838.0F, .theta_factor = MC_TWO_PI_F / 10000.0F};
}
static void initialize(void)
{
    MotorParam_t initial = parameters();
    SystemTimeConfig_t time = {.current = {.val = 0.0002F, .inv = 5000.0F},
        .speed = {.val = 0.002F, .inv = 500.0F}, .prescaler = 10.0F};
    check("initialize", Motor_Initialization(&initial));
    check("sample time", Motor_Set_SampleTime(&time));
    check("prescaler", Motor_Set_SpeedPrescaler(10U));
    check("filter", Motor_Set_Filter(10.0F, 500.0F));
}
static void test_units(void)
{
    initialize();
    check("initialized", Motor_Initialized);
    near("scale counts", Motor_Position_Scale, 9999.0F, 0.0F);
    near("offset counts", Motor_Position_Offset, 6838.0F, 0.0F);
    near("structured offset", motor_param.Position_Offset, 6838.0F, 0.0F);
    near("pole pairs", Motor_Pn, 2.0F, 0.00001F);
    near("sample Hz", Motor_SampleFreq, 500.0F, 0.0F);
    check("prescaler mirror", Motor_Speed_Prescaler == 10U);
    Motor_Set_Position(6838U);
    near("raw position count", Motor_Position, 6838.0F, 0.0F);
    near("zero mechanical rad", Motor_Theta_Mech, 0.0F, 0.000002F);
    near("zero electrical rad", Motor_Theta_Elec, 0.0F, 0.000002F);
    Motor_Set_Position(9338U);
    near("mechanical rad", Motor_Theta_Mech, MC_TWO_PI_F * 0.25F, 0.000003F);
    near("electrical rad", Motor_Theta_Elec, MC_TWO_PI_F * 0.5F, 0.000003F);
    near("electrical getter", Motor_Get_ThetaElec(), Motor_Theta_Elec, 0.0F);
    near("mechanical getter", Motor_Get_Theta_Mech(), Motor_Theta_Mech, 0.0F);
    Motor_Set_Theta_Elec(MC_TWO_PI_F * 1.25F);
    near("electrical setter normalized", Motor_Theta_Elec, MC_TWO_PI_F * 0.25F, 0.000003F);
    Motor_Set_Theta_Mech(-MC_TWO_PI_F * 0.25F);
    near("mechanical setter normalized", Motor_Theta_Mech, MC_TWO_PI_F * 0.75F, 0.000003F);
    Motor_Set_Speed(-321.0F);
    near("speed rpm", Motor_Speed, -321.0F, 0.0002F);
    near("speed getter", Motor_Get_Speed(), Motor_Speed, 0.0F);
}
static void test_offset_inputs(void)
{
    initialize();
    motor_param.Position_Offset = 7000.0F;
    Motor_Set_Position(7000U);
    near("structured changes angle", Motor_Theta_Elec, 0.0F, 0.000002F);
    near("structured writes alias", Motor_Position_Offset, 7000.0F, 0.0F);
    Motor_Position_Offset = 6000.0F;
    Motor_Set_Position(6000U);
    near("alias changes angle", Motor_Theta_Elec, 0.0F, 0.000002F);
    near("alias writes structured", motor_param.Position_Offset, 6000.0F, 0.0F);
    motor_param.Position_Offset = 1000.0F;
    Motor_Position_Offset = 2000.0F;
    Motor_Set_Position(1000U);
    near("structured priority angle", Motor_Theta_Elec, 0.0F, 0.000002F);
    near("conflict actual offset", Motor_Position_Offset, 1000.0F, 0.0F);
    const float invalid[] = {1000.5F, -1.0F, 10000.0F, NAN, INFINITY};
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        uint32_t count = McMath_Diagnostics.invalid_input_count;
        motor_param.Position_Offset = invalid[i];
        Motor_Set_Position(1000U);
        near("invalid structured readback", motor_param.Position_Offset, 1000.0F, 0.0F);
        near("invalid alias readback", Motor_Position_Offset, 1000.0F, 0.0F);
        near("invalid keeps angle", Motor_Theta_Elec, 0.0F, 0.000002F);
        check("invalid counted", McMath_Diagnostics.invalid_input_count == count + 1U);
    }
    motor_param.Position_Offset = NAN;
    Motor_Position_Offset = 3000.0F;
    Motor_Set_Position(1000U);
    near("invalid winner rejects alias", Motor_Position_Offset, 1000.0F, 0.0F);
    Motor_Position_Offset = 1000.25F;
    Motor_Set_Position(1000U);
    near("fractional alias rejected", Motor_Position_Offset, 1000.0F, 0.0F);
    motor_param.Pn = 4.0F;
    motor_param.Position_Scale = 19999.0F;
    motor_param.Rs = 99.0F;
    Motor_Set_Position(3500U);
    near("unsupported Pn readback", motor_param.Pn, 2.0F, 0.0F);
    near("unsupported scale readback", motor_param.Position_Scale, 9999.0F, 0.0F);
    near("unsupported Rs readback", motor_param.Rs, 0.65F, 0.0F);
    near("unsupported writes no angle effect", Motor_Theta_Elec, MC_TWO_PI_F * 0.5F, 0.000003F);
}
static void test_zero_shift(void)
{
    MotorParam_t initial = parameters();
    initial.Position_Offset = 0.0F;
    check("zero offset init", Motor_Initialization(&initial));
    SystemTimeConfig_t time = {.speed = {.inv = 500.0F}};
    check("stationary sample time", Motor_Set_SampleTime(&time));
    check("stationary prescaler", Motor_Set_SpeedPrescaler(1U));
    check("stationary filter", Motor_Set_Filter(10.0F, 500.0F));
    Motor_Set_Position(0U);
    motor_param.Position_Offset = 1000.0F;
    Motor_Set_Position(0U);
    near("offset no false speed", Motor_Speed, 0.0F, 0.001F);
    near("shifted mechanical angle", Motor_Theta_Mech, MC_TWO_PI_F * 0.9F, 0.000004F);
    Motor_Set_Position(10U);
    check("real rotation changes speed", Motor_Speed > 1.0F);
}
static void test_invalid_initialization(void)
{
    initialize();
    MotorParam_t initial = parameters();
    initial.Position_Offset = 10.5F;
    check("partial count init rejected", !Motor_Initialization(&initial));
    initial = parameters(); initial.Position_Scale = NAN;
    check("NaN scale rejected", !Motor_Initialization(&initial));
    initial = parameters(); initial.Position_Offset = INFINITY;
    check("infinite offset rejected", !Motor_Initialization(&initial));
    initial = parameters(); initial.Pn = NAN;
    check("NaN Pn rejected", !Motor_Initialization(&initial));
    Motor_Set_Position(6838U);
    near("invalid reinit preserves state", Motor_Theta_Elec, 0.0F, 0.000002F);
}
int main(void)
{
    McMath_ResetDiagnostics();
    test_units();
    test_offset_inputs();
    test_zero_shift();
    test_invalid_initialization();
    if (failures != 0U) return 1;
    puts("PASS: legacy motor A2L physical mirrors and live integer offset calibration");
    return 0;
}
