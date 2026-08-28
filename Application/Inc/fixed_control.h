#ifndef FIXED_CONTROL_H
#define FIXED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "foc_iq.h"
#include "transformation.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    Phase_t current_abc;
    float theta_rad;
    float speed_rpm;
    float bus_voltage;
    bool reset;
} FixedControlInput_t;

typedef struct
{
    Phase_t pwm_duty;
    Park_t voltage_dq;
    Park_t current_dq;
    float estimated_theta_rad;
    float estimated_speed_rpm;
} FixedControlOutput_t;

typedef struct
{
    uint32_t conversion_failures;
    uint32_t arithmetic_failures;
    uint32_t trigonometric_failures;
    uint32_t sqrt_failures;
} FixedControlSelfTest_t;

typedef struct
{
    FocIqState_t foc;
    FocIqParameters_t parameters;
    bool initialized;
} FixedControlState_t;

void FixedControl_Init(FixedControlState_t* state);
void FixedControl_Reset(FixedControlState_t* state);
FixedControlOutput_t FixedControl_Step(FixedControlState_t* state,
                                      const FixedControlInput_t* input);
bool FixedControl_BackgroundBuild(FixedControlState_t* state);
void FixedControl_CommitBackground(FixedControlState_t* state);
FixedControlSelfTest_t FixedControl_RunSelfTest(void);

/* A2L-compatible physical-unit gateway variables. */
extern volatile FocMode_t Foc_Mode;
extern volatile float Foc_Speed_Ref;
extern volatile float Foc_Speed_Ramp;
extern volatile float Foc_Speed_Fdbk;
extern volatile float Foc_Theta;
extern volatile float Foc_BusVoltage;
extern volatile float Foc_Id_Ref;
extern volatile float Foc_Iq_Ref;
extern volatile float Foc_Id_Fdbk;
extern volatile float Foc_Iq_Fdbk;
extern volatile float Foc_Ud_Ref;
extern volatile float Foc_Uq_Ref;
extern volatile bool MainInt_UseRealTheta;
extern volatile uint16_t Sensorless_Method;
extern volatile uint32_t FixedControl_CycleCount;
extern volatile FixedControlSelfTest_t FixedControl_SelfTest;

#ifdef __cplusplus
}
#endif

#endif
