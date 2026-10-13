#ifndef FIXED_DEBUG_GATEWAY_H
#define FIXED_DEBUG_GATEWAY_H

#include "foc_iq.h"
#include "transformation.h"

/* Physical-unit, observation-only aliases for legacy PiSnoop watch lists.
 * They never feed algorithm state back into the controller. */
extern volatile float Sensorless_ThetaEst, Sensorless_SpeedEst;
extern volatile float Sensorless_ThetaErr, Sensorless_SpeedErr;
extern volatile float Leso_Theta, Leso_Speed, Leso_Theta_Err, Leso_Speed_Err;
extern volatile float Hfi_Theta, Hfi_Speed, Hfi_Theta_Err, Hfi_Speed_Err;
extern volatile Clark_t Leso_CurEst, Leso_Current, Leso_Voltage, Leso_EmfEst;
extern volatile Park_t Leso_EmfEst_dq, Leso_Emf_Filtered;
extern volatile Park_t Leso_Emf_Slow_Filtered;
extern volatile Clark_t Hfi_IClarkFdbk, Hfi_IClarkResp, Hfi_IClarkFilt;
extern volatile Park_t MTPA_Inductor;

typedef struct
{
    float T_req, Psi_s, gamma, Id, Iq, Ld, Lq;
    bool valid;
} A2lMtpaPoint_t;
extern volatile A2lMtpaPoint_t mtpa_table[MTPA_IQ_TABLE_POINTS];

void FixedDebug_Publish(const FocIqState_t* state,
                        const FocIqInput_t* input,
                        const FocIqOutput_t* output);
/* Main-loop/startup only: never convert the complete table in the ADC ISR
 * or inside the interrupt-disabled table commit. */
void FixedDebug_PublishMtpa(const MtpaIqState_t* state);

#endif
