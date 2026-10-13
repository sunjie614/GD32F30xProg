#ifndef SENSORLESS_IQ_GATEWAY_H
#define SENSORLESS_IQ_GATEWAY_H

#include "fixed_gateway_types.h"
#include "sensorless_iq.h"
#include "sensorless_mode.h"

/* A2L caches belong to the integration context, not to the algorithm. */
typedef struct
{
    float accepted[16];
    bool leso_enabled, hfi_enabled;
    sensorless_method_t method;
} SensorlessGatewayState_t;

extern volatile bool Sensorless_Enabled;
extern volatile sensorless_method_t Sensorless_Method;
extern volatile bool Leso_Enabled, Hfi_Enabled;
extern volatile float Sensorless_ThetAdj;
extern volatile float Sensorless_Switch_Speed;
extern volatile float Sensorless_Threshold_Hfi, Sensorless_Threshold_Leso;
extern volatile float PLLKPMAX, PLLKIMAX;
extern volatile A2lPidMirror_t Sensorless_Theta_PID;
extern volatile float Leso_Gain, Leso_Factor, Leso_Wc_Min, Leso_Wc_Max;
extern volatile float Leso_Rs, Hfi_InjVolt;

void SensorlessGateway_Init(SensorlessGatewayState_t* cache,
                            SensorlessIqConfig_t* config);
/* Call Read first to validate pending physical writes against the old cache. */
void SensorlessGateway_Reinitialize(SensorlessGatewayState_t* cache,
                                    SensorlessIqConfig_t* config);
void SensorlessGateway_Read(SensorlessGatewayState_t* cache,
                            SensorlessIqConfig_t* config,
                            LesoIqConfig_t* leso_config,
                            HfiIqConfig_t* hfi_config,
                            SensorlessIqState_t* state,
                            LesoIqState_t* leso, HfiIqState_t* hfi);
void SensorlessGateway_Publish(SensorlessGatewayState_t* cache,
                               const SensorlessIqState_t* state,
                               const LesoIqState_t* leso,
                               const HfiIqState_t* hfi,
                               const FlyingIqState_t* flying,
                               ParkIq_t inductance,
                               bool reset);

#endif
