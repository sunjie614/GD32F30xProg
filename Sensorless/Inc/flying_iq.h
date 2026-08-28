#ifndef FLYING_IQ_H
#define FLYING_IQ_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t count;
    uint16_t startup_delay;
    bool enabled;
    bool completed;
} FlyingIqState_t;

void FlyingIq_Init(FlyingIqState_t* state, uint16_t startup_delay);
void FlyingIq_Reset(FlyingIqState_t* state);
void FlyingIq_SetEnabled(FlyingIqState_t* state, bool enabled);
void FlyingIq_Run(FlyingIqState_t* state, bool reset);

#endif
