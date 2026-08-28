#ifndef TRANSFORMATION_IQ_H
#define TRANSFORMATION_IQ_H

#include "mc_math.h"

typedef struct
{
    mc_real_t a;
    mc_real_t b;
    mc_real_t c;
} PhaseIq_t;

typedef struct
{
    mc_real_t a;
    mc_real_t b;
} ClarkIq_t;

typedef struct
{
    mc_real_t d;
    mc_real_t q;
} ParkIq_t;

mc_real_t AngleIq_WrapPu(mc_real_t angle_pu);
mc_real_t AngleIq_ErrorPu(mc_real_t reference_pu, mc_real_t feedback_pu);
ClarkIq_t TransformIq_Clarke(PhaseIq_t input);
ParkIq_t TransformIq_Park(ClarkIq_t input, mc_real_t angle_pu);
ClarkIq_t TransformIq_InversePark(ParkIq_t input, mc_real_t angle_pu);

#endif
