#include "transformation_iq.h"

mc_real_t AngleIq_WrapPu(mc_real_t angle_pu)
{
    while (angle_pu >= MC_ONE)
        angle_pu = McMath_Sub(angle_pu, MC_ONE);
    while (angle_pu < MC_ZERO)
        angle_pu = McMath_Add(angle_pu, MC_ONE);
    return angle_pu;
}

mc_real_t AngleIq_ErrorPu(mc_real_t reference_pu, mc_real_t feedback_pu)
{
    mc_real_t error = McMath_Sub(reference_pu, feedback_pu);
    if (error > MC_HALF)
        error = McMath_Sub(error, MC_ONE);
    else if (error < McMath_Neg(MC_HALF))
        error = McMath_Add(error, MC_ONE);
    return error;
}

ClarkIq_t TransformIq_Clarke(PhaseIq_t input)
{
    ClarkIq_t output;
    output.a = McMath_Sub(
        McMath_Mul(MC_CONST(0.6666666666667), input.a),
        McMath_Mul(MC_CONST(0.3333333333333),
                   McMath_Add(input.b, input.c)));
    output.b = McMath_Mul(MC_CONST(0.5773502691896),
                          McMath_Sub(input.b, input.c));
    return output;
}

ParkIq_t TransformIq_Park(ClarkIq_t input, mc_real_t angle_pu)
{
    mc_real_t cosine = McMath_CosPu(angle_pu);
    mc_real_t sine = McMath_SinPu(angle_pu);
    return (ParkIq_t){
        .d = McMath_Add(McMath_Mul(input.a, cosine),
                        McMath_Mul(input.b, sine)),
        .q = McMath_Sub(McMath_Mul(input.b, cosine),
                        McMath_Mul(input.a, sine))};
}

ClarkIq_t TransformIq_InversePark(ParkIq_t input, mc_real_t angle_pu)
{
    mc_real_t cosine = McMath_CosPu(angle_pu);
    mc_real_t sine = McMath_SinPu(angle_pu);
    return (ClarkIq_t){
        .a = McMath_Sub(McMath_Mul(input.d, cosine),
                        McMath_Mul(input.q, sine)),
        .b = McMath_Add(McMath_Mul(input.d, sine),
                        McMath_Mul(input.q, cosine))};
}
