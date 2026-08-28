#include "mtpa_iq.h"

#include <stddef.h>
#include "fixed_numeric_config.h"

#define MTPA_IQ_PSI_SCAN_STEPS 50U
#define MTPA_IQ_BISECT_STEPS   20U
#define MTPA_IQ_GOLDEN_STEPS   24U

typedef struct
{
    mc_real_t torque;
    mc_real_t id;
    mc_real_t iq;
    mc_real_t magnitude;
} MtpaIqEvaluation_t;

static mc_real_t mtpa_from_u32(uint32_t value)
{
#if defined(MC_NUMERIC_IQMATH)
    if (value > 127U)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MAX;
    }
    return (mc_real_t)((int32_t)value << 24);
#else
    return (mc_real_t)value;
#endif
}

static mc_real_t mtpa_pow_unsigned(mc_real_t value, uint8_t exponent)
{
    mc_real_t result = MC_ONE;
    mc_real_t base = McMath_Abs(value);
    for (uint8_t index = 0U; index < exponent; ++index)
        result = McMath_Mul(result, base);
    return result;
}

void MtpaIq_DefaultParameters(MtpaIqParameters_t* parameters)
{
    if (parameters == NULL)
        return;
    /* Convert the original physical model coefficients into the normalized
     * current/flux contract.  With a 1 Wb flux base this is coefficient/30 A. */
    *parameters = (MtpaIqParameters_t){
        .ad0 = MC_CONST(5.59756 / MC_CURRENT_BASE_A),
        .add = MC_CONST(5.15426 / MC_CURRENT_BASE_A),
        .aq0 = MC_CONST(6.306 / MC_CURRENT_BASE_A),
        .aqq = MC_CONST(171.571 / MC_CURRENT_BASE_A),
        .adq = MC_CONST(35.90 / MC_CURRENT_BASE_A),
        .d_exponent = 5U,
        .q_exponent = 1U,
        .cross_d_exponent = 1U,
        .cross_q_exponent = 0U,
        .torque_factor = MC_CONST(
            3.0 * MC_FLUX_BASE_WB * MC_CURRENT_BASE_A / MC_TORQUE_BASE_NM),
        .flux_maximum_pu = MC_CONST(1.10)};
}

void MtpaIq_Init(MtpaIqState_t* state,
                 const MtpaIqParameters_t* parameters)
{
    if (state == NULL)
        return;
    *state = (MtpaIqState_t){0};
    if (parameters != NULL)
        state->parameters = *parameters;
    else
        MtpaIq_DefaultParameters(&state->parameters);
}

void MtpaIq_Reset(MtpaIqState_t* state)
{
    if (state == NULL)
        return;
    state->table_valid = false;
    for (uint32_t index = 0U; index < MTPA_IQ_TABLE_POINTS; ++index)
        state->table[index] = (MtpaIqPoint_t){0};
}

void MtpaIq_SetParameters(MtpaIqState_t* state,
                          const MtpaIqParameters_t* parameters)
{
    if (state == NULL || parameters == NULL)
        return;
    state->parameters = *parameters;
    MtpaIq_Reset(state);
}

static void mtpa_model_current(const MtpaIqParameters_t* parameters,
                               mc_real_t flux_d,
                               mc_real_t flux_q,
                               mc_real_t* id,
                               mc_real_t* iq)
{
    mc_real_t abs_d = McMath_Abs(flux_d);
    mc_real_t abs_q = McMath_Abs(flux_q);
    mc_real_t term_d = McMath_Add(
        parameters->ad0,
        McMath_Mul(parameters->add,
                   mtpa_pow_unsigned(abs_d, parameters->d_exponent)));
    mc_real_t cross_d = McMath_Mul(
        McMath_Div(parameters->adq,
                   mtpa_from_u32((uint32_t)parameters->cross_q_exponent + 2U)),
        McMath_Mul(mtpa_pow_unsigned(abs_d,
                                     parameters->cross_d_exponent),
                   mtpa_pow_unsigned(abs_q,
                                     (uint8_t)(parameters->cross_q_exponent + 2U))));
    term_d = McMath_Add(term_d, cross_d);

    mc_real_t term_q = McMath_Add(
        parameters->aq0,
        McMath_Mul(parameters->aqq,
                   mtpa_pow_unsigned(abs_q, parameters->q_exponent)));
    mc_real_t cross_q = McMath_Mul(
        McMath_Div(parameters->adq,
                   mtpa_from_u32((uint32_t)parameters->cross_d_exponent + 2U)),
        McMath_Mul(mtpa_pow_unsigned(abs_q,
                                     parameters->cross_q_exponent),
                   mtpa_pow_unsigned(abs_d,
                                     (uint8_t)(parameters->cross_d_exponent + 2U))));
    term_q = McMath_Add(term_q, cross_q);
    *id = McMath_Mul(term_d, flux_d);
    *iq = McMath_Mul(term_q, flux_q);
}

static MtpaIqEvaluation_t mtpa_evaluate(const MtpaIqParameters_t* parameters,
                                        mc_real_t flux,
                                        mc_real_t gamma_pu)
{
    mc_real_t flux_d = McMath_Mul(flux, McMath_CosPu(gamma_pu));
    mc_real_t flux_q = McMath_Mul(flux, McMath_SinPu(gamma_pu));
    MtpaIqEvaluation_t result = {0};
    mtpa_model_current(parameters, flux_d, flux_q, &result.id, &result.iq);
    result.torque = McMath_Mul(
        parameters->torque_factor,
        McMath_Sub(McMath_Mul(flux_d, result.iq),
                   McMath_Mul(flux_q, result.id)));
    result.magnitude = McMath_Sqrt(
        McMath_Add(McMath_Mul(result.id, result.id),
                   McMath_Mul(result.iq, result.iq)));
    return result;
}

static bool mtpa_find_flux(const MtpaIqParameters_t* parameters,
                           mc_real_t torque,
                           mc_real_t gamma_pu,
                           mc_real_t* flux,
                           MtpaIqEvaluation_t* evaluation)
{
    mc_real_t low = MC_ZERO;
    mc_real_t high = MC_ZERO;
    MtpaIqEvaluation_t previous = mtpa_evaluate(parameters, MC_ZERO, gamma_pu);
    bool found = false;
    for (uint32_t index = 1U; index <= MTPA_IQ_PSI_SCAN_STEPS; ++index)
    {
        mc_real_t candidate = McMath_Div(
            McMath_Mul(parameters->flux_maximum_pu,
                       mtpa_from_u32(index)),
            mtpa_from_u32(MTPA_IQ_PSI_SCAN_STEPS));
        MtpaIqEvaluation_t current = mtpa_evaluate(parameters, candidate, gamma_pu);
        if (previous.torque < torque && current.torque >= torque)
        {
            high = candidate;
            low = McMath_Sub(candidate,
                McMath_Div(parameters->flux_maximum_pu,
                           mtpa_from_u32(MTPA_IQ_PSI_SCAN_STEPS)));
            found = true;
            break;
        }
        previous = current;
    }
    if (!found)
        return false;

    for (uint32_t index = 0U; index < MTPA_IQ_BISECT_STEPS; ++index)
    {
        mc_real_t middle = McMath_Mul(McMath_Add(low, high), MC_HALF);
        MtpaIqEvaluation_t current = mtpa_evaluate(parameters, middle, gamma_pu);
        if (current.torque >= torque)
            high = middle;
        else
            low = middle;
    }
    *flux = high;
    *evaluation = mtpa_evaluate(parameters, high, gamma_pu);
    return true;
}

static bool mtpa_compute_point(const MtpaIqParameters_t* parameters,
                               mc_real_t torque,
                               MtpaIqPoint_t* point)
{
    if (point == NULL)
        return false;
    if (torque <= MC_ZERO)
    {
        *point = (MtpaIqPoint_t){
            .torque_pu = MC_ZERO,
            .id_pu = MC_CONST(1.0 / MC_CURRENT_BASE_A),
            .ld_pu = MC_CONST(
                0.185 * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB),
            .lq_pu = MC_CONST(
                0.060 * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB),
            .valid = true};
        return true;
    }

    mc_real_t left = MC_ZERO;
    mc_real_t right = MC_CONST(0.25); /* pi/2 in per-unit angle. */
    const mc_real_t golden = MC_CONST(0.6180339887498949);
    mc_real_t c = McMath_Sub(right,
        McMath_Mul(McMath_Sub(right, left), golden));
    mc_real_t d = McMath_Add(left,
        McMath_Mul(McMath_Sub(right, left), golden));

    for (uint32_t iteration = 0U; iteration < MTPA_IQ_GOLDEN_STEPS; ++iteration)
    {
        mc_real_t flux_c = MC_ZERO;
        mc_real_t flux_d = MC_ZERO;
        MtpaIqEvaluation_t eval_c = {0};
        MtpaIqEvaluation_t eval_d = {0};
        bool valid_c = mtpa_find_flux(parameters, torque, c, &flux_c, &eval_c);
        bool valid_d = mtpa_find_flux(parameters, torque, d, &flux_d, &eval_d);
        mc_real_t magnitude_c = valid_c ? eval_c.magnitude : MC_CONST(127.0);
        mc_real_t magnitude_d = valid_d ? eval_d.magnitude : MC_CONST(127.0);
        if (magnitude_c < magnitude_d)
        {
            right = d;
            d = c;
            c = McMath_Sub(right,
                McMath_Mul(McMath_Sub(right, left), golden));
        }
        else
        {
            left = c;
            c = d;
            d = McMath_Add(left,
                McMath_Mul(McMath_Sub(right, left), golden));
        }
    }

    mc_real_t gamma = McMath_Mul(McMath_Add(left, right), MC_HALF);
    mc_real_t flux = MC_ZERO;
    MtpaIqEvaluation_t evaluation = {0};
    if (!mtpa_find_flux(parameters, torque, gamma, &flux, &evaluation))
        return false;
    *point = (MtpaIqPoint_t){
        .torque_pu = torque,
        .flux_pu = flux,
        .gamma_pu = gamma,
        .id_pu = evaluation.id,
        .iq_pu = evaluation.iq,
        .ld_pu = evaluation.id != MC_ZERO
               ? McMath_Div(McMath_Mul(flux, McMath_CosPu(gamma)), evaluation.id)
               : MC_ZERO,
        .lq_pu = evaluation.iq != MC_ZERO
               ? McMath_Div(McMath_Mul(flux, McMath_SinPu(gamma)), evaluation.iq)
               : MC_ZERO,
        .valid = true};
    return true;
}

static void mtpa_sort_by_iq(MtpaIqPoint_t* table)
{
    /* Runtime lookup is keyed by |Iq|, while the optimizer generates points
     * in torque order.  The nonlinear model does not guarantee those orders
     * are identical, so keep each complete point together and sort once. */
    for (uint32_t index = 1U; index < MTPA_IQ_TABLE_POINTS; ++index)
    {
        MtpaIqPoint_t value = table[index];
        mc_real_t key = McMath_Abs(value.iq_pu);
        uint32_t position = index;
        while (position > 0U
               && McMath_Abs(table[position - 1U].iq_pu) > key)
        {
            table[position] = table[position - 1U];
            position--;
        }
        table[position] = value;
    }
}

bool MtpaIq_BuildTable(MtpaIqState_t* state,
                       mc_real_t torque_minimum_pu,
                       mc_real_t torque_maximum_pu)
{
    if (state == NULL || torque_maximum_pu < torque_minimum_pu)
        return false;
    bool all_valid = true;
    mc_real_t span = McMath_Sub(torque_maximum_pu, torque_minimum_pu);
    for (uint32_t index = 0U; index < MTPA_IQ_TABLE_POINTS; ++index)
    {
        mc_real_t ratio = McMath_Div(
            mtpa_from_u32(index),
            mtpa_from_u32(MTPA_IQ_TABLE_POINTS - 1U));
        mc_real_t torque = McMath_Add(
            torque_minimum_pu, McMath_Mul(span, ratio));
        if (!mtpa_compute_point(&state->parameters,
                                torque,
                                &state->table[index]))
        {
            state->table[index] = (MtpaIqPoint_t){.torque_pu = torque};
            all_valid = false;
        }
    }
    mtpa_sort_by_iq(state->table);
    state->table_valid = all_valid;
    return all_valid;
}

mc_real_t MtpaIq_InterpolateIdByIq(const MtpaIqState_t* state,
                                   mc_real_t iq_reference_pu)
{
    if (state == NULL || !state->table_valid)
        return MC_ZERO;
    mc_real_t iq = McMath_Abs(iq_reference_pu);
    for (uint32_t index = 1U; index < MTPA_IQ_TABLE_POINTS; ++index)
    {
        const MtpaIqPoint_t* lower = &state->table[index - 1U];
        const MtpaIqPoint_t* upper = &state->table[index];
        if (!lower->valid || !upper->valid)
            continue;
        if (iq <= McMath_Abs(upper->iq_pu))
        {
            mc_real_t iq_low = McMath_Abs(lower->iq_pu);
            mc_real_t iq_high = McMath_Abs(upper->iq_pu);
            mc_real_t span = McMath_Sub(iq_high, iq_low);
            if (span <= MC_ZERO)
                return upper->id_pu;
            mc_real_t ratio = McMath_Div(McMath_Sub(iq, iq_low), span);
            return McMath_Add(lower->id_pu,
                McMath_Mul(ratio, McMath_Sub(upper->id_pu, lower->id_pu)));
        }
    }
    return state->table[MTPA_IQ_TABLE_POINTS - 1U].id_pu;
}
