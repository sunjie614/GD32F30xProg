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

/* A build owns its arithmetic status. Never infer candidate validity by
 * comparing global diagnostic counters: the control ISR may update those
 * counters while this background optimizer is running. Checked wrappers keep
 * the IQmath numerical implementation and track this call's failures only. */
typedef struct
{
    uint32_t errors;
} MtpaIqArithmetic_t;

static mc_real_t mtpa_check_result(MtpaIqArithmetic_t* math, mc_real_t value)
{
#if !defined(MC_NUMERIC_IQMATH)
    if (!isfinite(value))
        math->errors |= MTPA_IQ_BUILD_INVALID_INPUT;
#else
    (void)math;
#endif
    return value;
}

static mc_real_t mtpa_add(MtpaIqArithmetic_t* math, mc_real_t a, mc_real_t b)
{
#if defined(MC_NUMERIC_IQMATH)
    int64_t result = (int64_t)a + b;
    if (result > INT32_MAX || result < INT32_MIN)
        math->errors |= MTPA_IQ_BUILD_SATURATION;
#endif
    return mtpa_check_result(math, McMath_Add(a, b));
}

static mc_real_t mtpa_sub(MtpaIqArithmetic_t* math, mc_real_t a, mc_real_t b)
{
#if defined(MC_NUMERIC_IQMATH)
    int64_t result = (int64_t)a - b;
    if (result > INT32_MAX || result < INT32_MIN)
        math->errors |= MTPA_IQ_BUILD_SATURATION;
#endif
    return mtpa_check_result(math, McMath_Sub(a, b));
}

static mc_real_t mtpa_mul(MtpaIqArithmetic_t* math, mc_real_t a, mc_real_t b)
{
#if defined(MC_NUMERIC_IQMATH)
    int64_t product = (int64_t)a * b;
    int64_t magnitude = product < 0 ? -product : product;
    int64_t rounded = (magnitude + ((int64_t)1 << 23)) >> 24;
    if (product < 0)
        rounded = -rounded;
    if (rounded > INT32_MAX || rounded < INT32_MIN)
        math->errors |= MTPA_IQ_BUILD_SATURATION;
#endif
    return mtpa_check_result(math, McMath_Mul(a, b));
}

static mc_real_t mtpa_div(MtpaIqArithmetic_t* math, mc_real_t a, mc_real_t b)
{
    if (b == MC_ZERO)
        math->errors |= MTPA_IQ_BUILD_DIVIDE_BY_ZERO;
#if defined(MC_NUMERIC_IQMATH)
    else
    {
        int64_t result = ((int64_t)a * ((int64_t)1 << 24)) / b;
        if (result > INT32_MAX || result < INT32_MIN)
            math->errors |= MTPA_IQ_BUILD_SATURATION;
    }
#endif
    return mtpa_check_result(math, McMath_Div(a, b));
}

static mc_real_t mtpa_abs(MtpaIqArithmetic_t* math, mc_real_t value)
{
#if defined(MC_NUMERIC_IQMATH)
    if (value == INT32_MIN)
        math->errors |= MTPA_IQ_BUILD_SATURATION;
#endif
    return mtpa_check_result(math, McMath_Abs(value));
}

static mc_real_t mtpa_sqrt(MtpaIqArithmetic_t* math, mc_real_t value)
{
    if (value < MC_ZERO)
        math->errors |= MTPA_IQ_BUILD_INVALID_INPUT;
    return mtpa_check_result(math, McMath_Sqrt(value));
}

static mc_real_t mtpa_from_u32(MtpaIqArithmetic_t* math, uint32_t value)
{
#if defined(MC_NUMERIC_IQMATH)
    if (value > 127U)
    {
        math->errors |= MTPA_IQ_BUILD_SATURATION;
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MAX;
    }
    return (mc_real_t)((int32_t)value << 24);
#else
    (void)math;
    return (mc_real_t)value;
#endif
}

static mc_real_t mtpa_pow_unsigned(MtpaIqArithmetic_t* math,
                                   mc_real_t value, uint8_t exponent)
{
    mc_real_t result = MC_ONE;
    mc_real_t base = mtpa_abs(math, value);
    for (uint8_t index = 0U; index < exponent; ++index)
        result = mtpa_mul(math, result, base);
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
    state->build_errors = MTPA_IQ_BUILD_OK;
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

static void mtpa_model_current(MtpaIqArithmetic_t* math,
                               const MtpaIqParameters_t* parameters,
                               mc_real_t flux_d,
                               mc_real_t flux_q,
                               mc_real_t* id,
                               mc_real_t* iq)
{
    mc_real_t abs_d = mtpa_abs(math, flux_d);
    mc_real_t abs_q = mtpa_abs(math, flux_q);
    mc_real_t term_d = mtpa_add(math,
        parameters->ad0,
        mtpa_mul(math, parameters->add,
                   mtpa_pow_unsigned(math, abs_d, parameters->d_exponent)));
    mc_real_t cross_d = mtpa_mul(math,
        mtpa_div(math, parameters->adq,
                   mtpa_from_u32(math, (uint32_t)parameters->cross_q_exponent + 2U)),
        mtpa_mul(math, mtpa_pow_unsigned(math, abs_d,
                                     parameters->cross_d_exponent),
                   mtpa_pow_unsigned(math, abs_q,
                                     (uint8_t)(parameters->cross_q_exponent + 2U))));
    term_d = mtpa_add(math, term_d, cross_d);

    mc_real_t term_q = mtpa_add(math,
        parameters->aq0,
        mtpa_mul(math, parameters->aqq,
                   mtpa_pow_unsigned(math, abs_q, parameters->q_exponent)));
    mc_real_t cross_q = mtpa_mul(math,
        mtpa_div(math, parameters->adq,
                   mtpa_from_u32(math, (uint32_t)parameters->cross_d_exponent + 2U)),
        mtpa_mul(math, mtpa_pow_unsigned(math, abs_q,
                                     parameters->cross_q_exponent),
                   mtpa_pow_unsigned(math, abs_d,
                                     (uint8_t)(parameters->cross_d_exponent + 2U))));
    term_q = mtpa_add(math, term_q, cross_q);
    *id = mtpa_mul(math, term_d, flux_d);
    *iq = mtpa_mul(math, term_q, flux_q);
}

static MtpaIqEvaluation_t mtpa_evaluate(MtpaIqArithmetic_t* math,
                                        const MtpaIqParameters_t* parameters,
                                        mc_real_t flux,
                                        mc_real_t gamma_pu)
{
    mc_real_t flux_d = mtpa_mul(math, flux, McMath_CosPu(gamma_pu));
    mc_real_t flux_q = mtpa_mul(math, flux, McMath_SinPu(gamma_pu));
    MtpaIqEvaluation_t result = {0};
    mtpa_model_current(math, parameters, flux_d, flux_q, &result.id, &result.iq);
    result.torque = mtpa_mul(math,
        parameters->torque_factor,
        mtpa_sub(math, mtpa_mul(math, flux_d, result.iq),
                   mtpa_mul(math, flux_q, result.id)));
    result.magnitude = mtpa_sqrt(math,
        mtpa_add(math, mtpa_mul(math, result.id, result.id),
                   mtpa_mul(math, result.iq, result.iq)));
    return result;
}

static bool mtpa_find_flux(MtpaIqArithmetic_t* math,
                           const MtpaIqParameters_t* parameters,
                           mc_real_t torque,
                           mc_real_t gamma_pu,
                           mc_real_t* flux,
                           MtpaIqEvaluation_t* evaluation)
{
    mc_real_t low = MC_ZERO;
    mc_real_t high = MC_ZERO;
    MtpaIqEvaluation_t previous = mtpa_evaluate(math, parameters, MC_ZERO, gamma_pu);
    bool found = false;
    for (uint32_t index = 1U; index <= MTPA_IQ_PSI_SCAN_STEPS; ++index)
    {
        mc_real_t candidate = mtpa_div(math,
            mtpa_mul(math, parameters->flux_maximum_pu,
                       mtpa_from_u32(math, index)),
            mtpa_from_u32(math, MTPA_IQ_PSI_SCAN_STEPS));
        MtpaIqEvaluation_t current = mtpa_evaluate(math, parameters, candidate, gamma_pu);
        if (math->errors != MTPA_IQ_BUILD_OK)
            return false;
        if (previous.torque < torque && current.torque >= torque)
        {
            high = candidate;
            low = mtpa_sub(math, candidate,
                mtpa_div(math, parameters->flux_maximum_pu,
                           mtpa_from_u32(math, MTPA_IQ_PSI_SCAN_STEPS)));
            found = true;
            break;
        }
        previous = current;
    }
    if (!found)
        return false;

    for (uint32_t index = 0U; index < MTPA_IQ_BISECT_STEPS; ++index)
    {
        mc_real_t middle = mtpa_mul(math, mtpa_add(math, low, high), MC_HALF);
        MtpaIqEvaluation_t current = mtpa_evaluate(math, parameters, middle, gamma_pu);
        if (math->errors != MTPA_IQ_BUILD_OK)
            return false;
        if (current.torque >= torque)
            high = middle;
        else
            low = middle;
    }
    *flux = high;
    *evaluation = mtpa_evaluate(math, parameters, high, gamma_pu);
    return math->errors == MTPA_IQ_BUILD_OK;
}

static bool mtpa_compute_point(MtpaIqArithmetic_t* math,
                               const MtpaIqParameters_t* parameters,
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
    mc_real_t c = mtpa_sub(math, right,
        mtpa_mul(math, mtpa_sub(math, right, left), golden));
    mc_real_t d = mtpa_add(math, left,
        mtpa_mul(math, mtpa_sub(math, right, left), golden));

    for (uint32_t iteration = 0U; iteration < MTPA_IQ_GOLDEN_STEPS; ++iteration)
    {
        mc_real_t flux_c = MC_ZERO;
        mc_real_t flux_d = MC_ZERO;
        MtpaIqEvaluation_t eval_c = {0};
        MtpaIqEvaluation_t eval_d = {0};
        bool valid_c = mtpa_find_flux(math, parameters, torque, c, &flux_c, &eval_c);
        bool valid_d = mtpa_find_flux(math, parameters, torque, d, &flux_d, &eval_d);
        if (math->errors != MTPA_IQ_BUILD_OK)
            return false;
        mc_real_t magnitude_c = valid_c ? eval_c.magnitude : MC_CONST(127.0);
        mc_real_t magnitude_d = valid_d ? eval_d.magnitude : MC_CONST(127.0);
        if (magnitude_c < magnitude_d)
        {
            right = d;
            d = c;
            c = mtpa_sub(math, right,
                mtpa_mul(math, mtpa_sub(math, right, left), golden));
        }
        else
        {
            left = c;
            c = d;
            d = mtpa_add(math, left,
                mtpa_mul(math, mtpa_sub(math, right, left), golden));
        }
    }

    mc_real_t gamma = mtpa_mul(math, mtpa_add(math, left, right), MC_HALF);
    mc_real_t flux = MC_ZERO;
    MtpaIqEvaluation_t evaluation = {0};
    if (!mtpa_find_flux(math, parameters, torque, gamma, &flux, &evaluation))
    {
        math->errors |= MTPA_IQ_BUILD_NO_SOLUTION;
        return false;
    }
    if (evaluation.id <= MC_ZERO || evaluation.iq <= MC_ZERO)
    {
        math->errors |= MTPA_IQ_BUILD_INVALID_POINT;
        return false;
    }
    *point = (MtpaIqPoint_t){
        .torque_pu = torque,
        .flux_pu = flux,
        .gamma_pu = gamma,
        .id_pu = evaluation.id,
        .iq_pu = evaluation.iq,
        .ld_pu = evaluation.id != MC_ZERO
               ? mtpa_div(math, mtpa_mul(math, flux, McMath_CosPu(gamma)), evaluation.id)
               : MC_ZERO,
        .lq_pu = evaluation.iq != MC_ZERO
               ? mtpa_div(math, mtpa_mul(math, flux, McMath_SinPu(gamma)), evaluation.iq)
               : MC_ZERO,
        .valid = true};
    if (point->ld_pu <= MC_ZERO || point->lq_pu <= MC_ZERO)
        math->errors |= MTPA_IQ_BUILD_INVALID_POINT;
    point->valid = math->errors == MTPA_IQ_BUILD_OK;
    return point->valid;
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
    if (state == NULL)
        return false;
    MtpaIq_Reset(state);
    MtpaIqArithmetic_t checked = {MTPA_IQ_BUILD_OK};
    MtpaIqArithmetic_t* math = &checked;
    const MtpaIqParameters_t* p = &state->parameters;
    if (torque_minimum_pu < MC_ZERO || torque_maximum_pu <= torque_minimum_pu
        || p->ad0 <= MC_ZERO || p->aq0 <= MC_ZERO
        || p->torque_factor <= MC_ZERO || p->flux_maximum_pu <= MC_ZERO
        || p->cross_d_exponent > 125U || p->cross_q_exponent > 125U)
        math->errors |= MTPA_IQ_BUILD_INVALID_INPUT;
#if !defined(MC_NUMERIC_IQMATH)
    if (!isfinite(torque_minimum_pu) || !isfinite(torque_maximum_pu)
        || !isfinite(p->ad0) || !isfinite(p->add) || !isfinite(p->aq0)
        || !isfinite(p->aqq) || !isfinite(p->adq)
        || !isfinite(p->torque_factor) || !isfinite(p->flux_maximum_pu))
        math->errors |= MTPA_IQ_BUILD_INVALID_INPUT;
#endif
    mc_real_t span = mtpa_sub(math, torque_maximum_pu, torque_minimum_pu);
    for (uint32_t index = 0U; index < MTPA_IQ_TABLE_POINTS
         && math->errors == MTPA_IQ_BUILD_OK; ++index)
    {
        mc_real_t ratio = mtpa_div(math,
            mtpa_from_u32(math, index),
            mtpa_from_u32(math, MTPA_IQ_TABLE_POINTS - 1U));
        mc_real_t torque = mtpa_add(math,
            torque_minimum_pu, mtpa_mul(math, span, ratio));
        if (!mtpa_compute_point(math, p, torque, &state->table[index]))
        {
            state->table[index] = (MtpaIqPoint_t){.torque_pu = torque};
            if (math->errors == MTPA_IQ_BUILD_OK)
                math->errors |= MTPA_IQ_BUILD_NO_SOLUTION;
        }
    }
    if (math->errors == MTPA_IQ_BUILD_OK)
    {
        mtpa_sort_by_iq(state->table);
        for (uint32_t index = 1U; index < MTPA_IQ_TABLE_POINTS; ++index)
        {
            if (state->table[index].iq_pu <= state->table[index - 1U].iq_pu)
                math->errors |= MTPA_IQ_BUILD_INVALID_POINT;
        }
    }
    state->build_errors = math->errors;
    state->table_valid = math->errors == MTPA_IQ_BUILD_OK;
    return state->table_valid;
}

static mc_real_t mtpa_interpolate_value(mc_real_t lower, mc_real_t upper,
                                        mc_real_t iq, mc_real_t iq_low,
                                        mc_real_t span)
{
#if defined(MC_NUMERIC_IQMATH)
    /* Both |Iq| keys are nonnegative int32, so their difference is at most
     * INT32_MAX. The full value difference is at most UINT32_MAX; their
     * signed product and rounding margin therefore fit in int64. Keep that
     * precision until division instead of saturating a Q24 ratio first. */
    int64_t product = ((int64_t)iq - iq_low) * ((int64_t)upper - lower);
    int64_t magnitude = product < 0 ? -product : product;
    int64_t term = (magnitude + (int64_t)span / 2) / span;
    if (product < 0)
        term = -term;
    if (term > (int64_t)INT32_MAX - lower)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MAX;
    }
    if (term < (int64_t)INT32_MIN - lower)
    {
        McMath_Diagnostics.saturation_count++;
        return (mc_real_t)INT32_MIN;
    }
    return (mc_real_t)((int64_t)lower + term);
#else
    return lower + ((iq - iq_low) / span) * (upper - lower);
#endif
}

MtpaIqLookupResult_t MtpaIq_InterpolateByIq(
    const MtpaIqState_t* state, mc_real_t iq_reference_pu)
{
    MtpaIqLookupResult_t result = {0};
    if (state == NULL || !state->table_valid)
        return result;
#if !defined(MC_NUMERIC_IQMATH)
    if (!isfinite(iq_reference_pu))
    {
        McMath_Diagnostics.invalid_input_count++;
        return result;
    }
#endif
#if defined(MC_NUMERIC_IQMATH)
    if (iq_reference_pu == INT32_MIN)
        return result; /* Its positive magnitude is outside Q24. */
#endif
    mc_real_t iq = McMath_Abs(iq_reference_pu);
    /* Deliberate departure from the legacy final-interval extrapolation:
     * outside a valid interval the FOC caller must use the linear fallback. */
    uint32_t selected = MTPA_IQ_TABLE_POINTS;
    for (uint32_t index = 1U; index < MTPA_IQ_TABLE_POINTS; ++index)
    {
        const MtpaIqPoint_t* lower = &state->table[index - 1U];
        const MtpaIqPoint_t* upper = &state->table[index];
        if (iq >= McMath_Abs(lower->iq_pu)
            && iq <= McMath_Abs(upper->iq_pu))
        {
            selected = index - 1U;
            break;
        }
    }
    if (selected == MTPA_IQ_TABLE_POINTS)
        return result;
    const MtpaIqPoint_t* lower = &state->table[selected];
    const MtpaIqPoint_t* upper = &state->table[selected + 1U];
    mc_real_t iq_low = McMath_Abs(lower->iq_pu);
    mc_real_t iq_high = McMath_Abs(upper->iq_pu);
    mc_real_t span = McMath_Sub(iq_high, iq_low);
    if (!lower->valid || !upper->valid || span <= MC_ZERO
        || lower->iq_pu < MC_ZERO || upper->iq_pu < MC_ZERO
        || lower->id_pu < MC_ZERO || upper->id_pu < MC_ZERO
        || lower->ld_pu <= MC_ZERO || upper->ld_pu <= MC_ZERO
        || lower->lq_pu <= MC_ZERO || upper->lq_pu <= MC_ZERO)
    {
        /* Unlike the old 1e-9 denominator fallback, malformed tables do not
         * silently produce enormous references or dereference NULL outputs. */
        McMath_Diagnostics.invalid_input_count++;
        return result;
    }
#if !defined(MC_NUMERIC_IQMATH)
    if (!isfinite(span) || !isfinite(lower->id_pu) || !isfinite(upper->id_pu)
        || !isfinite(lower->ld_pu) || !isfinite(upper->ld_pu)
        || !isfinite(lower->lq_pu) || !isfinite(upper->lq_pu))
    {
        McMath_Diagnostics.invalid_input_count++;
        return result;
    }
#endif
    result.id_pu = mtpa_interpolate_value(
        lower->id_pu, upper->id_pu, iq, iq_low, span);
    result.ld_pu = mtpa_interpolate_value(
        lower->ld_pu, upper->ld_pu, iq, iq_low, span);
    result.lq_pu = mtpa_interpolate_value(
        lower->lq_pu, upper->lq_pu, iq, iq_low, span);
    result.valid = true;
    return result;
}

mc_real_t MtpaIq_InterpolateIdByIq(const MtpaIqState_t* state,
                                   mc_real_t iq_reference_pu)
{
    MtpaIqLookupResult_t result = MtpaIq_InterpolateByIq(
        state, iq_reference_pu);
    return result.valid ? result.id_pu : MC_ZERO;
}
