#include <math.h>
#include <stdio.h>
#include "fixed_numeric_config.h"
#include "mtpa_iq.h"
/* Compile the actual legacy optimizer/interpolator into this host-only TU.
 * Firmware never links this float implementation into IQMATH. */
#if defined(MC_NUMERIC_IQMATH)
#define MTPA_TEST_RESTORE_IQMATH
#undef MC_NUMERIC_IQMATH /* Parse legacy math headers in their FLOAT_REF mode. */
#endif
#include "../Application/Src/MTPA.c"
#if defined(MTPA_TEST_RESTORE_IQMATH)
#define MC_NUMERIC_IQMATH 1
#endif

float32_t arm_cos_f32(float32_t value) { return cosf(value); }
float32_t arm_sin_f32(float32_t value) { return sinf(value); }

static unsigned failures;

static void check(const char* name, bool value)
{
    if (!value) { fprintf(stderr, "FAIL %s\n", name); ++failures; }
}
static void near(const char* name, mc_real_t value, float expected)
{
    float actual = McMath_ToFloat(value);
    if (!isfinite(actual) || fabsf(actual - expected) > 0.00003F)
    {
        fprintf(stderr, "FAIL %s: IQ=%g old=%g\n", name, actual, expected);
        ++failures;
    }
}
static void make_table(MtpaIqState_t* state)
{
    MtpaIq_Init(state, NULL);
    for (unsigned i = 0U; i < MTPA_IQ_TABLE_POINTS; ++i)
        state->table[i] = (MtpaIqPoint_t){
            .iq_pu = McMath_FromFloat((float)i / 64.0F),
            .id_pu = McMath_FromFloat(0.125F + (float)i / 256.0F),
            .ld_pu = McMath_FromFloat(6.0F - (float)i / 32.0F),
            .lq_pu = McMath_FromFloat(2.0F + (float)i / 128.0F),
            .valid = true};
    state->table_valid = true;
}
static float old_interpolation(const MtpaIqState_t* state, float reference,
                               unsigned field)
{
    /* Exercise the actual old function, with non-NULL Iq_out because the old
     * FOC NULL call is a real legacy bug and must not be copied to firmware. */
    float iq = fabsf(reference) * MC_CURRENT_BASE_A;
    MTPA_Point old_table[MTPA_IQ_TABLE_POINTS];
    for (unsigned i = 0U; i < MTPA_IQ_TABLE_POINTS; ++i)
    {
        old_table[i] = (MTPA_Point){
            .Iq = McMath_ToFloat(state->table[i].iq_pu) * MC_CURRENT_BASE_A,
            .Id = McMath_ToFloat(state->table[i].id_pu) * MC_CURRENT_BASE_A,
            .Ld = McMath_ToFloat(state->table[i].ld_pu)
                * MC_FLUX_BASE_WB / MC_CURRENT_BASE_A,
            .Lq = McMath_ToFloat(state->table[i].lq_pu)
                * MC_FLUX_BASE_WB / MC_CURRENT_BASE_A,
            .valid = true};
    }
    float id = 0.0F, iq_out = 0.0F;
    MTPA_interp_by_Iq(old_table, MTPA_IQ_TABLE_POINTS, iq, &id, &iq_out);
    Park_t inductance = Mtpa_Get_LPark();
    return field == 0U ? id / MC_CURRENT_BASE_A
         : field == 1U ? inductance.d * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB
                       : inductance.q * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB;
}
static void test_old_table_parity(void)
{
    MtpaIqState_t state;
    make_table(&state);
    const float references[] = {0.0F, 0.0078125F, 0.015625F, 0.25F,
        0.7578125F, 0.78125F, -0.0078125F, -0.3125F, -0.78125F};
    for (unsigned i = 0U; i < sizeof(references) / sizeof(references[0]); ++i)
    {
        MtpaIqLookupResult_t result = MtpaIq_InterpolateByIq(
            &state, McMath_FromFloat(references[i]));
        check("valid joint lookup", result.valid);
        near("old in-range Id interpolation", result.id_pu,
             old_interpolation(&state, references[i], 0U));
        near("old in-range Ld interpolation", result.ld_pu,
             old_interpolation(&state, references[i], 1U));
        near("old in-range Lq interpolation", result.lq_pu,
             old_interpolation(&state, references[i], 2U));
        near("existing Id wrapper", MtpaIq_InterpolateIdByIq(
            &state, McMath_FromFloat(references[i])), McMath_ToFloat(result.id_pu));
    }
    /* A nonzero lower key must not use the legacy unmatched-low extrapolation. */
    for (unsigned i = 0U; i < MTPA_IQ_TABLE_POINTS; ++i)
        state.table[i].iq_pu = McMath_Add(state.table[i].iq_pu, MC_CONST(0.125));
    MtpaIqLookupResult_t low = MtpaIq_InterpolateByIq(&state, MC_ZERO);
    check("below nonzero first key is invalid", !low.valid);
}
static void test_fail_closed(void)
{
    MtpaIqState_t state;
    make_table(&state);
    check("NULL safe result", !MtpaIq_InterpolateByIq(NULL, MC_ZERO).valid);
    state.table_valid = false;
    check("invalid whole table safe", !MtpaIq_InterpolateByIq(&state, MC_ZERO).valid);
    state.table_valid = true;
    state.table[0].valid = false;
    check("invalid selected sample safe", !MtpaIq_InterpolateByIq(&state, MC_ZERO).valid);
    make_table(&state);
    state.table[1].iq_pu = MC_ZERO;
    uint32_t count = McMath_Diagnostics.invalid_input_count;
    check("duplicate keys fail closed", !MtpaIq_InterpolateByIq(&state, MC_ZERO).valid);
    check("duplicate keys diagnosed", McMath_Diagnostics.invalid_input_count == count + 1U);
    check("duplicate keys never divide by zero", McMath_Diagnostics.divide_by_zero_count == 0U);
}
static void test_range_boundaries(void)
{
    MtpaIqState_t state;
    make_table(&state);
    const float references[] = {0.8125F, 1.5F, -0.8125F, -1.5F};
    for (unsigned i = 0U; i < sizeof(references) / sizeof(references[0]); ++i)
        check("out of range rejects both directions", !MtpaIq_InterpolateByIq(
            &state, McMath_FromFloat(references[i])).valid);
#if defined(MC_NUMERIC_IQMATH)
    mc_real_t maximum = state.table[MTPA_IQ_TABLE_POINTS - 1U].iq_pu;
    for (int direction = -1; direction <= 1; direction += 2)
    {
        check("endpoint valid", MtpaIq_InterpolateByIq(&state, direction * maximum).valid);
        check("one LSB inside valid", MtpaIq_InterpolateByIq(&state,
            direction * (maximum - 1)).valid);
        check("one LSB beyond invalid", !MtpaIq_InterpolateByIq(&state,
            direction * (maximum + 1)).valid);
    }
    uint32_t count = McMath_Diagnostics.saturation_count;
    MtpaIqLookupResult_t result = MtpaIq_InterpolateByIq(&state, (mc_real_t)INT32_MAX);
    check("extreme reference does not extrapolate", !result.valid);
    check("INT_MIN magnitude rejected", !MtpaIq_InterpolateByIq(&state, INT32_MIN).valid);
    check("rejected lookup does not saturate", McMath_Diagnostics.saturation_count == count);
#endif
    make_table(&state);
    state.table[1].ld_pu = MC_ZERO;
    check("zero Ld rejects lookup", !MtpaIq_InterpolateByIq(&state, MC_ZERO).valid);
    make_table(&state);
    state.table[1].lq_pu = MC_CONST(-0.1);
    check("negative Lq rejects lookup", !MtpaIq_InterpolateByIq(&state, MC_ZERO).valid);
}

static void test_local_build_validation(void)
{
    MtpaIqState_t state;
    MtpaIqParameters_t parameters;
    MtpaIq_DefaultParameters(&parameters);
    MtpaIq_Init(&state, &parameters);
    /* Global diagnostic history belongs to all algorithms. A valid local
     * build must not fail merely because another context had a math fault. */
    McMath_Diagnostics.saturation_count = 1234U;
    McMath_Diagnostics.divide_by_zero_count = 5678U;
    McMath_Diagnostics.invalid_input_count = 9012U;
    check("unrelated diagnostics do not invalidate default model",
        MtpaIq_BuildTable(&state, MC_ZERO, MC_ONE)
        && state.build_errors == MTPA_IQ_BUILD_OK);
    check("successful build leaves other diagnostic history intact",
        McMath_Diagnostics.saturation_count == 1234U
        && McMath_Diagnostics.divide_by_zero_count == 5678U
        && McMath_Diagnostics.invalid_input_count == 9012U);
    parameters.ad0 = parameters.aq0 = MC_CONST(100.0);
    parameters.add = parameters.aqq = parameters.adq = MC_ZERO;
    MtpaIq_SetParameters(&state, &parameters);
    check("saturating model cannot publish valid table",
        !MtpaIq_BuildTable(&state, MC_ZERO, MC_ONE) && !state.table_valid
        && (state.build_errors & MTPA_IQ_BUILD_SATURATION) != 0U);
    check("failed model cannot be interpolated",
        !MtpaIq_InterpolateByIq(&state, MC_CONST(0.1)).valid);
    parameters.ad0 = parameters.aq0 = MC_ONE;
    MtpaIq_SetParameters(&state, &parameters);
    check("unsolvable model rejected without numerical saturation",
        !MtpaIq_BuildTable(&state, MC_ZERO, MC_ONE)
        && (state.build_errors & MTPA_IQ_BUILD_NO_SOLUTION) != 0U
        && (state.build_errors & MTPA_IQ_BUILD_SATURATION) == 0U);
    parameters.cross_q_exponent = 254U;
    MtpaIq_SetParameters(&state, &parameters);
    uint32_t divide_count = McMath_Diagnostics.divide_by_zero_count;
    check("wrapped exponent rejected before denominator/power conversion",
        !MtpaIq_BuildTable(&state, MC_ZERO, MC_ONE)
        && (state.build_errors & MTPA_IQ_BUILD_INVALID_INPUT) != 0U
        && McMath_Diagnostics.divide_by_zero_count == divide_count);
    MtpaIq_DefaultParameters(&parameters);
    MtpaIq_SetParameters(&state, &parameters);
    check("valid replacement recovers table and clears local build status",
        MtpaIq_BuildTable(&state, MC_ZERO, MC_ONE) && state.table_valid
        && state.build_errors == MTPA_IQ_BUILD_OK);
    check("invalid rebuild range cannot leave previous valid flag",
        !MtpaIq_BuildTable(&state, MC_ONE, MC_ZERO) && !state.table_valid);
    McMath_ResetDiagnostics();
}

static void test_actual_old_optimizer(void)
{
    MtpaIqState_t state;
    MtpaIq_Init(&state, NULL);
    check("IQ default optimizer valid", MtpaIq_BuildTable(&state, MC_ZERO, MC_ONE));
    MTPA_Point old_table[MTPA_IQ_TABLE_POINTS];
    MTPA_build_table(old_table, MTPA_IQ_TABLE_POINTS, 0.0F, MC_TORQUE_BASE_NM);
    float max_id_error = 0.0F, max_iq_error = 0.0F;
    float max_ld_error = 0.0F, max_lq_error = 0.0F;
    float max_ld_relative = 0.0F, max_lq_relative = 0.0F;
    for (unsigned i = 0U; i < MTPA_IQ_TABLE_POINTS; ++i)
    {
        const MtpaIqPoint_t* point = &state.table[i];
        unsigned old_index = (unsigned)(
            McMath_ToFloat(point->torque_pu) * (MTPA_IQ_TABLE_POINTS - 1U) + 0.5F);
        if (old_index >= MTPA_IQ_TABLE_POINTS)
        {
            check("optimizer torque maps to old table", false);
            continue;
        }
        const MTPA_Point* old = &old_table[old_index];
        check("old and IQ point valid", point->valid && old->valid);
        float id_error = fabsf(McMath_ToFloat(point->id_pu) - old->Id / MC_CURRENT_BASE_A);
        float iq_error = fabsf(McMath_ToFloat(point->iq_pu) - old->Iq / MC_CURRENT_BASE_A);
        float ld_error = fabsf(McMath_ToFloat(point->ld_pu)
            - old->Ld * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB);
        float lq_error = fabsf(McMath_ToFloat(point->lq_pu)
            - old->Lq * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB);
        float old_ld_pu = old->Ld * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB;
        float old_lq_pu = old->Lq * MC_CURRENT_BASE_A / MC_FLUX_BASE_WB;
        float ld_relative = ld_error / fmaxf(fabsf(old_ld_pu), 1e-6F);
        float lq_relative = lq_error / fmaxf(fabsf(old_lq_pu), 1e-6F);
        if (id_error > max_id_error) max_id_error = id_error;
        if (iq_error > max_iq_error) max_iq_error = iq_error;
        if (ld_error > max_ld_error) max_ld_error = ld_error;
        if (lq_error > max_lq_error) max_lq_error = lq_error;
        if (ld_relative > max_ld_relative) max_ld_relative = ld_relative;
        if (lq_relative > max_lq_relative) max_lq_relative = lq_relative;
        check("optimizer Id within 0.2 percent current base", id_error <= 0.002F);
        check("optimizer Iq within 0.2 percent current base", iq_error <= 0.002F);
        check("optimizer Ld within 0.5 percent old value", ld_relative <= 0.005F);
        check("optimizer Lq within 0.5 percent old value", lq_relative <= 0.005F);
    }
    printf("Actual old optimizer vs IQ maxima (pu): Id=%g Iq=%g Ld=%g Lq=%g\n",
           max_id_error, max_iq_error, max_ld_error, max_lq_error);
    printf("Actual old optimizer vs IQ inductance relative maxima: Ld=%g Lq=%g\n",
           max_ld_relative, max_lq_relative);
}
int main(void)
{
    McMath_ResetDiagnostics();
    test_old_table_parity();
    test_fail_closed();
    test_range_boundaries();
    test_local_build_validation();
    test_actual_old_optimizer();
    if (failures != 0U) return 1;
    puts("PASS: MTPA in-range legacy parity, explicit range rejection and local build validity");
    return 0;
}
