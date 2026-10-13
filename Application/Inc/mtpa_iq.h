#ifndef MTPA_IQ_H
#define MTPA_IQ_H

#include <stdbool.h>
#include <stdint.h>
#include "mc_math.h"

#define MTPA_IQ_TABLE_POINTS 51U

enum
{
    MTPA_IQ_BUILD_OK = 0U,
    MTPA_IQ_BUILD_INVALID_INPUT = 1U << 0,
    MTPA_IQ_BUILD_SATURATION = 1U << 1,
    MTPA_IQ_BUILD_DIVIDE_BY_ZERO = 1U << 2,
    MTPA_IQ_BUILD_NO_SOLUTION = 1U << 3,
    MTPA_IQ_BUILD_INVALID_POINT = 1U << 4
};

typedef struct
{
    /* Normalized model coefficients.  Flux, current and torque use the bases
     * declared in fixed_numeric_config.h. */
    mc_real_t ad0;
    mc_real_t add;
    mc_real_t aq0;
    mc_real_t aqq;
    mc_real_t adq;
    uint8_t d_exponent;
    uint8_t q_exponent;
    uint8_t cross_d_exponent;
    uint8_t cross_q_exponent;
    mc_real_t torque_factor;
    mc_real_t flux_maximum_pu;
} MtpaIqParameters_t;

typedef struct
{
    mc_real_t torque_pu;
    mc_real_t flux_pu;
    mc_real_t gamma_pu;
    mc_real_t id_pu;
    mc_real_t iq_pu;
    mc_real_t ld_pu;
    mc_real_t lq_pu;
    bool valid;
} MtpaIqPoint_t;

typedef struct
{
    MtpaIqParameters_t parameters;
    MtpaIqPoint_t table[MTPA_IQ_TABLE_POINTS];
    bool table_valid;
    uint32_t build_errors; /* Local build status; unrelated ISR math cannot taint it. */
} MtpaIqState_t;

typedef struct
{
    mc_real_t id_pu;
    mc_real_t ld_pu;
    mc_real_t lq_pu;
    bool valid;
} MtpaIqLookupResult_t;

void MtpaIq_DefaultParameters(MtpaIqParameters_t* parameters);
void MtpaIq_Init(MtpaIqState_t* state,
                 const MtpaIqParameters_t* parameters);
void MtpaIq_Reset(MtpaIqState_t* state);
void MtpaIq_SetParameters(MtpaIqState_t* state,
                          const MtpaIqParameters_t* parameters);
bool MtpaIq_BuildTable(MtpaIqState_t* state,
                       mc_real_t torque_minimum_pu,
                       mc_real_t torque_maximum_pu);
mc_real_t MtpaIq_InterpolateIdByIq(const MtpaIqState_t* state,
                                   mc_real_t iq_reference_pu);
/* One joint Id/Ld/Lq result. Only an actual valid table interval is usable;
 * out-of-range |Iq| is invalid and the caller selects its explicit fallback. */
MtpaIqLookupResult_t MtpaIq_InterpolateByIq(
    const MtpaIqState_t* state, mc_real_t iq_reference_pu);

#endif
