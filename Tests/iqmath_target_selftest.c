#include "fixed_control.h"
#include "mc_math.h"

/* Standalone target-link probe. On target, a zero return value means that all
   deterministic IQmath conversion/arithmetic/trigonometric checks passed. */
int main(void)
{
    FixedControl_Init();
    FixedControlSelfTest_t result = FixedControl_RunSelfTest();
    uint32_t failures = result.conversion_failures
                      + result.arithmetic_failures
                      + result.trigonometric_failures
                      + result.sqrt_failures
                      + McMath_Diagnostics.divide_by_zero_count
                      + McMath_Diagnostics.invalid_input_count;
    return failures == 0U ? 0 : 1;
}
