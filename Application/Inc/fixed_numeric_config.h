#ifndef FIXED_NUMERIC_CONFIG_H
#define FIXED_NUMERIC_CONFIG_H

/* Project numeric contract.  These are application-selected physical bases,
 * not constants supplied by IQmath.  Runtime algorithm state is normalized
 * against these values and stored as signed Q24. */
#define MC_CURRENT_BASE_A       (30.0F)
#define MC_VOLTAGE_BASE_V       (800.0F)
#define MC_SPEED_BASE_RPM       (1800.0F)
#define MC_TORQUE_BASE_NM       (50.0F)
#define MC_FLUX_BASE_WB         (1.0F)
#define MC_TEMPERATURE_BASE_C   (200.0F)
#define MC_TWO_PI_F             (6.28318530717958647692F)
#define MC_INV_TWO_PI_F         (0.15915494309189533577F)

#endif
