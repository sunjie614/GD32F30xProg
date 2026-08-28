#ifndef FOC_MODE_H
#define FOC_MODE_H

/* Keep the original names and numeric values.  The enum type is intentionally
 * public so the ELF debug information can regenerate the FocMode_t A2L value
 * table (IDLE/VF_MODE/IF_MODE/SPEED/STARTUP/IDENTIFY). */
typedef enum
{
    IDLE = 0,
    VF_MODE,
    IF_MODE,
    SPEED,
    STARTUP,
    IDENTIFY
} FocMode_t;

#endif
