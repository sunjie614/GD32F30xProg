#include "main_int.h"
#include "fixed_control.h"

volatile DeviceStateEnum_t MainInt_State = RUNNING;

bool MainInt_ModeRequestsStop(void)
{
    return MainInt_State == INIT || MainInt_State == EXIT
        || (unsigned)MainInt_State > EXIT;
}

void MainInt_PrepareMode(void)
{
    if ((unsigned)MainInt_State > EXIT)
        MainInt_State = EXIT;
    if (MainInt_State == SENSORLESS)
    {
        /* Legacy (method & FLYING) is always false because FLYING=0.
         * Preserve its actual result rather than inventing a new bit flag. */
        Foc_Mode = SPEED;
    }
    else if (MainInt_State == INIT || MainInt_State == EXIT)
        Foc_Mode = IDLE;
}
