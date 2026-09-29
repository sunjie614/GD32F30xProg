#include "main.h"  // IWYU pragma: export
#include "Initialization.h"
#include "com.h"
#include "hardware_interface.h"
#include "main_int.h"
#if defined(MC_NUMERIC_IQMATH)
#include "fixed_control.h"
#endif

bool pin = false;

/*!
    \brief      main function
*/
int main(void)
{
#if defined(MC_NUMERIC_IQMATH)
    /* Initialize all fixed-point state before enabling the ADC interrupt. */
    if (!Initialization_MTPA() || !Initialization_Drivers())
    {
        /* No ADC ISR, CAN control, or PWM arming after failed startup. */
        Peripheral_Set_Stop(true);
        while (1) { }
    }
#else
    Initialization_Drivers();
    Initialization_MTPA();
#endif
    while (1)
    {
#if defined(MC_NUMERIC_IQMATH)
        Initialization_ArmHardwareProtectIfReady();
        MainInt_ControlBackground();
#endif
        COM_CANProtocol();
        COM_SCIProtocol();
        // COM_DAQProtocol(systick_ms); Use CCP DAQ may cause PiSnoop display
        // offline
        Peripheral_Update_Temperature();
#if defined(MC_NUMERIC_IQMATH)
        if (FixedControl_ModeRequestsStop())
            Peripheral_Set_Stop(true);
#endif
        Peripheral_Update_Break();

        pin = Peripheral_Get_HardwareBrk();
        // DWT_Count = DWT->CYCCNT; // 读取DWT计数器
    }
}
