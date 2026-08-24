#include "main_int.h"

#include "buffer.h"
#include "fixed_control.h"
#include "hardware_interface.h"
#include "reciprocal.h"

void Main_Int_Handler(void)
{
    Phase_t current = Peripheral_Get_PhaseCurrent();
    AngleResult_t position = Peripheral_Update_Position();
    FloatWithInv_t bus = Peripheral_UpdateUdc();
    bool stop = Peripheral_Update_Break();

    FixedControlInput_t input = {
        .current_abc = current,
        .theta_rad = position.theta,
        .speed_rpm = position.speed,
        .bus_voltage = bus.val,
        .reset = stop};
    FixedControlOutput_t output = FixedControl_Step(&input);

    Peripheral_Set_Stop(stop);
    Peripheral_Set_PWMChangePoint(output.pwm_duty);

    /* Float telemetry is deliberately isolated at the CAN/A2L boundary. */
    Buffer_Put(position.theta, 0U);
    Buffer_Put(output.estimated_theta_rad, 1U);
    Buffer_Put(position.speed, 2U);
    Buffer_Put(output.estimated_speed_rpm, 3U);
    Buffer_Put(Foc_Id_Fdbk, 16U);
    Buffer_Put(Foc_Iq_Fdbk, 17U);
    Buffer_Send();
}
