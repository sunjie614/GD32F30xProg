#ifndef _ADC_H
#define _ADC_H

#include <stdbool.h>

// Udc, inverse Udc, Adc_Temperature is readed in main loop
extern float Adc_Temperature;

void Adc_Init_GPIO(void);
void Adc_Init_DMA(void);
void Adc_Initialization(void);
void Adc_Calibrate_CurrentOffset(void);
#if defined(MC_NUMERIC_IQMATH)
bool Adc_Calibrate_CurrentOffsetChecked(void);
#endif

float Adc_Get_Temperature(void);
float Adc_Get_VoltageBus(void);
float Adc_Get_VoltageBusInv(void);
void  Adc_Get_ThreePhaseCurrent(float* Ia, float* Ib, float* Ic);

#endif
