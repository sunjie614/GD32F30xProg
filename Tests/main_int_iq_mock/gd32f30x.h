#ifndef MAIN_INT_IQ_HOST_GD32F30X_H
#define MAIN_INT_IQ_HOST_GD32F30X_H

#include <stdint.h>

typedef struct { uint32_t CYCCNT; } MainIntTestDwt_t;
extern MainIntTestDwt_t MainIntTestDwt;
extern uint32_t MainIntTestTimerCchp;
extern uint32_t SystemCoreClock;

#define DWT (&MainIntTestDwt)
#define TIMER0 0U
#define TIMER_CCHP(timer) (MainIntTestTimerCchp)
#define TIMER_CCHP_POEN (1U << 15)
#define ADC0 0U
#define ADC_INT_FLAG_EOIC (1U << 2)
#define ADC0_1_IRQn 18

void nvic_irq_disable(int irq);
void nvic_irq_enable(int irq, uint8_t priority, uint8_t subpriority);
void adc_interrupt_flag_clear(uint32_t adc, uint32_t flags);
void NVIC_ClearPendingIRQ(int irq);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);

#endif
