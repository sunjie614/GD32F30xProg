#ifndef HARDWARE_TEST_TIM_H
#define HARDWARE_TEST_TIM_H
#include <stdint.h>
#define TIMER0 0U
#define ENABLE 1U
#define DISABLE 0U
#define TIMER_SWEVG_BRKG (1U << 7)
#define TIMER_INT_BRK (1U << 7)
extern volatile uint32_t HardwareTest_Swevg;
#define TIMER_SWEVG(timer) HardwareTest_Swevg
#if defined(MC_NUMERIC_IQMATH)
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
#endif
void timer_primary_output_config(uint32_t timer, unsigned enabled);
void timer_interrupt_enable(uint32_t timer, uint32_t interrupt);
void timer_interrupt_disable(uint32_t timer, uint32_t interrupt);
void Set_PWM_Compare(float a, float b, float c, float brake);
#endif
