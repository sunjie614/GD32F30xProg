#include "Initialization.h"

#include "adc.h"
#include "buffer.h"
#include "can.h"
#include "com.h"
#include "fixed_control.h"
#include "gd32f30x.h"
#include "gd32f30x_exti.h"
#include "gd32f30x_gpio.h"
#include "gpio.h"
#include "motor.h"
#include "parameters.h"
#include "position_sensor.h"
#include "protect.h"
#include "systick.h"
#include "tim.h"
#include "usart.h"

volatile uint32_t DWT_Count = 0U;

static const SystemTimeConfig_t Fixed_SystemTime = {
    .current = {.val = MAIN_LOOP_TIME, .inv = MAIN_LOOP_FREQ},
    .speed = {.val = SPEED_LOOP_TIME, .inv = SPEED_LOOP_FREQ},
    .prescaler = SPEED_LOOP_PRESCALER};

static const MotorParam_t Fixed_MotorParameters = {
    .Rs = MOTOR_RS,
    .Ld = MOTOR_LD,
    .Lq = MOTOR_LQ,
    .Flux = MOTOR_FLUX,
    .Pn = MOTOR_PN,
    .Resolver_Pn = MOTOR_RESOLVER_PN,
    .inv_MotorPn = 1.0F / MOTOR_PN,
    .Position_Offset = MOTOR_POSITION_OFFSET,
    .Position_Scale = MOTOR_POSITION_SCALE,
    .theta_factor = MOTOR_THETA_FACTOR};

bool Initialization_Modules(void)
{
    Protect_Parameter_t protection = {
        .Udc_rate = PROTECT_VOLTAGE_RATE,
        .Udc_fluctuation = PROTECT_VOLTAGE_FLUCTUATION,
        .I_Max = PROTECT_CURRENT_MAX,
        .Temperature = PROTECT_TEMPERATURE,
        .Flag = No_Protect};
    bool result = Protect_Initialization(&protection);
    result = Motor_Initialization(&Fixed_MotorParameters) && result;
    result = Motor_Set_SampleTime(&Fixed_SystemTime) && result;
    result = Motor_Set_SpeedPrescaler((uint16_t)SPEED_LOOP_PRESCALER) && result;
    result = Motor_Set_Filter(10.0F, SPEED_LOOP_FREQ) && result;
    Buffer_Init(BUFFER_CAPACITY, BUFFER_PRESCALER);
    FixedControl_Init();
    return result;
}

static void fixed_init_dwt(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void fixed_init_nvic(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(TIMER0_BRK_IRQn, 0U, 0U);
    nvic_irq_enable(EXTI4_IRQn, 1U, 0U);
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 2U, 0U);
    nvic_irq_enable(ADC0_1_IRQn, 3U, 0U);
    nvic_irq_enable(TIMER3_IRQn, 4U, 0U);
    nvic_irq_enable(DMA0_Channel3_IRQn, 5U, 0U);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
}

static void fixed_init_exti(void)
{
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOE, GPIO_PIN_SOURCE_4);
    exti_init(EXTI_4, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_4);
}

bool Initialization_MTPA(void)
{
    /* IQMATH MTPA/identification state is part of the fixed control context. */
    return Initialization_Modules();
}

bool Initialization_Drivers(void)
{
    systick_config();
    TIM1_Init();
    fixed_init_dwt();
    USART_Init(&husart0);
    USART_DMA_Init();
    GPIO_Init();
    Position_Sensor_Init();
    TIM0_PWM_Init(MAIN_INT_TIMER_PRESCALER,
                  MAIN_INT_TIMER_PERIOD,
                  MAIN_INT_TIMER_DEADTIME_PERIOD);
    Adc_Initialization();
    Can_Initialization();
    gpio_bit_set(SOFT_OPEN_PORT, SOFT_OPEN_PIN);
    fixed_init_exti();
    fixed_init_nvic();
    Com_Initialization();
    return true;
}
