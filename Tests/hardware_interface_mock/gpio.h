#ifndef HARDWARE_TEST_GPIO_H
#define HARDWARE_TEST_GPIO_H
#include <stdint.h>
typedef enum { RESET = 0, SET = 1 } FlagStatus;
#define GPIOE 4U
#define GPIOD 3U
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_15 (1U << 15)
#define FAN_OPEN_PORT GPIOD
#define FAN_OPEN_PIN GPIO_PIN_8
FlagStatus gpio_input_bit_get(uint32_t port, uint32_t pin);
void gpio_bit_set(uint32_t port, uint32_t pin);
void gpio_bit_reset(uint32_t port, uint32_t pin);
#endif
