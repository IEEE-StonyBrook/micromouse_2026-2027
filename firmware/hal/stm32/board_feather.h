#ifndef HAL_STM32_BOARD_FEATHER_H
#define HAL_STM32_BOARD_FEATHER_H

// Adafruit Feather STM32F405 Express, the P1B bring-up board. Pins follow
// Adafruit's published pinout; the rev1 mouse board gets its own file.

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

namespace board
{
// Red user LED on Feather pin D13, active high.
constexpr uint32_t         LED_PORT = GPIOC;
constexpr uint16_t         LED_PIN  = GPIO1;
constexpr rcc_periph_clken LED_RCC  = RCC_GPIOC;
} // namespace board

#endif // HAL_STM32_BOARD_FEATHER_H
