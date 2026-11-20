#ifndef HAL_STM32_GPIO_H
#define HAL_STM32_GPIO_H

// Maps the plain board::Pin values in config/board_feather.h onto libopencm3.

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "config/board_feather.h"

namespace hal::stm32
{
/// GPIO port base address. Ports sit 0x400 apart from GPIOA (RM0090 memory map).
inline uint32_t gpioPort(board::Pin pin)
{
    return GPIOA + 0x400u * static_cast<uint32_t>(pin.port - 'A');
}

inline uint16_t gpioBit(board::Pin pin)
{
    return static_cast<uint16_t>(1u << pin.number);
}

/// Port clock enable. The GPIOx enable bits are consecutive in RCC_AHB1ENR.
inline rcc_periph_clken gpioRcc(board::Pin pin)
{
    return static_cast<rcc_periph_clken>(RCC_GPIOA + (pin.port - 'A'));
}

/// Enables the port clock and puts the pin in output mode.
inline void gpioOutput(board::Pin pin)
{
    rcc_periph_clock_enable(gpioRcc(pin));
    gpio_mode_setup(gpioPort(pin), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, gpioBit(pin));
}

/// Enables the port clock and hands the pin to a peripheral.
inline void gpioAlternate(board::Pin pin, uint8_t af, uint8_t pull = GPIO_PUPD_NONE)
{
    rcc_periph_clock_enable(gpioRcc(pin));
    gpio_mode_setup(gpioPort(pin), GPIO_MODE_AF, pull, gpioBit(pin));
    gpio_set_output_options(gpioPort(pin), GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, gpioBit(pin));
    gpio_set_af(gpioPort(pin), af, gpioBit(pin));
}
} // namespace hal::stm32

#endif // HAL_STM32_GPIO_H
