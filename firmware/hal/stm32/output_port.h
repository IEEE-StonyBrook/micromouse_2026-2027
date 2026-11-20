#ifndef HAL_STM32_OUTPUT_PORT_H
#define HAL_STM32_OUTPUT_PORT_H

#include "hal/output_port.h"
#include "hal/stm32/gpio.h"

namespace hal::stm32
{
/**
 * @brief hal::OutputPort on the STM32's GPIO registers. Works on the 16 MHz
 *        HSI straight out of reset, before setupClocks(): it only enables the
 *        port clock and writes the port registers.
 */
class OutputPort final : public hal::OutputPort
{
  public:
    /// Writes ODR through BSRR. The port clock must run for the write to land,
    /// so this enables it first.
    void setLevel(board::Pin pin, bool high) override
    {
        rcc_periph_clock_enable(gpioRcc(pin));
        if (high)
            gpio_set(gpioPort(pin), gpioBit(pin));
        else
            gpio_clear(gpioPort(pin), gpioBit(pin));
    }

    /// MODER to output; OTYPER is push-pull from reset.
    void makeOutput(board::Pin pin) override
    {
        rcc_periph_clock_enable(gpioRcc(pin));
        gpio_mode_setup(gpioPort(pin), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, gpioBit(pin));
    }
};
} // namespace hal::stm32

#endif // HAL_STM32_OUTPUT_PORT_H
