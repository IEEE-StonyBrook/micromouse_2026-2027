// STM32 half of DriverLab's pin-level diagnostics (driver_lab/board_io.h).
#include "driver_lab/board_io.h"

#include <libopencm3/stm32/gpio.h>

#include "config/board_feather.h"
#include "hal/stm32/gpio.h"

namespace driver_lab_board
{
void setupRightEncoderPinsRaw()
{
    // Nothing to do: the input data register reads a pin in alternate-function
    // mode too, so the pins stay on TIM4 and the encoder keeps counting.
}

void readRightEncoderPins(bool& pin_a, bool& pin_b)
{
    const board::Pin a = board::ENCODER_PINS[2].pin;
    const board::Pin b = board::ENCODER_PINS[3].pin;
    pin_a              = gpio_get(hal::stm32::gpioPort(a), hal::stm32::gpioBit(a)) != 0;
    pin_b              = gpio_get(hal::stm32::gpioPort(b), hal::stm32::gpioBit(b)) != 0;
}

void driveMotorsRaw(bool, float)
{
    // DIRTEST sets the Pico's TB6552 DIR and PWM pins directly. The DRV8874s
    // have no DIR pin and are driven only through MotorBridge's arm gate, so
    // DIRTEST drives nothing here; OL at a small voltage shows each wheel's
    // direction instead.
}
} // namespace driver_lab_board
