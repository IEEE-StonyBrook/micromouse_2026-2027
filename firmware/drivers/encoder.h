#ifndef DRIVERS_ENCODER_H
#define DRIVERS_ENCODER_H

#include <cstdint>

#include "common/log.h"
#include "config/config.h"
#include "hal/peripherals.h"

#if defined(STM32_BUILD)
namespace hal::stm32
{
class QuadCounter;
}
#endif

/**
 * @brief Quadrature encoder reader using PIO hardware
 *
 * Implements a hardware-accelerated quadrature encoder decoder using the
 * Raspberry Pi Pico's PIO (Programmable I/O) peripheral. Tracks motor
 * rotation with high accuracy and low CPU overhead.
 */
class Encoder
{
  public:
#if defined(STM32_BUILD)
    /**
     * @brief STM32: counts from a timer in encoder mode, widened to 32 bits
     *        by the tick (drivers/stm32/encoder.cpp).
     * @param counter The timer's QuadCounter
     * @param invert_direction If true, inverts tick count direction
     */
    Encoder(const hal::stm32::QuadCounter& counter, bool invert_direction);
#else
    /**
     * @brief Constructs encoder reader with PIO configuration
     * @param pio_block PIO block that runs the decoder program
     * @param gpio_pin Base GPIO pin (channel B assumed to be next sequential pin)
     * @param invert_direction If true, inverts tick count direction
     */
    Encoder(hal::PioBlock pio_block, int gpio_pin, bool invert_direction = false);
#endif

    /**
     * @brief Resets encoder tick count to zero
     */
    void reset();

    /**
     * @brief Returns current encoder tick count
     * @return Total ticks since last reset (positive or negative)
     */
    int ticks() const;

  private:
#if defined(STM32_BUILD)
    const hal::stm32::QuadCounter* counter_;
    int                            offset_ticks_;
    bool                           invert_direction_;
#else
    void load_pio_program();

    const hal::PioBlock pio_block_;
    unsigned            state_machine_;
    int                 offset_ticks_;
    bool                invert_direction_;
    int                 gpio_pin_;
#endif
};

#endif
