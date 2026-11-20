#ifndef HAL_STM32_QUAD_COUNTER_H
#define HAL_STM32_QUAD_COUNTER_H

#include <cstdint>

#include "config/board_feather.h"
#include "hal/stm32/counter_extender.h"

namespace hal::stm32
{
/**
 * @brief A quadrature encoder counted in hardware by a general-purpose timer.
 *
 * The timer runs in encoder mode 3, counting every edge of both channels
 * (4 counts per encoder cycle), with an 8-sample input filter at the 84 MHz
 * timer clock: a level must hold for about 95 ns to count, which rejects
 * ringing without limiting any real wheel speed. The 16-bit hardware count
 * is widened to 32 bits by update(), which the 1 kHz tick calls in every mode.
 *
 * The count rises when channel A leads B. Which way that turns the wheel
 * depends on the wiring; the drive layer applies the sign.
 */
class QuadCounter
{
  public:
    /// a and b are channels 1 and 2 of one timer (TIM3 or TIM4).
    void init(const board::TimerPin& a, const board::TimerPin& b);

    /// The raw hardware count.
    uint16_t raw16() const;

    /// Folds the hardware count into the 32-bit count. Tick context only.
    void update();

    /// The 32-bit count at the last update(). Safe from any context.
    int32_t count() const;

  private:
    uint32_t        timer_ = 0;
    EncoderExtender extended_;
};
} // namespace hal::stm32

#endif // HAL_STM32_QUAD_COUNTER_H
