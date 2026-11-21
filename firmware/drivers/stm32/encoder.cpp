// STM32 Encoder: a QuadCounter's 32-bit count, which the tick widens every
// tick, with the wheel's direction applied.
#include "drivers/encoder.h"

#include "hal/stm32/quad_counter.h"

Encoder::Encoder(const hal::stm32::QuadCounter& counter, bool invert_direction)
    : counter_(&counter), offset_ticks_(0), invert_direction_(invert_direction)
{
}

int Encoder::ticks() const
{
    const int raw = static_cast<int>(counter_->count());
    return (invert_direction_ ? -raw : raw) - offset_ticks_;
}

void Encoder::reset()
{
    offset_ticks_ += ticks();
}
