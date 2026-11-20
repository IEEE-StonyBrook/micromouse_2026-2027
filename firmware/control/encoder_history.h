#ifndef CONTROL_ENCODER_HISTORY_H
#define CONTROL_ENCODER_HISTORY_H

#include <cstdint>
#include <limits>

#include "config/geometry.h"
#include "config/motion.h"

// Per-tick encoder delta stored in the moving-average ring buffers of
// Drivetrain and DriverLab. int8 tops out at 127 counts/tick, which a
// ~62 counts/mm mouse passes near 1 m/s at 500 Hz.
using EncoderHistoryDelta = int16_t;

template <typename T> constexpr bool holdsCountsPerTick(float counts)
{
    return counts <= static_cast<float>(std::numeric_limits<T>::max()) &&
           counts >= static_cast<float>(std::numeric_limits<T>::min());
}

// Worst case: the fastest wheel over the finer of the two tick sizes.
constexpr float MAX_ENCODER_COUNTS_PER_TICK =
    ROBOT_MAX_WHEEL_SPEED_MMPS /
    ((MM_PER_TICK_LEFT < MM_PER_TICK_RIGHT ? MM_PER_TICK_LEFT : MM_PER_TICK_RIGHT) *
     LOOP_FREQUENCY_HZ);

static_assert(holdsCountsPerTick<EncoderHistoryDelta>(MAX_ENCODER_COUNTS_PER_TICK) &&
                  holdsCountsPerTick<EncoderHistoryDelta>(-MAX_ENCODER_COUNTS_PER_TICK),
              "encoder history type too narrow for counts/mm x top speed / loop rate");

#endif // CONTROL_ENCODER_HISTORY_H
