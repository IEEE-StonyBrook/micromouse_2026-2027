#ifndef HAL_TIME_H
#define HAL_TIME_H

#include <cstdint>

/**
 * @brief Platform-neutral clock. Implemented in hal/pico/time.cpp (the SDK
 *        timer), hal/stm32/time.cpp (TIM5) and hal/host/time.cpp (std::chrono).
 */
namespace hal::time
{
/// Milliseconds since boot. Wraps at 2^32 (about 49.7 days); compare with
/// unsigned subtraction.
uint32_t nowMs();

/// Microseconds since boot, 64 bits wide so it never wraps in practice.
uint64_t nowUs();

/// Blocks the caller for at least duration_ms. Never call from an interrupt.
void sleepMs(uint32_t duration_ms);
} // namespace hal::time

#endif // HAL_TIME_H
