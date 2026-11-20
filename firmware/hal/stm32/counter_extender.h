#ifndef HAL_STM32_COUNTER_EXTENDER_H
#define HAL_STM32_COUNTER_EXTENDER_H

/**
 * @file counter_extender.h
 * @brief Widens a free-running hardware counter in software. No libopencm3;
 *        host-tested in tests/counter_extender_test.cpp.
 */

#include <cstdint>
#include <type_traits>

namespace hal::stm32
{
/**
 * Accumulates the change of a Raw-bit hardware counter into a Wide value.
 *
 * Each update() adds raw - last, read as a signed step when Wide is signed
 * (an encoder that can count either way) and as an unsigned step otherwise
 * (a timer that only counts up). It is correct as long as update() runs at
 * least once per half wrap of Raw for a signed counter, once per wrap for an
 * unsigned one; the 1 kHz tick meets that for both users.
 *
 * Not thread-safe: one context updates, and readers elsewhere take a lock.
 */
template <typename Raw, typename Wide> class CounterExtender
{
    static_assert(std::is_unsigned<Raw>::value, "hardware counters are unsigned");
    static_assert(sizeof(Wide) > sizeof(Raw), "Wide must be wider than Raw");

    using UWide = typename std::make_unsigned<Wide>::type;
    using SRaw  = typename std::make_signed<Raw>::type;

  public:
    /// Restarts from the counter's current raw value, reporting `value` there.
    void reset(Raw raw, Wide value = 0)
    {
        last_  = raw;
        value_ = static_cast<UWide>(value);
    }

    /// Folds in a new sample and returns the widened count.
    Wide update(Raw raw)
    {
        value_ = static_cast<UWide>(value_ + step(raw));
        last_  = raw;
        return value();
    }

    /// The widened count at `raw`, without recording the sample.
    Wide peek(Raw raw) const { return static_cast<Wide>(static_cast<UWide>(value_ + step(raw))); }

    /// The widened count at the last update().
    Wide value() const { return static_cast<Wide>(value_); }

    /// The raw value seen at the last update().
    Raw lastRaw() const { return last_; }

  private:
    UWide step(Raw raw) const
    {
        const Raw diff = static_cast<Raw>(raw - last_);
        if (std::is_signed<Wide>::value)
            return static_cast<UWide>(static_cast<Wide>(static_cast<SRaw>(diff)));
        return static_cast<UWide>(diff);
    }

    // Accumulated unsigned, so wrapping the Wide range is defined behaviour.
    UWide value_ = 0;
    Raw   last_  = 0;
};

/// TIM5's 32-bit microsecond count widened to 64 bits (wraps after 584 000 years).
using TimeExtender = CounterExtender<uint32_t, uint64_t>;
} // namespace hal::stm32

#endif // HAL_STM32_COUNTER_EXTENDER_H
