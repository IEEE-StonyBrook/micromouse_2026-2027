// Host tests for the STM32 busy-wait that stands in for the sleeps before the
// tick starts. The cycle counter is faked, wrap included.
#include <cstdint>

#include "doctest/doctest.h"
#include "hal/stm32/spin_wait.h"

using hal::stm32::spinForUs;

namespace
{
/// A 32-bit cycle counter that advances `step` cycles per read.
struct FakeCycles
{
    uint32_t now;
    uint32_t step;
    uint64_t reads = 0;
    uint32_t operator()()
    {
        ++reads;
        const uint32_t v = now;
        now += step;
        return v;
    }
};
} // namespace

TEST_CASE("STM32 spin wait: zero returns without waiting")
{
    FakeCycles c{0, 1};
    spinForUs(0, [&] { return c(); }, 168);
    CHECK(c.reads <= 1);
}

TEST_CASE("STM32 spin wait: waits at least the requested cycles, across a counter wrap")
{
    FakeCycles     c{0xFFFFF000u, 100};
    const uint32_t start = c.now;
    spinForUs(10, [&] { return c(); }, 168); // 1680 cycles
    const uint32_t elapsed = c.now - start;
    CHECK(elapsed >= 1680u);
    CHECK(elapsed < 1680u + 3 * 100u);
}

TEST_CASE("STM32 spin wait: a wait longer than one 32-bit counter period still ends on time")
{
    // 30 s at 168 MHz is 5.04e9 cycles, past the 2^32 (25.6 s) wrap.
    FakeCycles c{0, 1u << 20};
    spinForUs(30000000ull, [&] { return c(); }, 168);
    const uint64_t elapsed = c.reads * (1ull << 20);
    CHECK(elapsed >= 5040000000ull);
    CHECK(elapsed < 5040000000ull + 3 * (1ull << 20));
}
