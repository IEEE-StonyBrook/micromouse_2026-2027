// Host tests for CounterExtender, which widens a free-running hardware counter
// (TIM5's 32-bit microsecond count, the encoder timers' 16-bit counts) as long
// as it is sampled at least once per half wrap.
#include <cstdint>

#include "doctest/doctest.h"
#include "hal/stm32/counter_extender.h"

using hal::stm32::TimeExtender;

TEST_CASE("Time extender counts on past the 32-bit wrap")
{
    TimeExtender t;
    t.reset(0xFFFFFC18u); // 1000 us before the wrap
    CHECK(t.update(0xFFFFFFFFu) == 999u);
    CHECK(t.update(0x000003E8u) == 2000u); // wrapped: 1000 us past zero
    CHECK(t.value() == 2000u);
}

TEST_CASE("Time extender passes 2^32 us (71.6 minutes) without losing the high word")
{
    TimeExtender t;
    t.reset(0);
    uint32_t raw = 0;
    // 1 ms ticks for 72 minutes, as the TIM7 tick samples it.
    for (uint32_t i = 0; i < 72u * 60u * 1000u; ++i)
    {
        raw += 1000u;
        t.update(raw);
    }
    CHECK(t.value() == 72ull * 60ull * 1000000ull);
}

TEST_CASE("Time extender peek reads between samples without moving the base")
{
    TimeExtender t;
    t.reset(0xFFFFFF00u, 5000u);
    CHECK(t.peek(0x00000100u) == 5000u + 0x200u);
    CHECK(t.value() == 5000u);
    CHECK(t.update(0x00000010u) == 5000u + 0x110u);
}

using hal::stm32::EncoderExtender;

TEST_CASE("Encoder extender counts forward through the 16-bit wrap")
{
    EncoderExtender e;
    e.reset(65530);
    CHECK(e.update(65535) == 5);
    CHECK(e.update(4) == 10); // 65535 -> 0 -> 4
    CHECK(e.update(300) == 306);
}

TEST_CASE("Encoder extender counts backward through the 16-bit wrap")
{
    EncoderExtender e;
    e.reset(3);
    CHECK(e.update(0) == -3);
    CHECK(e.update(65530) == -9); // 0 -> 65535 -> 65530
    CHECK(e.update(65000) == -539);
}

TEST_CASE("Encoder extender runs past +/-32768 counts, which a raw int16 cannot")
{
    EncoderExtender fwd;
    fwd.reset(0);
    uint16_t raw = 0;
    for (int i = 0; i < 1000; ++i) // 1000 ticks of +200 counts
    {
        raw = static_cast<uint16_t>(raw + 200);
        fwd.update(raw);
    }
    CHECK(fwd.value() == 200000);

    EncoderExtender back;
    back.reset(0);
    raw = 0;
    for (int i = 0; i < 1000; ++i)
    {
        raw = static_cast<uint16_t>(raw - 200);
        back.update(raw);
    }
    CHECK(back.value() == -200000);
}

TEST_CASE("Encoder extender takes up to 32767 counts per sample either way")
{
    EncoderExtender e;
    e.reset(0);
    CHECK(e.update(32767) == 32767);
    e.reset(0);
    CHECK(e.update(static_cast<uint16_t>(-32767)) == -32767);
}
