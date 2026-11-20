// Host tests for the microsecond half of hal::time against the std::chrono backend.
#include "doctest/doctest.h"
#include "hal/time.h"

TEST_CASE("sleepUntilUs returns no earlier than its deadline")
{
    const uint64_t deadline_us = hal::time::nowUs() + 3000;
    hal::time::sleepUntilUs(deadline_us);
    CHECK(hal::time::nowUs() >= deadline_us);
}

TEST_CASE("sleepUntilUs on a past deadline returns at once")
{
    const uint64_t start_us = hal::time::nowUs();
    hal::time::sleepUntilUs(0); // boot, always in the past
    // Generous bound: a missed deadline must not block for anything like a tick.
    CHECK(hal::time::nowUs() - start_us < 1000);
}

TEST_CASE("nowUs and nowMs read the same clock")
{
    const uint64_t now_us = hal::time::nowUs();
    const uint32_t now_ms = hal::time::nowMs();
    CHECK(now_ms >= static_cast<uint32_t>(now_us / 1000));
    CHECK(now_ms - static_cast<uint32_t>(now_us / 1000) <= 5);
}
