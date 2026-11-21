// The encoder history must hold a full-speed tick's worth of counts (#17).
#include <cstdint>

#include "control/encoder_history.h"
#include "doctest/doctest.h"

TEST_CASE("encoder history holds +-200 counts per tick")
{
    // ~62 counts/mm at 1.6 m/s and 500 Hz is ~200 counts per tick.
    CHECK_FALSE(holdsCountsPerTick<int8_t>(200.0f));
    CHECK(holdsCountsPerTick<EncoderHistoryDelta>(200.0f));
    CHECK(holdsCountsPerTick<EncoderHistoryDelta>(-200.0f));

    volatile int delta = 200;
    CHECK(static_cast<int>(static_cast<int8_t>(delta)) != delta); // int8 wraps
    CHECK(static_cast<int>(static_cast<EncoderHistoryDelta>(delta)) == delta);
    CHECK(static_cast<int>(static_cast<EncoderHistoryDelta>(-delta)) == -delta);
}
