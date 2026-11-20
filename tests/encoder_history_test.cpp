// Host check for the encoder history width (#17). Ticket #26 folds this into
// the host test target.
#include <cstdint>
#include <cstdio>

#include "control/encoder_history.h"

namespace
{
int failures = 0;

void check(bool ok, const char* what)
{
    std::printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        failures++;
}
} // namespace

int main()
{
    // ~62 counts/mm at 1.6 m/s and 500 Hz is ~200 counts per tick.
    check(!holdsCountsPerTick<int8_t>(200.0f), "int8 cannot hold 200 counts/tick");
    check(holdsCountsPerTick<EncoderHistoryDelta>(200.0f), "history type holds 200 counts/tick");
    check(holdsCountsPerTick<EncoderHistoryDelta>(-200.0f), "history type holds -200 counts/tick");

    volatile int delta       = 200;
    const int    int8_stored = static_cast<int8_t>(delta);
    check(int8_stored != delta, "int8 store of 200 wraps");
    const int history_stored = static_cast<EncoderHistoryDelta>(delta);
    check(history_stored == delta, "history store of 200 is exact");
    const int history_stored_neg = static_cast<EncoderHistoryDelta>(-delta);
    check(history_stored_neg == -delta, "history store of -200 is exact");

    return failures == 0 ? 0 : 1;
}
