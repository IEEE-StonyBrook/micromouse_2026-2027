#include "hal/console.h"

namespace hal::console
{
namespace
{
// One writer context (the main loop) on every target; volatile so a later
// interrupt-side reader sees fresh values.
volatile bool     tx_gated         = false;
volatile uint32_t tx_dropped_bytes = 0;
} // namespace

void write(const char* buf, std::size_t len)
{
    if (buf == nullptr || len == 0)
        return;
    if (tx_gated)
    {
        tx_dropped_bytes = tx_dropped_bytes + static_cast<uint32_t>(len);
        return;
    }
    detail::writeBytes(buf, len);
}

void setTxGate(bool gated)
{
    tx_gated = gated;
}

bool txGated()
{
    return tx_gated;
}

uint32_t txGateDroppedBytes()
{
    return tx_dropped_bytes;
}

void resetTxGateDrops()
{
    tx_dropped_bytes = 0;
}
} // namespace hal::console
