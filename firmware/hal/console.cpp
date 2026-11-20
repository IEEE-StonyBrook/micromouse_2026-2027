#include "hal/console.h"

#include <cstdarg>
#include <cstdio>
#include <string>

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

void printFormat(const char* format, ...)
{
    if (format == nullptr)
        return;
    char    buffer[384];
    va_list args;
    va_start(args, format);
    const int length = std::vsnprintf(buffer, sizeof buffer, format, args);
    va_end(args);
    if (length <= 0)
        return;
    if (static_cast<std::size_t>(length) < sizeof buffer)
    {
        write(buffer, static_cast<std::size_t>(length));
        return;
    }
    std::string long_line(static_cast<std::size_t>(length) + 1, '\0');
    va_start(args, format);
    std::vsnprintf(&long_line[0], long_line.size(), format, args);
    va_end(args);
    write(long_line.data(), static_cast<std::size_t>(length));
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
