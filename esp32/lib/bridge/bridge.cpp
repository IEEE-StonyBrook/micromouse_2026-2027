#include "bridge.h"

#include <cstdio>
#include <cstring>

namespace bridge
{

void Bridge::set_connected(bool connected)
{
    if (connected == connected_)
    {
        return;
    }
    connected_ = connected;
    payload_   = MIN_PAYLOAD;
    if (!connected)
    {
        // Nobody will read what is still queued for the old connection.
        stats_.uplink_discarded += uplink_.size();
        gap_discarded_ += uplink_.size();
        uplink_.clear();
        gap_dropped_ = 0;
        return;
    }
    if (gap_discarded_ > 0)
    {
        char line[64];
        std::snprintf(line, sizeof line, "[bridge] discarded %lu bytes while disconnected\r\n",
                      static_cast<unsigned long>(gap_discarded_));
        pushMarker(line);
        gap_discarded_ = 0;
    }
}

void Bridge::set_mtu(uint16_t att_mtu)
{
    size_t p = att_mtu > 3 ? att_mtu - 3u : 0u;
    payload_ = p < MIN_PAYLOAD ? MIN_PAYLOAD : (p > MAX_PAYLOAD ? MAX_PAYLOAD : p);
}

bool Bridge::pushMarker(const char* text)
{
    size_t n = std::strlen(text);
    if (uplink_.free() < n)
    {
        return false;
    }
    for (size_t i = 0; i < n; ++i)
    {
        uplink_.push(static_cast<uint8_t>(text[i]));
    }
    return true;
}

void Bridge::pushFromUart(const uint8_t* data, size_t n, uint32_t now_ms)
{
    if (n == 0)
    {
        return;
    }
    last_uart_ms_ = now_ms;
    stats_.uart_rx_bytes += n;
    if (!connected_)
    {
        stats_.uplink_discarded += n;
        gap_discarded_ += n;
        return;
    }
    for (size_t i = 0; i < n; ++i)
    {
        if (gap_dropped_ > 0)
        {
            // Mark the gap where it happened, before the first byte after it;
            // until the marker fits, the gap keeps growing.
            char line[48];
            std::snprintf(line, sizeof line, "\r\n[bridge] dropped %lu bytes\r\n",
                          static_cast<unsigned long>(gap_dropped_));
            if (uplink_.free() > std::strlen(line) && pushMarker(line))
            {
                gap_dropped_ = 0;
            }
        }
        if (gap_dropped_ > 0 || !uplink_.push(data[i]))
        {
            ++stats_.uplink_dropped;
            ++gap_dropped_;
        }
    }
}

size_t Bridge::peekBleChunk(uint8_t* out, uint32_t now_ms) const
{
    if (!connected_)
    {
        return 0;
    }
    size_t n        = uplink_.size() < payload_ ? uplink_.size() : payload_;
    bool   full     = n == payload_;
    bool   has_line = false;
    for (size_t i = 0; i < n; ++i)
    {
        out[i] = uplink_.at(i);
        has_line |= out[i] == '\n';
    }
    // Unsigned subtraction keeps this right across the 49-day millis() wrap.
    bool idle = now_ms - last_uart_ms_ >= FLUSH_IDLE_MS;
    return (full || has_line || idle) ? n : 0;
}

void Bridge::commitBleChunk(size_t n)
{
    n = n < uplink_.size() ? n : uplink_.size();
    uplink_.drop(n);
    stats_.ble_tx_bytes += n;
}

void Bridge::pushFromBle(const uint8_t* data, size_t n)
{
    for (size_t i = 0; i < n; ++i)
    {
        if (!downlink_.push(data[i]))
        {
            ++stats_.downlink_dropped;
        }
    }
}

size_t Bridge::popForUart(uint8_t* out, size_t max)
{
    size_t n = downlink_.size() < max ? downlink_.size() : max;
    for (size_t i = 0; i < n; ++i)
    {
        out[i] = downlink_.at(i);
    }
    downlink_.drop(n);
    return n;
}

} // namespace bridge
