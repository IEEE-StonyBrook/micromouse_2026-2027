// UART <-> BLE bridging logic for the ESP32-C3, free of Arduino and NimBLE so
// it runs under `pio test -e native`. src/main.cpp is the thin glue.
//
// Uplink (STM32 -> host): UART bytes are buffered and leave as BLE
// notifications of at most the negotiated payload (ATT MTU - 3).
// Downlink (host -> STM32): BLE writes are buffered and drained to the UART.
// Neither direction ever blocks; a full buffer drops and counts, and the host
// sees a "[bridge] ..." line wherever uplink bytes went missing.
#pragma once

#include <cstddef>
#include <cstdint>

#include "ring.h"

namespace bridge
{

constexpr size_t MIN_PAYLOAD = 20;  // default ATT MTU 23 - 3 header bytes
constexpr size_t MAX_PAYLOAD = 244; // fits one LE data-length-extended packet

// About 40 ms of UART output at 1 Mbaud: rides out BLE connection-interval
// stalls without holding stale output for long.
constexpr size_t UPLINK_CAPACITY = 4096;
// Host commands are short lines, and the UART drains far faster than BLE fills.
constexpr size_t DOWNLINK_CAPACITY = 1024;

// A partial line (the CLI's "> " prompt has no newline) is held this long
// after the last UART byte before it is sent, so a line arriving at 1 Mbaud
// (about 100 bytes/ms) leaves in one notification instead of dozens.
constexpr uint32_t FLUSH_IDLE_MS = 5;

struct Stats
{
    uint32_t uart_rx_bytes    = 0; // bytes read from the STM32
    uint32_t ble_tx_bytes     = 0; // bytes notified to the host
    uint32_t uplink_dropped   = 0; // UART bytes lost to a full uplink buffer
    uint32_t uplink_discarded = 0; // UART bytes thrown away with no host
    uint32_t downlink_dropped = 0; // host bytes lost to a full downlink buffer
};

class Bridge
{
  public:
    // A new connection starts at the default MTU and, if UART bytes were
    // discarded since the last connection, with a line saying how many.
    void set_connected(bool connected);
    bool connected() const { return connected_; }

    // Takes the negotiated ATT MTU; the notification payload is 3 bytes less,
    // clamped to [MIN_PAYLOAD, MAX_PAYLOAD].
    void   set_mtu(uint16_t att_mtu);
    size_t payload() const { return payload_; }

    void pushFromUart(const uint8_t* data, size_t n, uint32_t now_ms);

    // Copies the next notification payload into out (MAX_PAYLOAD bytes) and
    // returns its length, or 0 when nothing is ready. The bytes stay buffered
    // until commitBleChunk(), so a failed notify is retried, not lost.
    size_t peekBleChunk(uint8_t* out, uint32_t now_ms) const;
    void   commitBleChunk(size_t n);

    void   pushFromBle(const uint8_t* data, size_t n);
    size_t popForUart(uint8_t* out, size_t max);

    const Stats& stats() const { return stats_; }

  private:
    // Appends text to the uplink if it fits whole; returns whether it did.
    bool pushMarker(const char* text);

    Ring<UPLINK_CAPACITY>   uplink_;
    Ring<DOWNLINK_CAPACITY> downlink_;
    Stats                   stats_;
    bool                    connected_     = false;
    size_t                  payload_       = MIN_PAYLOAD;
    uint32_t                last_uart_ms_  = 0;
    uint32_t                gap_dropped_   = 0; // dropped, not yet marked
    uint32_t                gap_discarded_ = 0; // discarded, not yet marked
};

} // namespace bridge
