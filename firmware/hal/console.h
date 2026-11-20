#ifndef HAL_CONSOLE_H
#define HAL_CONSOLE_H

#include <cstddef>
#include <cstdint>

/**
 * @brief Platform-neutral, non-blocking text console for the CLI and DriverLab.
 *        The gate lives in hal/console.cpp; the byte transport is implemented in
 *        hal/pico/console.cpp (USB stdio, plus the HC-05 UART when polled) and
 *        hal/host/console.cpp (in-memory queues, see hal/host/console_test.h).
 */
namespace hal::console
{
/// Next received byte (0..255), or -1 if none is waiting. Never blocks.
int getcNonBlocking();

/// Sends len bytes. While the TX gate is closed the bytes are dropped and counted.
void write(const char* buf, std::size_t len);

/// Closes (true) or opens (false) the TX gate. Used to keep the link quiet
/// during timing-critical windows.
void setTxGate(bool gated);

/// True while the TX gate is closed.
bool txGated();

/// Bytes dropped by the TX gate since boot or the last resetTxGateDrops().
uint32_t txGateDroppedBytes();

void resetTxGateDrops();

namespace detail
{
/// Backend transport: sends len bytes, ignoring the gate. Called only by write().
void writeBytes(const char* buf, std::size_t len);
} // namespace detail
} // namespace hal::console

#endif // HAL_CONSOLE_H
