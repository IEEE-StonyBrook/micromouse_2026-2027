#ifndef HAL_STM32_CONSOLE_H
#define HAL_STM32_CONSOLE_H

#include <cstdint>

/**
 * @brief STM32 console setup and counters, on top of hal/console.h.
 *
 * USART3 at 1 Mbaud on the Feather's TX/RX pins. Transmit goes through a
 * 2 KB ring drained by DMA: hal::console::write() and printf (through the
 * retargeted _write) never block, and a write that does not fit is dropped
 * and counted. Receive is a 256-byte circular DMA buffer; the USART's
 * idle-line interrupt marks the end of each burst and wakes the main loop.
 */
namespace hal::console::stm32
{
/// Sets up the pins, USART3 and both DMA streams. Call after setupClocks().
void init();

/// Bytes refused because the TX ring was full (separate from the TX gate's count).
uint32_t txRingDroppedBytes();

void resetTxRingDrops();

/// Receive bursts ended by an idle line since init().
uint32_t rxIdleEvents();

/// USART receive overruns since init(): bytes lost before the DMA could take them.
uint32_t rxOverruns();
} // namespace hal::console::stm32

#endif // HAL_STM32_CONSOLE_H
