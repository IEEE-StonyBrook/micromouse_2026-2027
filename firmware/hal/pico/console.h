#ifndef HAL_PICO_CONSOLE_H
#define HAL_PICO_CONSOLE_H

/**
 * @brief Pico-only console setup, called from main_pico.cpp.
 */
namespace hal::console::pico
{
/// Also read the HC-05 UART (uart0) in getcNonBlocking(), after USB. Turn it
/// on only where nothing else owns uart0 RX: DriverLab polls it, while the CLI
/// leaves it to the Bluetooth RX interrupt. Off by default.
void setUartRxPolling(bool enabled);
} // namespace hal::console::pico

#endif // HAL_PICO_CONSOLE_H
