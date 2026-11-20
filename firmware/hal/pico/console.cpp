#include "hal/console.h"
#include "hal/pico/console.h"

#include <cstdio>

#include "hardware/uart.h"
#include "pico/stdio.h"

namespace hal::console
{
namespace
{
bool uart_rx_polling = false;
} // namespace

void pico::setUartRxPolling(bool enabled)
{
    uart_rx_polling = enabled;
}

int getcNonBlocking()
{
    const int c = getchar_timeout_us(0); // USB CDC
    if (c >= 0)
        return c;
    if (uart_rx_polling && uart_is_readable(uart0))
        return static_cast<unsigned char>(uart_getc(uart0));
    return -1;
}

void detail::writeBytes(const char* buf, std::size_t len)
{
    // stdout reaches USB, plus the HC-05 when DriverLab's stdio mirror is on.
    std::fwrite(buf, 1, len, stdout);
}
} // namespace hal::console
