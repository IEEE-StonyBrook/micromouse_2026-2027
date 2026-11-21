#ifndef HAL_HOST_CONSOLE_TEST_H
#define HAL_HOST_CONSOLE_TEST_H

#include <string>

/**
 * @brief Test hooks for the host console backend: feed input, read back output.
 */
namespace hal::console::host
{
/// Queues text as if it had arrived on the console.
void pushInput(const std::string& text);

/// Returns everything written since the last call, and clears it.
std::string takeOutput();

/// Empties both queues, opens the TX gate and zeroes the drop count.
void reset();
} // namespace hal::console::host

#endif // HAL_HOST_CONSOLE_TEST_H
