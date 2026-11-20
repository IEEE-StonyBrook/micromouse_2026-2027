#include "hal/time.h"

#include <chrono>
#include <thread>

namespace hal::time
{
namespace
{
// "Boot" on the host is static initialisation, so times start near zero as on the Pico.
const std::chrono::steady_clock::time_point kBoot = std::chrono::steady_clock::now();
} // namespace

uint32_t nowMs()
{
    const auto elapsed = std::chrono::steady_clock::now() - kBoot;
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
}

uint64_t nowUs()
{
    const auto elapsed = std::chrono::steady_clock::now() - kBoot;
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}

void sleepMs(uint32_t duration_ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
}
} // namespace hal::time
