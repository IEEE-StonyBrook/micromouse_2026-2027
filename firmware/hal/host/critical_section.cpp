#include "hal/critical_section.h"

#include <atomic>
#include <new>

namespace hal
{
namespace
{
std::atomic_flag* flag(unsigned char* storage)
{
    return std::launder(reinterpret_cast<std::atomic_flag*>(storage));
}
} // namespace

void CriticalSection::init()
{
    static_assert(sizeof(std::atomic_flag) <= sizeof(storage_),
                  "std::atomic_flag outgrew hal::CriticalSection storage");
    new (storage_) std::atomic_flag();
    flag(storage_)->clear();
}

void CriticalSection::enter()
{
    while (flag(storage_)->test_and_set(std::memory_order_acquire))
    {
    }
}

void CriticalSection::exit()
{
    flag(storage_)->clear(std::memory_order_release);
}
} // namespace hal
