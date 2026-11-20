#include "hal/irq.h"

#include <mutex>

namespace hal::irq
{
namespace
{
// Recursive, as on the Pico a nested Lock is harmless.
std::recursive_mutex& mutex()
{
    static std::recursive_mutex m;
    return m;
}
} // namespace

Lock::Lock() : saved_(0)
{
    mutex().lock();
}

Lock::~Lock()
{
    (void)saved_; // the host has no interrupt state to restore
    mutex().unlock();
}
} // namespace hal::irq
