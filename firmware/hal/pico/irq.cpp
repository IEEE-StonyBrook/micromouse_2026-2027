#include "hal/irq.h"

#include "hardware/sync.h"

namespace hal::irq
{
Lock::Lock() : saved_(save_and_disable_interrupts())
{
}

Lock::~Lock()
{
    restore_interrupts(saved_);
}
} // namespace hal::irq
