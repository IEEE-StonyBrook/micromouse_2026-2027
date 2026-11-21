#ifndef HAL_IRQ_H
#define HAL_IRQ_H

#include <cstdint>

namespace hal::irq
{
/**
 * @brief Scoped lock that keeps interrupt handlers, the control tick among
 * them, from running on the calling core while it is held.
 *
 * Main-loop code takes it to hand data to or read data from an interrupt
 * handler without the handler seeing half an update. It nests, and it is safe
 * to take inside a handler. It does not exclude the other core: use
 * hal::CriticalSection for that.
 *
 * Backends: the Pico saves PRIMASK and disables interrupts on this core
 * (save_and_disable_interrupts / restore_interrupts), so hold it only for a
 * few dozen instructions. The host takes a process-wide recursive mutex, so a
 * test thread and a tick thread can share state. The STM32 saves PRIMASK and
 * masks interrupts (cm_mask_interrupts), like the Pico.
 */
class Lock
{
  public:
    Lock();
    ~Lock();
    Lock(const Lock&)            = delete;
    Lock& operator=(const Lock&) = delete;

  private:
    uint32_t saved_;
};
} // namespace hal::irq

#endif // HAL_IRQ_H
