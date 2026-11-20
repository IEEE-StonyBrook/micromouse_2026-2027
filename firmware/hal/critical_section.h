#ifndef HAL_CRITICAL_SECTION_H
#define HAL_CRITICAL_SECTION_H

#include <cstdint>

namespace hal
{
/**
 * @brief Lock that excludes both interrupts and the other core.
 *
 * On the Pico this wraps the SDK critical_section_t: a hardware spin lock
 * taken with interrupts disabled on the calling core. On the host it is a
 * spin lock. Hold it only for a few dozen instructions.
 *
 * The platform state lives in opaque storage so this header needs no SDK;
 * each backend checks at compile time that its state fits.
 */
class CriticalSection
{
  public:
    /// Claims the platform lock. Call once before the first enter().
    void init();
    void enter();
    void exit();

  private:
    alignas(4) unsigned char storage_[8];
};
} // namespace hal

#endif // HAL_CRITICAL_SECTION_H
