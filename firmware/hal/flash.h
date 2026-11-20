#ifndef HAL_FLASH_H
#define HAL_FLASH_H

#include <cstddef>
#include <cstdint>

namespace hal
{
/**
 * @brief A region of NOR flash made of equal erase sectors, addressed by byte
 *        offset from its start and written a 32-bit word at a time.
 *
 * Erasing sets every word of a sector to 0xFFFFFFFF; programming can only
 * clear bits, so a word is erased before it is written. The flash store
 * builds on this, so it tests on the host against hal/host/memory_flash.h.
 * Backends: hal/stm32/flash.h (sectors 1-2 of the STM32F405).
 */
class FlashDevice
{
  public:
    static constexpr uint32_t ERASED_WORD = 0xFFFFFFFFu;

    virtual uint32_t sectorCount() const     = 0;
    virtual uint32_t sectorSizeBytes() const = 0;

    /// Blocks until the sector reads back erased. False on a flash error.
    virtual bool erase(uint32_t sector) = 0;

    /// Writes count words from offset_bytes (4-byte aligned) in order, lowest
    /// address first. False on a flash error or an out-of-range write.
    virtual bool program(uint32_t offset_bytes, const uint32_t* words, std::size_t count) = 0;

    /// Reads count words from offset_bytes (4-byte aligned).
    virtual void read(uint32_t offset_bytes, uint32_t* words, std::size_t count) const = 0;

  protected:
    ~FlashDevice() = default;
};
} // namespace hal

#endif // HAL_FLASH_H
