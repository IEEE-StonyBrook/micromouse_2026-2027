#ifndef HAL_STM32_FLASH_H
#define HAL_STM32_FLASH_H

#include "hal/flash.h"
#include "hal/stm32/flash_layout.h"

namespace hal::stm32
{
/**
 * @brief The flash store's region of on-chip flash: sectors 1-2, 16 KB each,
 *        between _flash_store_start and _flash_store_end in stm32f405.ld.
 *
 * Erases and programs 32 bits at a time (PSIZE x32), which RM0090 allows
 * only with VDD at 2.7-3.6 V. Every call blocks, and while the flash is busy
 * any code or interrupt fetching from flash stalls with it: a 16 KB sector
 * erase takes up to 500 ms (DS8626 t_ERASE16KB). Callers save only through
 * FlashStore, whose SaveGuard keeps the mouse disarmed and its outputs off.
 */
class StoreFlash final : public hal::FlashDevice
{
  public:
    /// Reads the region from the linker symbols. False if it is not a run
    /// of whole equal sectors; every erase and program then fails.
    bool init();

    uint32_t sectorCount() const override { return layout_.count; }
    uint32_t sectorSizeBytes() const override { return layout_.sector_bytes; }
    bool     erase(uint32_t sector) override;
    bool     program(uint32_t offset_bytes, const uint32_t* words, std::size_t count) override;
    void     read(uint32_t offset_bytes, uint32_t* words, std::size_t count) const override;

  private:
    uint32_t     base_   = 0;
    StoreSectors layout_ = {false, 0, 0, 0};
};
} // namespace hal::stm32

#endif // HAL_STM32_FLASH_H
