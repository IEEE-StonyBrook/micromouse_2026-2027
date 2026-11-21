#ifndef HAL_HOST_MEMORY_FLASH_H
#define HAL_HOST_MEMORY_FLASH_H

#include <cstdint>
#include <vector>

#include "hal/flash.h"

namespace hal::host
{
/**
 * @brief NOR flash in host memory that can lose power at any word.
 *
 * Erase and program both count one operation per word. After
 * cutPowerAfterWords(n), n more words complete and the next operation is
 * torn: a program leaves that word holding a mix of its old and new bits,
 * and an erase leaves every word of the sector part erased (a real erase is
 * not ordered by address). The device then ignores every erase and program
 * until restorePower(). Reads always work.
 */
class MemoryFlash final : public hal::FlashDevice
{
  public:
    MemoryFlash(uint32_t sector_count, uint32_t sector_size_bytes);

    uint32_t sectorCount() const override { return sector_count_; }
    uint32_t sectorSizeBytes() const override { return sector_size_bytes_; }
    bool     erase(uint32_t sector) override;
    bool     program(uint32_t offset_bytes, const uint32_t* words, std::size_t count) override;
    void     read(uint32_t offset_bytes, uint32_t* words, std::size_t count) const override;

    void cutPowerAfterWords(uint64_t words);
    void restorePower();
    bool powered() const { return powered_; }

    /// Words erased or programmed since construction.
    uint64_t wordOps() const { return word_ops_; }

    /// Overwrites a word directly, bypassing the erase rule, to corrupt a record.
    void poke(uint32_t offset_bytes, uint32_t value);

  private:
    /// Which bits of a torn word reach their new value. Fixed per address so
    /// a failing power-cut case replays exactly.
    static uint32_t tearMask(std::size_t index);

    /// Applies one word operation; false once power is gone.
    bool step(std::size_t index, uint32_t value);

    uint32_t              sector_count_;
    uint32_t              sector_size_bytes_;
    std::vector<uint32_t> words_;
    bool                  powered_    = true;
    bool                  cut_armed_  = false;
    uint64_t              words_left_ = 0;
    uint64_t              word_ops_   = 0;
};
} // namespace hal::host

#endif // HAL_HOST_MEMORY_FLASH_H
