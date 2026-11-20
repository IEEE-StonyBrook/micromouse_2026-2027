#include "hal/host/memory_flash.h"

namespace hal::host
{
MemoryFlash::MemoryFlash(uint32_t sector_count, uint32_t sector_size_bytes)
    : sector_count_(sector_count), sector_size_bytes_(sector_size_bytes),
      words_(static_cast<std::size_t>(sector_count) * sector_size_bytes / 4, ERASED_WORD)
{
}

bool MemoryFlash::step(std::size_t index, uint32_t value)
{
    if (!powered_)
        return false;
    if (cut_armed_ && words_left_ == 0)
    {
        // Torn word: some bits reached their new value and some did not. The
        // mask is fixed per address so a test failure replays exactly.
        const uint32_t mask = 0x5A5A5A5Au ^ static_cast<uint32_t>(index * 2654435761u);
        words_[index]       = (value & mask) | (words_[index] & ~mask);
        powered_            = false;
        cut_armed_          = false;
        return false;
    }
    if (cut_armed_)
        --words_left_;
    words_[index] = value;
    ++word_ops_;
    return true;
}

bool MemoryFlash::erase(uint32_t sector)
{
    if (sector >= sector_count_)
        return false;
    const std::size_t first = static_cast<std::size_t>(sector) * sector_size_bytes_ / 4;
    for (std::size_t i = 0; i < sector_size_bytes_ / 4; ++i)
        if (!step(first + i, ERASED_WORD))
            return false;
    return true;
}

bool MemoryFlash::program(uint32_t offset_bytes, const uint32_t* words, std::size_t count)
{
    if (offset_bytes % 4 != 0 || offset_bytes / 4 + count > words_.size())
        return false;
    const std::size_t first = offset_bytes / 4;
    for (std::size_t i = 0; i < count; ++i)
        if (!step(first + i, words_[first + i] & words[i]))
            return false;
    return true;
}

void MemoryFlash::read(uint32_t offset_bytes, uint32_t* words, std::size_t count) const
{
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t index = offset_bytes / 4 + i;
        words[i]                = index < words_.size() ? words_[index] : ERASED_WORD;
    }
}

void MemoryFlash::cutPowerAfterWords(uint64_t words)
{
    cut_armed_  = true;
    words_left_ = words;
}

void MemoryFlash::restorePower()
{
    powered_ = true;
}

void MemoryFlash::poke(uint32_t offset_bytes, uint32_t value)
{
    words_[offset_bytes / 4] = value;
}
} // namespace hal::host
