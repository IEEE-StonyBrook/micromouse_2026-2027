#ifndef COMMON_CRC32_H
#define COMMON_CRC32_H

#include <cstddef>
#include <cstdint>

/**
 * CRC-32 (IEEE 802.3: reflected polynomial 0xEDB88320, initial and final
 * XOR 0xFFFFFFFF), bitwise. Slow but tableless: the flash store checks at
 * most 32 KB, and only at boot or on a save.
 */
namespace crc32
{
constexpr uint32_t INITIAL = 0xFFFFFFFFu;

inline uint32_t update(uint32_t state, const void* data, std::size_t len)
{
    const auto* bytes = static_cast<const uint8_t*>(data);
    for (std::size_t i = 0; i < len; ++i)
    {
        state ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit)
            state = (state >> 1) ^ (0xEDB88320u & (0u - (state & 1u)));
    }
    return state;
}

inline uint32_t finish(uint32_t state)
{
    return state ^ 0xFFFFFFFFu;
}

inline uint32_t compute(const void* data, std::size_t len)
{
    return finish(update(INITIAL, data, len));
}
} // namespace crc32

#endif // COMMON_CRC32_H
