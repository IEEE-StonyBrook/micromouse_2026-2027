#include "common/board_identity.h"

#include "common/crc32.h"

uint32_t uidHash(const uint32_t (&uid)[3])
{
    uint8_t bytes[12];
    for (int i = 0; i < 12; ++i)
        bytes[i] = static_cast<uint8_t>(uid[i / 4] >> (8 * (i % 4)));
    return crc32::compute(bytes, sizeof bytes);
}

bool profileValid(const board::BoardProfile& profile)
{
    bool terminated = false;
    for (char c : profile.name)
        terminated = terminated || c == '\0';
    // >= is false for a NaN divider, so that is rejected too.
    return terminated && profile.name[0] != '\0' && profile.ir_set < board::IR_SET_COUNT &&
           profile.has_fan <= 1 && profile.battery_divider >= 1.0f;
}

BoardIdentity::BoardIdentity(FlashStore& store) : store_(store)
{
}

LoadStatus BoardIdentity::boot()
{
    board::BoardProfile loaded{};
    const LoadResult    result = store_.load(&loaded, sizeof loaded);
    identified_                = result.len == sizeof loaded && profileValid(loaded);
    if (identified_)
        profile_ = loaded;
    return result.status;
}

SaveResult BoardIdentity::pick(std::size_t index)
{
    if (index >= board::PROFILE_PRESET_COUNT)
        return {SaveStatus::kRefused, SaveBlocker::kNone, 0};
    const board::BoardProfile& chosen = board::PROFILE_PRESETS[index];
    const SaveResult           result = store_.save(&chosen, sizeof chosen);
    if (result.status == SaveStatus::kOk)
    {
        profile_    = chosen;
        identified_ = true;
    }
    return result;
}
