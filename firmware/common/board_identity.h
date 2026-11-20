#ifndef COMMON_BOARD_IDENTITY_H
#define COMMON_BOARD_IDENTITY_H

#include <cstddef>
#include <cstdint>

#include "common/flash_store.h"
#include "config/board_profiles.h"

/// Layout version of the settings record; bump it whenever what the store
/// holds changes shape, so no board loads a record it would misread.
constexpr uint32_t SETTINGS_VERSION = 1;

/// CRC-32 of the 96-bit chip UID, as little-endian bytes in word order. The
/// store writes it into every record and loads only records with its own.
uint32_t uidHash(const uint32_t (&uid)[3]);

/// A name that is non-empty and terminated, a known IR set, has_fan 0 or 1
/// and a divider of at least 1.
bool profileValid(const board::BoardProfile& profile);

/**
 * @brief Which mouse this board is, from the profile it saved under its UID.
 *
 * A board with no valid profile of its own is unknown: it must pick a preset
 * (pick()) before it may arm. The store is built with this chip's uidHash,
 * so a record copied from another board does not count.
 */
class BoardIdentity
{
  public:
    explicit BoardIdentity(FlashStore& store);

    /// Loads the saved profile. The status drives the store's LED code.
    LoadStatus boot();

    bool identified() const { return identified_; }
    /// Arming is refused until the board knows which mouse it is.
    bool armAllowed() const { return identified_; }
    /// Only meaningful when identified().
    const board::BoardProfile& profile() const { return profile_; }

    /// Saves preset `index` as this board's profile. An index past the
    /// presets is refused (kRefused with blocker kNone) without touching
    /// flash; a store refusal or error leaves the identity as it was.
    SaveResult pick(std::size_t index);

  private:
    FlashStore&         store_;
    board::BoardProfile profile_{};
    bool                identified_ = false;
};

#endif // COMMON_BOARD_IDENTITY_H
