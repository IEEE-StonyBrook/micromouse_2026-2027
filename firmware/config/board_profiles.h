#ifndef CONFIG_BOARD_PROFILES_H
#define CONFIG_BOARD_PROFILES_H

/**
 * @file board_profiles.h
 * @brief What differs between the STM32 mice, and the presets an unknown
 *        board picks from on first boot.
 *
 * A board saves its chosen profile in the flash store, bound to its chip
 * UID (common/board_identity.h), so this file never lists UIDs.
 *
 * BoardProfile is stored byte for byte: changing its layout means bumping
 * SETTINGS_VERSION in common/board_identity.h, which sends every board back
 * to the picker.
 */

#include <cstddef>
#include <cstdint>

namespace board
{
constexpr std::size_t PROFILE_NAME_BYTES = 16;

/// IR emitter/receiver calibration sets; a profile's ir_set indexes them.
constexpr uint8_t IR_SET_COUNT = 2;

struct BoardProfile
{
    char    name[PROFILE_NAME_BYTES]; ///< NUL-terminated, shown at boot
    uint8_t has_fan;                  ///< 1 if a suction fan is fitted
    uint8_t ir_set;                   ///< Which IR calibration set the board uses
    uint8_t reserved[2];              ///< Zero; keeps battery_divider aligned
    float   battery_divider;          ///< Battery volts per ADC-pin volt
};

// UNVERIFIED: the rev1 schematic's battery divider ratio. 2.0 is a
// placeholder until it is measured; the battery driver does not read it yet.
constexpr BoardProfile PROFILE_PRESETS[] = {
    {"mouse1", 1, 0, {0, 0}, 2.0f},
    {"mouse2", 0, 1, {0, 0}, 2.0f},
};
constexpr std::size_t PROFILE_PRESET_COUNT = sizeof PROFILE_PRESETS / sizeof PROFILE_PRESETS[0];
} // namespace board

#endif // CONFIG_BOARD_PROFILES_H
