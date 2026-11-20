#ifndef COMMON_FLASH_STORE_H
#define COMMON_FLASH_STORE_H

#include <cstddef>
#include <cstdint>

#include "hal/flash.h"

/**
 * @file flash_store.h
 * @brief Settings kept in flash as two A/B slots, so a save cut off by a
 *        power loss never costs the last good copy.
 *
 * Each slot is one erase sector (sectors 1 and 2 on the STM32F405) holding
 * at most one record, in 32-bit words:
 *
 *   word 0  magic     RECORD_MAGIC
 *   word 1  version   the caller's settings layout version
 *   word 2  seq       save counter, compared with 32-bit wrap-around: of two
 *                     valid records, the one ahead of the other is newest
 *   word 3  len       payload length in bytes
 *   word 4  uid_hash  hash of the chip UID that wrote it
 *   word 5  payload   len bytes, zero-padded to a whole word
 *   last    crc32     CRC-32 of the header words and the len payload bytes
 *
 * A save erases the slot that does not hold the newest valid record and
 * writes the new record there, CRC last, so until the CRC word lands the old
 * record is still the newest valid one. A load takes the valid record with
 * the highest seq; a record with the wrong magic, version, uid_hash, length
 * or CRC is skipped. If none is valid the caller keeps its defaults.
 */

/// Why a save was refused. Erasing a sector stalls the flash bus, and with it
/// every interrupt running from flash, for hundreds of ms, so the mouse must
/// not be relying on its control loop or driving any output when it saves.
enum class SaveBlocker : uint8_t
{
    kNone,
    kArmed,
    kMotorsAwake,
    kFanOn,
    kEmittersOn,
};

/// What the app reports about its outputs, once it has tried to turn them off.
struct OutputState
{
    bool armed;
    bool motors_awake;
    bool fan_on;
    bool emitters_on;
};

/// The first reason the outputs forbid a save; armed comes first.
SaveBlocker saveBlocker(const OutputState& outputs);

/**
 * @brief The precondition the store checks before every save.
 *
 * The app implements it: refuse if armed, otherwise force the motor drivers
 * asleep, the fan PWM to 0 and the IR emitters off, then report anything
 * still on (saveBlocker() turns an OutputState into the verdict).
 */
class SaveGuard
{
  public:
    virtual SaveBlocker prepareForSave() = 0;

  protected:
    ~SaveGuard() = default;
};

enum class LoadStatus : uint8_t
{
    kOk,            ///< Newest record loaded; the other slot is valid or erased
    kEmpty,         ///< Both slots erased (first boot): defaults kept
    kFellBack,      ///< A slot is damaged (a corrupt record, or a save or
                    ///< erase cut off part way); loaded the other one
    kNoValidRecord, ///< No usable record, or the newest one is larger than
                    ///< the caller's buffer: defaults kept
};

struct LoadResult
{
    LoadStatus  status;
    uint32_t    seq; ///< Of the loaded record, 0 if none
    std::size_t len; ///< Bytes copied into the caller's buffer
};

enum class SaveStatus : uint8_t
{
    kOk,
    kRefused,    ///< The SaveGuard said no; flash untouched
    kTooLarge,   ///< Payload exceeds maxPayloadBytes(); flash untouched
    kFlashError, ///< Erase, program or read-back failed; the old record still loads
};

struct SaveResult
{
    SaveStatus  status;
    SaveBlocker blocker; ///< Set when status is kRefused
    uint32_t    seq;     ///< Of the record written
};

/// LED code for a load: 2 flashes when it fell back to the other slot, 4
/// when no record loaded, 0 when nothing is wrong.
uint8_t ledFlashes(LoadStatus status);

/// Whether the LED is lit at uptime_ms while showing a code of `flashes`
/// 100 ms blinks, 100 ms apart, repeating every 2 s.
constexpr bool ledCodeOn(uint32_t uptime_ms, uint8_t flashes)
{
    const uint32_t phase_ms = uptime_ms % 2000;
    return phase_ms < 200u * flashes && (phase_ms / 100) % 2 == 0;
}

class FlashStore
{
  public:
    static constexpr uint32_t RECORD_MAGIC = 0x4A425331u; // "JBS1"

    /// Uses sectors 0 and 1 of flash. version and uid_hash must match for a
    /// record to load, and are written into every record saved.
    FlashStore(hal::FlashDevice& flash, SaveGuard& guard, uint32_t version, uint32_t uid_hash);

    /// Copies the newest valid record into payload. Leaves payload untouched
    /// unless a record of at most capacity_bytes loads.
    LoadResult load(void* payload, std::size_t capacity_bytes) const;

    /// Checks the guard, then writes a new record over the older slot.
    SaveResult save(const void* payload, std::size_t len);

    std::size_t maxPayloadBytes() const;

  private:
    struct SlotInfo
    {
        bool     valid;
        bool     blank;
        uint32_t seq;
        uint32_t len;
    };

    /// At least two sectors, each with room for a header and a CRC. A
    /// device that fails this never loads (kNoValidRecord) or saves (kFlashError).
    bool     usable() const;
    SlotInfo inspect(uint32_t slot) const;
    bool     sectorErased(uint32_t base) const;
    /// Index of the newest valid slot, or -1.
    int newest(const SlotInfo (&slots)[2]) const;

    hal::FlashDevice& flash_;
    SaveGuard&        guard_;
    uint32_t          version_;
    uint32_t          uid_hash_;
};

#endif // COMMON_FLASH_STORE_H
