#include "control/run_log.h"

#include <cmath>
#include <cstdio>

#include "config/motion.h"
#include "hal/system.h"

namespace
{
constexpr int16_t PACKED_NAN = INT16_MIN;
constexpr float   PACKED_MAX = 32767.0f;

// Steps per unit of each packed field, in Sample order. Each range is
// +-32767 steps: deg at 0.01 (+-327 deg), target at 0.02 (+-655 deg, room
// for a turn past 180), volts at 1 mV, deg/s at 0.1, mm/s and ToF mm at 0.5,
// position at 0.2 mm (+-6.5 m, a full diagonal of a 16x16 maze), encoder
// distance at 1 mm (+-32.7 m, a full 13.4 s ring at 2.4 m/s).
constexpr float STEPS_PER_UNIT[RunLog::FIELDS] = {
    50.0f,   // target_deg
    100.0f,  // yaw_deg
    100.0f,  // rot_error_deg
    1000.0f, // rot_volts
    100.0f,  // hold_err_deg
    10.0f,   // wall_degps
    10.0f,   // hold_degps
    2.0f,    // fwd_mmps
    2.0f,    // left_mmps
    2.0f,    // right_mmps
    1000.0f, // left_volts
    1000.0f, // right_volts
    2.0f,    // left_tof_mm
    2.0f,    // front_tof_mm
    2.0f,    // right_tof_mm
    5.0f,    // pos_mm
    1.0f,    // enc_l_mm
    1.0f,    // enc_r_mm
};

// Rounds to the nearest step and clamps to the int16 range; sets *clipped
// when the value did not fit.
int16_t packField(float value, float steps_per_unit, bool* clipped)
{
    if (std::isnan(value))
        return PACKED_NAN;
    const float steps = value * steps_per_unit;
    if (steps > PACKED_MAX)
    {
        *clipped = true;
        return static_cast<int16_t>(PACKED_MAX);
    }
    if (steps < -PACKED_MAX)
    {
        *clipped = true;
        return static_cast<int16_t>(-PACKED_MAX);
    }
    return static_cast<int16_t>(steps >= 0.0f ? steps + 0.5f : steps - 0.5f);
}

float unpackField(int16_t steps, float steps_per_unit)
{
    if (steps == PACKED_NAN)
        return NAN;
    return static_cast<float>(steps) / steps_per_unit;
}

RunLog HAL_CCM(run_log_instance);
} // namespace

RunLog& runLog()
{
    return run_log_instance;
}

RunLog::Packed RunLog::pack(const Sample& s)
{
    const float values[FIELDS] = {
        s.target_deg,  s.yaw_deg,      s.rot_error_deg, s.rot_volts,  s.hold_err_deg, s.wall_degps,
        s.hold_degps,  s.fwd_mmps,     s.left_mmps,     s.right_mmps, s.left_volts,   s.right_volts,
        s.left_tof_mm, s.front_tof_mm, s.right_tof_mm,  s.pos_mm,     s.enc_l_mm,     s.enc_r_mm};
    Packed packed{};
    bool   clipped = false;
    for (int i = 0; i < FIELDS; ++i)
        packed.field[i] = packField(values[i], STEPS_PER_UNIT[i], &clipped);
    packed.events = static_cast<uint16_t>(s.events | (clipped ? CLIPPED : 0));
    return packed;
}

RunLog::Sample RunLog::unpack(const Packed& p)
{
    float v[FIELDS];
    for (int i = 0; i < FIELDS; ++i)
        v[i] = unpackField(p.field[i], STEPS_PER_UNIT[i]);
    Sample s;
    s.target_deg    = v[0];
    s.yaw_deg       = v[1];
    s.rot_error_deg = v[2];
    s.rot_volts     = v[3];
    s.hold_err_deg  = v[4];
    s.wall_degps    = v[5];
    s.hold_degps    = v[6];
    s.fwd_mmps      = v[7];
    s.left_mmps     = v[8];
    s.right_mmps    = v[9];
    s.left_volts    = v[10];
    s.right_volts   = v[11];
    s.left_tof_mm   = v[12];
    s.front_tof_mm  = v[13];
    s.right_tof_mm  = v[14];
    s.pos_mm        = v[15];
    s.enc_l_mm      = v[16];
    s.enc_r_mm      = v[17];
    s.events        = p.events;
    return s;
}

void RunLog::formatRow(char* out, size_t size, uint16_t index, uint16_t dt_ms, const Sample& s)
{
    std::snprintf(out, size,
                  "T,%d,%.1f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.0f,%.0f,%.0f,%.2f,%.2f,%d,%.0f,%.0f,"
                  "%.0f,%.1f,%.0f,%.0f,%u",
                  index * dt_ms, static_cast<double>(s.target_deg), static_cast<double>(s.yaw_deg),
                  static_cast<double>(s.rot_error_deg), static_cast<double>(s.rot_volts),
                  static_cast<double>(s.hold_err_deg), static_cast<double>(s.wall_degps),
                  static_cast<double>(s.hold_degps), static_cast<double>(s.fwd_mmps),
                  static_cast<double>(s.left_mmps), static_cast<double>(s.right_mmps),
                  static_cast<double>(s.left_volts), static_cast<double>(s.right_volts),
                  (s.events & ENCODER_ROTATION) ? 1 : 0, static_cast<double>(s.left_tof_mm),
                  static_cast<double>(s.front_tof_mm), static_cast<double>(s.right_tof_mm),
                  static_cast<double>(s.pos_mm), static_cast<double>(s.enc_l_mm),
                  static_cast<double>(s.enc_r_mm), static_cast<unsigned>(s.events));
}

void RunLog::formatHeader(char* out, size_t size) const
{
    std::snprintf(out, size,
                  "TRACE n=%u dt_ms=%u cols=%s run=%lu loop_hz=%d batt_v=%.2f wrapped=%d "
                  "cmd=\"%s\"",
                  static_cast<unsigned>(count()), static_cast<unsigned>(dtMs()), COLUMNS,
                  static_cast<unsigned long>(header_.run_id),
                  static_cast<int>(LOOP_FREQUENCY_HZ + 0.5f),
                  static_cast<double>(header_.battery_v), wrapped_ ? 1 : 0, header_.cmd);
}

void RunLog::set_header(const char* cmd, float battery_v)
{
    header_.run_id    = header_.run_id + 1;
    header_.battery_v = battery_v;
    size_t i          = 0;
    for (; cmd != nullptr && cmd[i] != '\0' && i + 1 < CMD_BYTES; ++i)
    {
        // Unsigned: char's sign differs between the host and Arm.
        const unsigned char c = static_cast<unsigned char>(cmd[i]);
        header_.cmd[i]        = c == '"' ? '\'' : (c < 0x20 || c == 0x7F) ? ' ' : cmd[i];
    }
    header_.cmd[i] = '\0';
    segment_       = 0;
}

void RunLog::start(uint16_t period_ms)
{
    active_         = false;
    wrapped_        = false;
    count_          = 0;
    next_           = 0;
    tick_           = 0;
    const int ticks = LOOP_TICKS(static_cast<float>(period_ms) / 1000.0f);
    period_ticks_   = static_cast<uint16_t>(ticks > 0 ? ticks : 1);
    active_         = true;
}

void RunLog::stop()
{
    active_ = false;
}

bool RunLog::due()
{
    if (!active_ || ++tick_ < period_ticks_)
        return false;
    tick_ = 0;
    return true;
}

void RunLog::record(const Sample& sample)
{
    if (!active_)
        return;
    Packed packed   = pack(sample);
    packed.events   = static_cast<uint16_t>(packed.events | (segment_ << SEGMENT_SHIFT));
    samples_[next_] = packed;
    next_           = static_cast<uint16_t>((next_ + 1) % CAPACITY);
    if (count_ < CAPACITY)
        count_ = count_ + 1;
    else
        wrapped_ = true;
}

uint16_t RunLog::dtMs() const
{
    return static_cast<uint16_t>(period_ticks_ * 1000.0f / LOOP_FREQUENCY_HZ + 0.5f);
}

RunLog::Sample RunLog::sample(uint16_t index) const
{
    return unpack(samples_[(next_ + CAPACITY - count_ + index) % CAPACITY]);
}
