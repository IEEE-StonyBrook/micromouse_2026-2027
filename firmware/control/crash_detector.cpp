#include "control/crash_detector.h"

#include <algorithm>
#include <cmath>

#include "config/config.h"

namespace
{
constexpr int kSaturationTicks = LOOP_TICKS(CRASH_SATURATION_TIME_S);
constexpr int kStallTicks      = LOOP_TICKS(CRASH_STALL_TIME_S);
constexpr int kRunawayTicks    = LOOP_TICKS(CRASH_RUNAWAY_TIME_S);

bool stalled(float amps, float mmps)
{
    return std::fabs(amps) >= CRASH_STALL_AMPS && std::fabs(mmps) < CRASH_STALL_SPEED_MMPS;
}
} // namespace

const char* crashCauseName(CrashCause cause)
{
    switch (cause)
    {
        case CrashCause::kSaturation:
            return "SATURATION";
        case CrashCause::kStall:
            return "STALL";
        case CrashCause::kRunaway:
            return "RUNAWAY";
        default:
            return "NONE";
    }
}

CrashDetector::CrashDetector(bool enabled) : enabled_(enabled)
{
}

void CrashDetector::resetWindows()
{
    saturation_ticks_ = 0;
    stall_ticks_      = 0;
    runaway_ticks_    = 0;
    runaway_start_mm_ = 0.0f;
}

void CrashDetector::trip(CrashCause cause)
{
    cause_.store(static_cast<uint8_t>(cause));
    tripped_.store(true);
    trip_count_.store(trip_count_.load() + 1); // only the control tick writes it
    resetWindows();
}

bool CrashDetector::update(const CrashInputs& in)
{
    // Load then store, not exchange: the RP2040 has no atomic read-modify-write.
    // A clear() landing between the two is already satisfied by this one.
    if (clear_request_.load())
    {
        clear_request_.store(false);
        tripped_.store(false);
        cause_.store(static_cast<uint8_t>(CrashCause::kNone));
        resetWindows();
    }
    if (tripped_.load())
        return false;
    if (!enabled_.load() || inhibit_.load())
    {
        resetWindows();
        return false;
    }

    // What the motor can actually get: the clamp, or less on a sagging battery.
    const float limit_v =
        CRASH_SATURATION_FRACTION * std::min(static_cast<float>(MAX_VOLTAGE), in.battery_volts);
    const bool saturated =
        std::fabs(in.left_volts) >= limit_v || std::fabs(in.right_volts) >= limit_v;
    saturation_ticks_ = saturated ? saturation_ticks_ + 1 : 0;
    if (saturation_ticks_ >= kSaturationTicks)
    {
        trip(CrashCause::kSaturation);
        return true;
    }

    const bool stall = in.has_current && (stalled(in.left_amps, in.left_mmps) ||
                                          stalled(in.right_amps, in.right_mmps));
    stall_ticks_     = stall ? stall_ticks_ + 1 : 0;
    if (stall_ticks_ >= kStallTicks)
    {
        trip(CrashCause::kStall);
        return true;
    }

    // Runaway: the error must be large for a whole window and larger at its
    // end than at its start. A large but steady error (a slow wheel the PD is
    // holding) restarts the window instead.
    const float error_mm = std::fabs(in.fwd_error_mm);
    if (error_mm < CRASH_RUNAWAY_ERROR_MM)
    {
        runaway_ticks_ = 0;
    }
    else
    {
        if (runaway_ticks_ == 0)
            runaway_start_mm_ = error_mm;
        if (++runaway_ticks_ >= kRunawayTicks)
        {
            if (error_mm - runaway_start_mm_ >= CRASH_RUNAWAY_GROWTH_MM)
            {
                trip(CrashCause::kRunaway);
                return true;
            }
            runaway_ticks_ = 0;
        }
    }
    return false;
}
