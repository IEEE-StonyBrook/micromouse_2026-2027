// Run log: int16-packed samples with event bits in a fixed ring, decimated by
// time. make -C tests runs this at 500 Hz and again at 1 kHz.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>

#include "config/motion.h"
#include "control/run_log.h"
#include "doctest/doctest.h"

namespace
{
// A sample with every field set to a distinct in-range value.
RunLog::Sample typical()
{
    RunLog::Sample s;
    s.target_deg    = -91.23f;
    s.yaw_deg       = 179.37f;
    s.rot_error_deg = -3.21f;
    s.rot_volts     = 1.234f;
    s.hold_err_deg  = -0.57f;
    s.wall_degps    = 12.3f;
    s.hold_degps    = -45.6f;
    s.fwd_mmps      = 1499.7f;
    s.left_mmps     = -812.3f;
    s.right_mmps    = 2999.2f;
    s.left_volts    = -5.987f;
    s.right_volts   = 5.999f;
    s.left_tof_mm   = 87.6f;
    s.front_tof_mm  = 1234.4f;
    s.right_tof_mm  = 64.2f;
    s.pos_mm        = 2879.9f;
    s.enc_l_mm      = 2875.4f;
    s.enc_r_mm      = -31002.6f;
    s.events        = RunLog::ENCODER_ROTATION | RunLog::WALL_FRONT;
    return s;
}

// Half a step: the most a round trip may move a value that is in range.
void checkRoundTrip(const RunLog::Sample& in)
{
    const RunLog::Sample out = RunLog::unpack(RunLog::pack(in));
    CHECK(std::fabs(out.target_deg - in.target_deg) <= 0.01f + 1e-4f); // 0.02 deg steps
    CHECK(std::fabs(out.yaw_deg - in.yaw_deg) <= 0.005f + 1e-4f);      // 0.01 deg
    CHECK(std::fabs(out.rot_error_deg - in.rot_error_deg) <= 0.005f + 1e-4f);
    CHECK(std::fabs(out.rot_volts - in.rot_volts) <= 0.0005f + 1e-5f); // 1 mV
    CHECK(std::fabs(out.hold_err_deg - in.hold_err_deg) <= 0.005f + 1e-4f);
    CHECK(std::fabs(out.wall_degps - in.wall_degps) <= 0.05f + 1e-3f); // 0.1 deg/s
    CHECK(std::fabs(out.hold_degps - in.hold_degps) <= 0.05f + 1e-3f);
    CHECK(std::fabs(out.fwd_mmps - in.fwd_mmps) <= 0.25f + 1e-3f); // 0.5 mm/s
    CHECK(std::fabs(out.left_mmps - in.left_mmps) <= 0.25f + 1e-3f);
    CHECK(std::fabs(out.right_mmps - in.right_mmps) <= 0.25f + 1e-3f);
    CHECK(std::fabs(out.left_volts - in.left_volts) <= 0.0005f + 1e-5f);
    CHECK(std::fabs(out.right_volts - in.right_volts) <= 0.0005f + 1e-5f);
    CHECK(std::fabs(out.left_tof_mm - in.left_tof_mm) <= 0.25f + 1e-3f); // 0.5 mm
    CHECK(std::fabs(out.front_tof_mm - in.front_tof_mm) <= 0.25f + 1e-3f);
    CHECK(std::fabs(out.right_tof_mm - in.right_tof_mm) <= 0.25f + 1e-3f);
    CHECK(std::fabs(out.pos_mm - in.pos_mm) <= 0.1f + 1e-3f);     // 0.2 mm
    CHECK(std::fabs(out.enc_l_mm - in.enc_l_mm) <= 0.5f + 1e-3f); // 1 mm
    CHECK(std::fabs(out.enc_r_mm - in.enc_r_mm) <= 0.5f + 1e-3f);
    CHECK(out.events == in.events);
}

// The log is ~56 KB, so tests keep it on the heap.
std::unique_ptr<RunLog> freshLog()
{
    return std::make_unique<RunLog>();
}

// Records one sample whose pos_mm tags it with its index.
void recordTagged(RunLog& log, int index)
{
    RunLog::Sample s;
    s.pos_mm = static_cast<float>(index);
    log.record(s);
}
} // namespace

TEST_CASE("run log: a packed sample is 18 int16 fields and an event word")
{
    CHECK(sizeof(RunLog::Packed) == 38);
}

TEST_CASE("run log: holds over 13 s at 8 ms inside the STM32's 64 KB CCM")
{
    CHECK(RunLog::DEFAULT_PERIOD_MS == 8);
    CHECK(RunLog::CAPACITY * RunLog::DEFAULT_PERIOD_MS >= 13000);
    CHECK(sizeof(RunLog) <= 64u * 1024u); // the log is all that lives in CCM
}

TEST_CASE("run log: every field round-trips within half a step")
{
    checkRoundTrip(typical());
    checkRoundTrip(RunLog::Sample{}); // all zero
}

TEST_CASE("run log: worked values unpack to the nearest step")
{
    RunLog::Sample in;
    in.yaw_deg               = 12.344f; // 0.01 deg steps: 1234
    in.target_deg            = 90.0f;   // 0.02 deg steps: 4500
    in.left_volts            = 6.0f;    // 1 mV steps: 6000
    in.left_mmps             = 300.2f;  // 0.5 mm/s steps: 600
    in.pos_mm                = 180.07f; // 0.2 mm steps: 900
    in.front_tof_mm          = 54.3f;   // 0.5 mm steps: 109
    in.enc_l_mm              = 1234.6f; // 1 mm steps: 1235
    in.enc_r_mm              = -0.4f;   // 1 mm steps: 0
    const RunLog::Packed p   = RunLog::pack(in);
    const RunLog::Sample out = RunLog::unpack(p);
    CHECK(out.yaw_deg == doctest::Approx(12.34f));
    CHECK(out.target_deg == doctest::Approx(90.0f));
    CHECK(out.left_volts == doctest::Approx(6.0f));
    CHECK(out.left_mmps == doctest::Approx(300.0f));
    CHECK(out.pos_mm == doctest::Approx(180.0f));
    CHECK(out.front_tof_mm == doctest::Approx(54.5f));
    CHECK(out.enc_l_mm == doctest::Approx(1235.0f));
    CHECK(out.enc_r_mm == 0.0f);
    CHECK((p.events & RunLog::CLIPPED) == 0);
}

TEST_CASE("run log: out-of-range values saturate and flag the sample")
{
    RunLog::Sample in;
    in.yaw_deg               = 1000.0f;  // past the +-327.67 deg range
    in.pos_mm                = -9000.0f; // past the +-6553.4 mm range
    in.left_mmps             = 1.0e9f;
    in.enc_r_mm              = 40000.0f; // past the +-32767 mm range
    const RunLog::Sample out = RunLog::unpack(RunLog::pack(in));
    CHECK(out.enc_r_mm == doctest::Approx(32767.0f));
    CHECK(out.yaw_deg == doctest::Approx(327.67f));
    CHECK(out.pos_mm == doctest::Approx(-6553.4f));
    CHECK(out.left_mmps == doctest::Approx(16383.5f));
    CHECK((out.events & RunLog::CLIPPED) != 0);
    // Everything else stays exact and the caller's events survive.
    in.events = RunLog::WALL_LEFT;
    CHECK(RunLog::unpack(RunLog::pack(in)).events == (RunLog::WALL_LEFT | RunLog::CLIPPED));
}

TEST_CASE("run log: a NaN field stays NaN instead of reading as a value")
{
    RunLog::Sample in;
    in.front_tof_mm          = std::numeric_limits<float>::quiet_NaN();
    const RunLog::Sample out = RunLog::unpack(RunLog::pack(in));
    CHECK(std::isnan(out.front_tof_mm));
    CHECK((out.events & RunLog::CLIPPED) == 0);
    CHECK(out.left_tof_mm == 0.0f);
}

TEST_CASE("run log: event bits round-trip one at a time")
{
    for (uint16_t bit :
         {RunLog::ENCODER_ROTATION, RunLog::WALL_LEFT, RunLog::WALL_FRONT, RunLog::WALL_RIGHT,
          RunLog::VOLTS_SATURATED, RunLog::FORWARD_ACTIVE, RunLog::ROTATION_ACTIVE})
    {
        RunLog::Sample in;
        in.events = bit;
        CHECK(RunLog::unpack(RunLog::pack(in)).events == bit);
    }
}

TEST_CASE("run log: a run past the capacity keeps its last samples, oldest first")
{
    auto log = freshLog();
    log->start();
    const int extra = 25;
    for (int i = 0; i < RunLog::CAPACITY + extra; ++i)
        recordTagged(*log, i);
    CHECK(log->count() == RunLog::CAPACITY);
    CHECK(log->sample(0).pos_mm == doctest::Approx(extra));
    CHECK(log->sample(1).pos_mm == doctest::Approx(extra + 1.0f));
    CHECK(log->sample(100).pos_mm == doctest::Approx(extra + 100.0f));
}

TEST_CASE("run log: a short run dumps in recording order")
{
    auto log = freshLog();
    log->start();
    for (int i = 0; i < 5; ++i)
        recordTagged(*log, i * 10);
    CHECK(log->count() == 5);
    for (int i = 0; i < 5; ++i)
        CHECK(log->sample(static_cast<uint16_t>(i)).pos_mm == doctest::Approx(i * 10.0f));
}

TEST_CASE("run log: start clears the previous run, stop freezes it")
{
    auto log = freshLog();
    log->start();
    recordTagged(*log, 1);
    recordTagged(*log, 2);
    log->stop();
    CHECK_FALSE(log->active());
    CHECK_FALSE(log->due()); // a stopped log never asks for a sample
    CHECK(log->count() == 2);
    log->start();
    CHECK(log->active());
    CHECK(log->count() == 0);
}

TEST_CASE("run log: decimation is by time, the same at 500 Hz and 1 kHz")
{
    auto log = freshLog();
    log->start(); // 8 ms
    CHECK(log->dtMs() == 8);
    int due = 0;
    for (int i = 0; i < LOOP_TICKS(0.080f); ++i)
        if (log->due())
            ++due;
    CHECK(due == 10); // 80 ms of 8 ms samples

    log->start(20);
    CHECK(log->dtMs() == 20);
    due = 0;
    for (int i = 0; i < LOOP_TICKS(1.0f); ++i)
        if (log->due())
            ++due;
    CHECK(due == 50);

    // The first sample lands one period after start, not on the first tick.
    log->start();
    int first = 0;
    while (!log->due())
        ++first;
    CHECK(first + 1 == LOOP_TICKS(0.008f));
}

TEST_CASE("run log: a period rounds to the nearest whole tick")
{
    // A period of exactly half a tick more (an odd ms at 500 Hz) may round
    // either way: LOOP_TICKS rounds in float, where 0.251 s is not exact.
    auto      log = freshLog();
    const int hz  = static_cast<int>(LOOP_FREQUENCY_HZ);
    for (int period_ms = 1; period_ms <= 1000; ++period_ms)
    {
        const int  twice   = 2 * period_ms * hz / 1000; // ticks x2, floored
        const bool tie     = (2 * period_ms * hz) % 1000 == 0 && twice % 2 == 1;
        const int  nearest = std::max(1, (period_ms * hz + 500) / 1000);
        log->start(static_cast<uint16_t>(period_ms));
        int ticks = 1;
        while (!log->due())
            ++ticks;
        CAPTURE(period_ms);
        if (tie)
            CHECK((ticks == nearest || ticks == std::max(1, nearest - 1)));
        else
            CHECK(ticks == nearest);
    }
}

TEST_CASE("run log: a period shorter than a tick samples every tick")
{
    auto log = freshLog();
    log->start(0);
    CHECK(log->dtMs() == static_cast<uint16_t>(1000.0f / LOOP_FREQUENCY_HZ));
    CHECK(log->due());
    CHECK(log->due());
}

TEST_CASE("run log: the encoder distance columns sit after pos_mm, before ev")
{
    const std::string cols = RunLog::COLUMNS;
    CHECK(cols.find(",pos_mm,enc_l_mm,enc_r_mm,ev") != std::string::npos);
    CHECK(cols.substr(cols.size() - 3) == ",ev");
}

TEST_CASE("run log: a dump row keeps the trace CSV columns and adds the events")
{
    RunLog::Sample s = typical();
    s.events         = RunLog::ENCODER_ROTATION | RunLog::WALL_FRONT;
    char row[RunLog::ROW_BYTES];
    RunLog::formatRow(row, sizeof row, 3, 8, s);
    CHECK(std::string(row) == "T,24,-91.2,179.4,-3.21,1.23,-0.57,12.3,-45.6,1500,-812,2999,-5.99,"
                              "6.00,1,88,1234,64,2879.9,2875,-31003,5");
}
