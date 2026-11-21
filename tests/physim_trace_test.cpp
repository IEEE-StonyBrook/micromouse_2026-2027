// Physics sim traces: the firmware's run log wired into the rig, the Pico's
// PATH command reproduced on an open floor, and open-loop volts replay of a
// logged run. make -C tests runs these at 500 Hz and again at 1 kHz.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "config/geometry.h"
#include "config/motion.h"
#include "control/run_log.h"
#include "doctest/doctest.h"
#include "sim/physim.h"

namespace
{
std::vector<std::string> split(const std::string& s, char sep)
{
    std::vector<std::string> out;
    size_t                   start = 0;
    for (size_t i = 0; i <= s.size(); i++)
        if (i == s.size() || s[i] == sep)
        {
            out.push_back(s.substr(start, i - start));
            start = i + 1;
        }
    return out;
}

std::string join(const std::vector<std::string>& lines)
{
    std::string out;
    for (const std::string& l : lines)
        out += l + "\n";
    return out;
}
} // namespace

TEST_CASE("physim trace: a scenario dumps the firmware's TRACE format plus truth columns")
{
    std::vector<std::string> trace;
    physim::RunOptions       opt;
    opt.trace              = &trace;
    const physim::Result r = physim::run(physim::Scenario::kStraight10, physim::nominal(), opt);
    REQUIRE(r.finished);
    REQUIRE(trace.size() > 3);

    const std::string& header = trace.front();
    CHECK(header.rfind("TRACE n=", 0) == 0);
    CHECK(header.find(" dt_ms=8 ") != std::string::npos);
    CHECK(header.find(",pos_mm,") != std::string::npos);
    CHECK(header.find(",true_dist_mm,true_yaw_deg,ev ") != std::string::npos);
    CHECK(header.find(" wrapped=0 ") != std::string::npos);
    CHECK(header.find(" cmd=\"") != std::string::npos);
    CHECK(trace.back() == "TRACE end");

    // One row per sample, one value per column.
    const size_t cols_at = header.find("cols=") + 5;
    const auto   cols    = split(header.substr(cols_at, header.find(' ', cols_at) - cols_at), ',');
    const size_t rows    = trace.size() - 2;
    CHECK(header.find("n=" + std::to_string(rows) + " ") != std::string::npos);
    const auto last = split(trace[trace.size() - 2].substr(2), ',');
    REQUIRE(last.size() == cols.size());

    // Motion samples while it drives: the ~2.2 s move in 8 ms samples. The
    // last truth matches the plant, which stands still after it.
    CHECK(rows > 250);
    CHECK(std::stof(last[cols.size() - 3]) == doctest::Approx(r.distance_mm).epsilon(0.002));
}

TEST_CASE("physim trace: PATH arguments parse as the Pico's PATH command does")
{
    physim::PathCommand c;
    std::string         error;
    REQUIRE(physim::parsePathCommand("PATH FFRF2 L 600 3000 SMOOTH", &c, &error));
    CHECK(c.sequence == "FFRF2L");
    CHECK(c.speed_mmps == 600.0f);
    CHECK(c.accel_mmps2 == 3000.0f);
    CHECK(c.omega_degps == CLI_PATH_OMEGA_DEGPS);
    CHECK(c.smooth);
    CHECK_FALSE(c.raw);

    // The header's cmd= text round-trips.
    physim::PathCommand again;
    REQUIRE(physim::parsePathCommand(physim::describe(c), &again, &error));
    CHECK(again.sequence == c.sequence);
    CHECK(again.speed_mmps == 600.0f);
    CHECK(again.alpha_degps2 == CLI_PATH_ALPHA_DEGPS2);
    CHECK(again.smooth);

    CHECK_FALSE(physim::parsePathCommand("FXF", &c, &error));
    CHECK_FALSE(physim::parsePathCommand("F 600 9000", &c, &error)); // accel over 5000
    // One cell between two arcs is fine; an arc needs half a cell either side.
    REQUIRE(physim::parsePathCommand("FLFLF SMOOTH", &c, &error));

    // Trailing flags come off before numbers, as on the Pico: a final 1 or 0
    // is the smooth flag, not a parameter.
    REQUIRE(physim::parsePathCommand("FFRF 600 1", &c, &error));
    CHECK(c.smooth);
    CHECK(c.accel_mmps2 == CLI_PATH_ACCEL_MMPS2);
    REQUIRE(physim::parsePathCommand("FFRF 600 3000 0 RAW", &c, &error));
    CHECK_FALSE(c.smooth);
    CHECK(c.raw);
    CHECK_FALSE(physim::parsePathCommand("FF SMOOTH RF", &c, &error)); // flags only at the end
    CHECK_FALSE(physim::parsePathCommand("FF 600abc", &c, &error));
    CHECK_FALSE(physim::parsePathCommand("FF 600 3000 240 800 5", &c, &error));
}

TEST_CASE("physim trace: a run longer than the ring dumps its last samples, flagged wrapped")
{
    // 18 cells and a spin at 180 mm/s is ~21 s of samples, past the ring's 13.44 s.
    std::vector<std::string> trace;
    physim::RunOptions       opt;
    opt.trace = &trace;
    physim::PathCommand c;
    std::string         error;
    REQUIRE(physim::parsePathCommand("F9LF9", &c, &error));
    const physim::Result r = physim::runPath(c, physim::nominal(), opt);
    REQUIRE(r.finished);
    CHECK(trace.front().find(" wrapped=1 ") != std::string::npos);
    CHECK(trace.size() == RunLog::CAPACITY + 2u);
    // Truth stays aligned to the kept samples: the last one is the plant at
    // rest, the first is ~13 s (at most 180 mm/s) before it.
    const auto  first  = split(trace[1], ',');
    const auto  last   = split(trace[trace.size() - 2], ',');
    const float end_mm = std::stof(last[last.size() - 3]);
    CHECK(end_mm == doctest::Approx(r.distance_mm).epsilon(0.002));
    const float span_mm = end_mm - std::stof(first[first.size() - 3]);
    CHECK(span_mm > 1500.0f);
    CHECK(span_mm < 13.44f * 181.0f);
}

TEST_CASE("physim trace: parseTrace picks a dump by index and keeps a cut row's slot")
{
    const std::string cols = "t_ms,left_mmps,right_mmps,left_v,right_v,ev";
    const std::string text = "[ 1.0] [INFO] TRACE n=1 dt_ms=8 cols=" + cols +
                             "\n[ 2.0] [INFO] T,0,0,0,1.0,1.0,0\nTRACE end\n"
                             "TRACE n=3 dt_ms=4 cols=" +
                             cols +
                             " batt_v=7.50 wrapped=1 cmd=\"PATH F\"\n"
                             "T,0,0,0,2.0,2.0,0\nT,4,0,0,2.\nT,8,5,5,3.0,3.0,0\nTRACE end\n";
    physim::TraceRun  run;
    std::string       error;
    REQUIRE(physim::parseTrace(text, 1, &run, &error));
    CHECK(run.dt_ms == 4);
    CHECK(run.batt_v == doctest::Approx(7.5f));
    CHECK(run.wrapped);
    CHECK(run.cmd == "PATH F");
    REQUIRE(run.rows.size() == 3);
    CHECK(std::isnan(run.rows[1][0]));
    CHECK(run.rows[2][run.column("left_v")] == 3.0f);
    REQUIRE(physim::parseTrace(text, 0, &run, &error));
    CHECK(std::isnan(run.batt_v));
    CHECK_FALSE(run.wrapped);
    CHECK_FALSE(physim::parseTrace(text, 2, &run, &error));
}

TEST_CASE("physim trace: PATH FFRFFLF ends on the expected cell, spun and smooth")
{
    for (const char* args : {"FFRFFLF", "FFRFFLF 300 1500 SMOOTH"})
    {
        CAPTURE(args);
        physim::PathCommand c;
        std::string         error;
        REQUIRE(physim::parsePathCommand(args, &c, &error));
        const physim::Result r = physim::runPath(c, physim::nominal());
        REQUIRE(r.finished);
        // From the first cell's centre facing +x: 2 cells +x, 2 cells -y, 1 cell +x.
        CHECK(r.x_mm == doctest::Approx(3.0f * CELL_SIZE_MM).epsilon(0.03));
        CHECK(r.y_mm == doctest::Approx(-2.0f * CELL_SIZE_MM).epsilon(0.05));
        CHECK(std::fabs(r.yaw_deg) < 4.0f);
    }
}

TEST_CASE("physim trace: volts replay of a logged run tracks the same plant, not another")
{
    std::vector<std::string> trace;
    physim::RunOptions       opt;
    opt.trace = &trace;
    physim::PathCommand c;
    std::string         error;
    REQUIRE(physim::parsePathCommand("FFRF", &c, &error));
    REQUIRE(physim::runPath(c, physim::nominal(), opt).finished);

    physim::TraceRun logged;
    REQUIRE(physim::parseTrace("[  12.0] [INFO] " + join(trace), 0, &logged, &error));
    CHECK(logged.dt_ms == 8);
    CHECK(logged.batt_v == doctest::Approx(physim::nominal().battery_v).epsilon(0.01));
    CHECK(logged.rows.size() == trace.size() - 2);

    std::vector<std::string>   replayed;
    const physim::ReplayResult same = physim::replayVolts(logged, physim::nominal(), &replayed);
    CHECK(same.samples == static_cast<int>(logged.rows.size()));
    CHECK(replayed.size() == trace.size());
    // Volts held for a whole sample instead of tick by tick: close, not exact.
    CHECK(same.rms_left_mmps < 15.0f);
    CHECK(same.rms_right_mmps < 15.0f);

    physim::Params sticky            = physim::nominal();
    sticky.friction_v_left           = 0.6f;
    sticky.friction_v_right          = 0.6f;
    const physim::ReplayResult other = physim::replayVolts(logged, sticky, nullptr);
    CHECK(other.rms_left_mmps > 2.0f * same.rms_left_mmps);
}
