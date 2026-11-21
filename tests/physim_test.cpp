// Physics sim: the real control stack against the DC-motor, slipping-tyre
// plant in sim/. Nominal moves must land; each knob must move the outcome.
// make -C tests runs these at 500 Hz and again at 1 kHz.
#include <cmath>

#include "config/geometry.h"
#include "doctest/doctest.h"
#include "sim/physim.h"

namespace
{
constexpr float TEN_CELLS_MM = 10.0f * CELL_SIZE_MM;

physim::Result straight(const physim::Params& p)
{
    return physim::run(physim::Scenario::kStraight10, p);
}
} // namespace

TEST_CASE("physim: nominal 10-cell straight ends within 3 mm")
{
    const physim::Result r = straight(physim::nominal());
    REQUIRE(r.finished);
    CHECK(std::fabs(r.distance_mm - TEN_CELLS_MM) < 3.0f);
    CHECK(std::fabs(r.yaw_deg) < 1.0f);
}

TEST_CASE("physim: nominal smooth 90 ends within 1 deg")
{
    const physim::Result r = physim::run(physim::Scenario::kSmooth90, physim::nominal());
    REQUIRE(r.finished);
    CHECK(std::fabs(r.yaw_deg - 90.0f) < 1.0f);
}

// The plant carries the kS friction the feedforward cancels. Without it the
// spin overran by ~3 deg: kS over-drove the wheels, the rotation PD held
// ~-1.7 deg of error against it until the profile ended and cut the motors,
// and the frictionless gearbox coasted on.
TEST_CASE("physim: nominal in-place 90 ends within 1 deg")
{
    const physim::Result r = physim::run(physim::Scenario::kInPlace90, physim::nominal());
    REQUIRE(r.finished);
    CHECK(std::fabs(r.distance_mm) < 3.0f);
    CHECK(std::fabs(r.yaw_deg - 90.0f) < 1.0f);
}

TEST_CASE("physim: the next mouse's 62.6 counts/mm encoder still lands the straight")
{
    physim::Params p       = physim::nominal();
    p.counts_per_mm_left   = physim::NEW_MOUSE_COUNTS_PER_MM;
    p.counts_per_mm_right  = physim::NEW_MOUSE_COUNTS_PER_MM;
    const physim::Result r = straight(p);
    CHECK(std::fabs(r.distance_mm - TEN_CELLS_MM) < 3.0f);
}

TEST_CASE("physim: a fixed seed reproduces a run exactly; another seed does not")
{
    physim::Params p        = physim::nominal();
    p.gyro_noise_degps_rthz = 1.0f;
    p.slip.rate_hz          = 5.0f;
    const physim::Result a  = straight(p);
    const physim::Result b  = straight(p);
    CHECK(a.distance_mm == b.distance_mm);
    CHECK(a.yaw_deg == b.yaw_deg);
    CHECK(a.imu_yaw_deg == b.imu_yaw_deg);

    p.seed                 = 2;
    const physim::Result c = straight(p);
    CHECK(c.imu_yaw_deg != a.imu_yaw_deg);
}

TEST_CASE("physim: low traction on one wheel throws the straight off heading")
{
    // 0.2 g of grip on the left is below the 2500 mm/s^2 braking: the left
    // wheel cannot stop its side, which overruns and swings the robot CW.
    const physim::Result nominal = straight(physim::nominal());
    physim::Params       p       = physim::nominal();
    p.mu_left                    = 0.2f;
    const physim::Result r       = straight(p);
    CHECK(r.yaw_deg < nominal.yaw_deg - 20.0f);
}

TEST_CASE("physim: random slip events cost straight-line distance accuracy")
{
    // Grip gone for 50 ms, ten times a second per wheel: the encoders count
    // wheel turns the body never makes (or misses the body sliding on).
    const physim::Result nominal = straight(physim::nominal());
    physim::Params       p       = physim::nominal();
    p.slip.rate_hz               = 10.0f;
    p.slip.duration_s            = 0.05f;
    p.slip.traction_scale        = 0.0f;
    const physim::Result r       = straight(p);
    REQUIRE(r.finished);
    CHECK(std::fabs(r.distance_mm - nominal.distance_mm) > 3.0f);
}

TEST_CASE("physim: a coarse encoder costs straight-line distance accuracy")
{
    const physim::Result nominal = straight(physim::nominal());
    physim::Params       p       = physim::nominal();
    // 10 mm per count, ~30x coarser than today. At 5 mm the kS friction stops
    // the wheels close enough to a count edge that the 1 kHz run landed 2.4 mm off.
    p.counts_per_mm_left   = 0.1f;
    p.counts_per_mm_right  = 0.1f;
    const physim::Result r = straight(p);
    CHECK(std::fabs(r.distance_mm - TEN_CELLS_MM) >
          std::fabs(nominal.distance_mm - TEN_CELLS_MM) + 3.0f);
}

TEST_CASE("physim: gyro bias drags the held heading by about bias x time")
{
    // Heading hold steers the IMU yaw to 0, so the true heading drifts the
    // other way at the bias rate: 2 deg/s over the ~2.2 s move is ~-4 deg.
    physim::Params p       = physim::nominal();
    p.gyro_bias_degps      = 2.0f;
    const physim::Result r = straight(p);
    CHECK(r.yaw_deg < -2.5f);
    CHECK(r.yaw_deg > -6.0f);
    CHECK(r.imu_yaw_deg > 1.0f);
}

TEST_CASE("physim: gyro noise lets the held heading wander from the truth")
{
    const physim::Result nominal = straight(physim::nominal());
    CHECK(std::fabs(nominal.imu_yaw_deg - nominal.yaw_deg) < 0.05f);

    physim::Params p        = physim::nominal();
    p.gyro_noise_degps_rthz = 1.0f;
    const physim::Result r  = straight(p);
    CHECK(std::fabs(r.imu_yaw_deg - r.yaw_deg) > 0.5f);
    CHECK(r.yaw_deg != nominal.yaw_deg);
}

TEST_CASE("physim: a seam over-reads the encoders and logs a pitch and z-accel spike")
{
    const physim::Result nominal = straight(physim::nominal());
    CHECK(nominal.seam_crossings == 0);
    CHECK(nominal.peak_pitch_rate_degps == 0.0f);

    physim::Params p             = physim::nominal();
    p.seam.enabled               = true;
    p.seam.at_mm                 = 900.0f;
    p.seam.encoder_extra_mm      = 3.0f;
    p.seam.pitch_rate_peak_degps = 400.0f;
    p.seam.accel_z_peak_mps2     = 30.0f;
    const physim::Result r       = straight(p);
    REQUIRE(r.finished);
    CHECK(r.seam_crossings == 2);
    // On a seam that loses grip, the 3 mm jump over the 20 ms crossing reads as
    // a 150 mm/s overrun: the forward PD brakes, the tyres skid and the body
    // slides on, handing back most of the over-read. It still stops short.
    CHECK(r.distance_mm < nominal.distance_mm - 0.5f);

    // Keep the grip and the over-read alone shows: the encoders count ~3 mm
    // the body never travels, so it stops ~3 mm short (2.7-2.8 measured).
    p.seam.traction_scale     = 1.0f;
    const physim::Result grip = straight(p);
    CHECK(grip.distance_mm < nominal.distance_mm - 2.0f);
    CHECK(grip.distance_mm > nominal.distance_mm - 3.5f);
    CHECK(grip.distance_mm < r.distance_mm - 1.0f);
    // Sampled once per tick, so the peaks land near, not on, the set values.
    CHECK(r.peak_pitch_rate_degps > 300.0f);
    CHECK(r.peak_pitch_rate_degps <= 400.0f);
    CHECK(r.peak_accel_z_mps2 > 25.0f);
    CHECK(r.peak_accel_z_mps2 <= 30.0f);
}
