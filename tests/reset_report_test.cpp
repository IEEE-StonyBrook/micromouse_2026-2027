// The next-boot reset verdict: the reset cause plus the breadcrumb the
// previous session left, turned into the lines the boot prints.
#include <cstring>
#include <string>

#include "common/reset_report.h"
#include "doctest/doctest.h"

namespace
{
constexpr float SAG_V = 6.4f;

hal::system::ResetCause none()
{
    return hal::system::ResetCause{};
}

Breadcrumb::Snapshot crumb(float v_min, const char* activity = "search run")
{
    Breadcrumb::Snapshot s{};
    s.valid        = true;
    s.uptime_ms    = 4200;
    s.voltage_min  = v_min;
    s.voltage_last = v_min;
    std::strncpy(s.last_log, activity, sizeof s.last_log - 1);
    return s;
}

std::string first(const ResetReport& r)
{
    return r.count > 0 ? r.lines[0].text : "";
}
} // namespace

TEST_CASE("A brownout with a breadcrumb names the brownout, the sag and the activity")
{
    hal::system::ResetCause c = none();
    c.brownout                = true;
    const ResetReport r       = resetReport(c, crumb(5.9f), SAG_V);
    REQUIRE(r.count == 2);
    CHECK(r.lines[0].severity == ResetSeverity::kError);
    CHECK(first(r) == "POWER FAULT (brownout): the battery rail collapsed under load. It fell to "
                      "5.90 V after 4200 ms, during: search run");
}

TEST_CASE("A brownout is reported even when no breadcrumb survived")
{
    hal::system::ResetCause c = none();
    c.brownout                = true;
    const ResetReport r       = resetReport(c, Breadcrumb::Snapshot{}, SAG_V);
    REQUIRE(r.count >= 1);
    CHECK(r.lines[0].severity == ResetSeverity::kError);
    CHECK(first(r).find("brownout") != std::string::npos);
}

TEST_CASE("A watchdog timeout says the main loop hung and what it was doing")
{
    hal::system::ResetCause c = none();
    c.watchdog_timeout        = true;
    c.run_pin                 = true; // a decoder may leave it set; the watchdog wins
    const ResetReport r       = resetReport(c, crumb(0.0f, "profile save"), SAG_V);
    REQUIRE(r.count >= 1);
    CHECK(r.lines[0].severity == ResetSeverity::kError);
    CHECK(first(r) == "WATCHDOG RESET: the main loop hung after 4200 ms, during: profile save");
}

TEST_CASE("A power-up with no breadcrumb is a cold boot, not a fault")
{
    hal::system::ResetCause c = none();
    c.power_on                = true;
    const ResetReport r       = resetReport(c, Breadcrumb::Snapshot{}, SAG_V);
    REQUIRE(r.count == 1);
    CHECK(r.lines[0].severity == ResetSeverity::kInfo);
    CHECK(first(r) == "Reset cause: cold power-up. No history from a previous run.");
}

TEST_CASE("Pico verdicts keep their wording")
{
    hal::system::ResetCause c = none();
    c.power_on                = true;

    // POR stays latched on the Pico, so a surviving breadcrumb means the rail dipped.
    ResetReport r = resetReport(c, crumb(7.8f), SAG_V);
    CHECK(first(r) == "POWER FAULT: reset while the rail still read 7.80 V, after 4200 ms, "
                      "during: search run");
    CHECK(std::string(r.lines[1].text) == "  -> Voltage never sagged, so suspect an intermittent "
                                          "power or ground connection rather than the battery.");

    r = resetReport(c, crumb(0.0f), SAG_V);
    CHECK(first(r) == "POWER FAULT: reset after 4200 ms, during: search run");

    hal::system::ResetCause sw = c;
    sw.software                = true;
    r                          = resetReport(sw, crumb(7.8f), SAG_V);
    CHECK(r.lines[0].severity == ResetSeverity::kInfo);
    CHECK(first(r) == "Reset cause: software reboot (reflash or bootloader). Not a power fault.");

    hal::system::ResetCause pin = c;
    pin.run_pin                 = true;
    r                           = resetReport(pin, crumb(7.8f), SAG_V);
    CHECK(r.lines[0].severity == ResetSeverity::kWarning);
    CHECK(first(r) == "Reset cause: external reset (RUN pin or reset button).");
}

TEST_CASE("An empty activity reads as nothing logged yet")
{
    hal::system::ResetCause c = none();
    c.brownout                = true;
    const ResetReport r       = resetReport(c, crumb(7.8f, ""), SAG_V);
    CHECK(first(r).find("during: (nothing logged yet)") != std::string::npos);
}
