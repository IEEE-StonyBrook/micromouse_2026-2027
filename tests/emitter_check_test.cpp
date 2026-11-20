// The IR emitter isolation self-test: fire each emitter alone, read every
// receiver dark-subtracted, and judge the crosstalk matrix.
#include <string>

#include "common/emitter_check.h"
#include "doctest/doctest.h"
#include "fakes/fake_ir_bank.h"
#include "hal/host/console_test.h"

TEST_CASE("crosstalk capture fires each emitter alone and subtracts the dark reading")
{
    fake::IrBank bank(4);
    const uint16_t dark[4] = {100, 200, 300, 400};
    for (int r = 0; r < 4; ++r)
        bank.dark[r] = dark[r];
    bank.coupling[0][0] = 1000;
    bank.coupling[0][1] = 50;
    bank.coupling[1][1] = 1200;
    bank.coupling[2][2] = 900;
    bank.coupling[2][3] = 30;
    bank.coupling[3][3] = 1100;

    const CrosstalkMatrix m = captureCrosstalk(bank);

    REQUIRE(m.channels == 4);
    CHECK(m.counts[0][0] == 1000);
    CHECK(m.counts[0][1] == 50);
    CHECK(m.counts[0][2] == 0);
    CHECK(m.counts[1][1] == 1200);
    CHECK(m.counts[2][3] == 30);
    CHECK(m.counts[3][3] == 1100);
    CHECK(m.counts[3][0] == 0);

    // Every read lights at most one emitter, and each emitter gets its turn.
    for (int e = 0; e < 4; ++e)
    {
        int lit_reads = 0;
        for (int l : bank.lit)
            lit_reads += (l == e);
        CHECK(lit_reads == CROSSTALK_SAMPLES);
    }
}

namespace
{
/// A 4-channel matrix with `diag` on the diagonal and `off` everywhere else.
CrosstalkMatrix uniform(int32_t diag, int32_t off)
{
    CrosstalkMatrix m;
    m.channels = 4;
    for (int e = 0; e < 4; ++e)
        for (int r = 0; r < 4; ++r)
            m.counts[e][r] = (e == r) ? diag : off;
    return m;
}
} // namespace

TEST_CASE("a diagonal-dominant matrix passes with no leaks")
{
    const CrosstalkVerdict v = judgeCrosstalk(uniform(1000, 40));
    CHECK(v.pass);
    CHECK(v.leak_count == 0);
    CHECK(v.weak_mask == 0);
}

TEST_CASE("a pair leaking more than a quarter of the receiver's own signal fails and is named")
{
    CrosstalkMatrix m = uniform(1000, 40);
    m.counts[0][1]    = 251; // emitter 0 into receiver 1: 25.1 % of 1000
    m.counts[3][2]    = 250; // exactly 25 %: allowed

    const CrosstalkVerdict v = judgeCrosstalk(m);
    CHECK_FALSE(v.pass);
    REQUIRE(v.leak_count == 1);
    CHECK(v.leaks[0].emitter == 0);
    CHECK(v.leaks[0].receiver == 1);
}

TEST_CASE("a receiver that barely sees its own emitter fails as weak")
{
    CrosstalkMatrix m = uniform(1000, 0);
    m.counts[2][2]    = CROSSTALK_MIN_DIAGONAL_COUNTS - 1;
    m.counts[0][2]    = 60; // above 25 % of the weak diagonal, but not judged as a leak

    const CrosstalkVerdict v = judgeCrosstalk(m);
    CHECK_FALSE(v.pass);
    CHECK(v.weak_mask == (1u << 2));
    CHECK(v.leak_count == 0);

    m.counts[2][2] = CROSSTALK_MIN_DIAGONAL_COUNTS;
    m.counts[0][2] = 0;
    CHECK(judgeCrosstalk(m).pass);
}

TEST_CASE("a board with no emitters never passes")
{
    CHECK_FALSE(judgeCrosstalk(CrosstalkMatrix{}).pass);
}

TEST_CASE("the self-test prints the matrix and names each weak receiver and leaking pair")
{
    fake::IrBank bank(4);
    for (int c = 0; c < 4; ++c)
        bank.coupling[c][c] = 1000;
    bank.coupling[2][2] = 40;  // weak receiver 2
    bank.coupling[0][1] = 300; // emitter 0 leaks into receiver 1
    hal::console::host::reset();

    const CrosstalkVerdict v   = runEmitterCheck(bank);
    const std::string      out = hal::console::host::takeOutput();

    CHECK_FALSE(v.pass);
    CHECK(out.find("  E0  1000   300     0     0") != std::string::npos);
    CHECK(out.find("  E2     0     0    40     0") != std::string::npos);
    CHECK(out.find("weak: R2 (40 < 100 counts)") != std::string::npos);
    CHECK(out.find("leak: E0 -> R1 (300 counts, 30 % of R1's own; limit 25 %)") !=
          std::string::npos);
    CHECK(out.find("emitters: FAIL") != std::string::npos);
}

TEST_CASE("the self-test reports PASS on a clean board and says when there are no emitters")
{
    fake::IrBank bank(4);
    for (int c = 0; c < 4; ++c)
        bank.coupling[c][c] = 1000;
    hal::console::host::reset();
    CHECK(runEmitterCheck(bank).pass);
    CHECK(hal::console::host::takeOutput().find("emitters: PASS") != std::string::npos);

    fake::IrBank none(0);
    CHECK_FALSE(runEmitterCheck(none).pass);
    CHECK(hal::console::host::takeOutput() == "emitters: none on this board\n");
}
