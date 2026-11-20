// The IR emitter isolation self-test: fire each emitter alone, read every
// receiver dark-subtracted, and judge the crosstalk matrix.
#include <string>

#include "common/emitter_check.h"
#include "doctest/doctest.h"
#include "fakes/fake_ir_bank.h"

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
