// Host tests for hal::console against the in-memory host backend.
#include <cstring>
#include <string>

#include "doctest/doctest.h"
#include "hal/console.h"
#include "hal/host/console_test.h"

namespace
{
void write(const char* text)
{
    hal::console::write(text, std::strlen(text));
}
} // namespace

TEST_CASE("console getc returns queued input in order, then -1")
{
    namespace console = hal::console;
    console::host::reset();
    CHECK(console::getcNonBlocking() == -1);

    console::host::pushInput("ab");
    const int first  = console::getcNonBlocking();
    const int second = console::getcNonBlocking();
    CHECK(first == 'a');
    CHECK(second == 'b');
    CHECK(console::getcNonBlocking() == -1);

    // High bytes come back as 0..255, never as -1.
    console::host::pushInput("\xff");
    CHECK(console::getcNonBlocking() == 0xFF);
}

TEST_CASE("console write sends exactly len bytes, embedded NUL included")
{
    namespace console = hal::console;
    console::host::reset();
    write("hello");
    console::write("x\0y", 3);
    CHECK(console::host::takeOutput() == std::string("hellox\0y", 8));
}

TEST_CASE("console TX gate drops and counts gated writes")
{
    namespace console = hal::console;
    console::host::reset();
    CHECK_FALSE(console::txGated());
    console::setTxGate(true);
    CHECK(console::txGated());
    write("dropped");
    write("!!");
    CHECK(console::host::takeOutput().empty());
    CHECK(console::txGateDroppedBytes() == 9);

    console::setTxGate(false);
    write("ok");
    CHECK(console::host::takeOutput() == "ok");
    CHECK(console::txGateDroppedBytes() == 9);

    console::resetTxGateDrops();
    CHECK(console::txGateDroppedBytes() == 0);
}
