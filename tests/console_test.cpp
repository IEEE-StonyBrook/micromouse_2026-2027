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

TEST_CASE("console printFormat formats like printf, long lines included")
{
    namespace console = hal::console;
    console::host::reset();
    console::printFormat("kM = %8.2f mm/s/V, n=%d %s\n", 123.456, 7, "ok");
    CHECK(console::host::takeOutput() == "kM =   123.46 mm/s/V, n=7 ok\n");

    // Past the stack buffer: the whole line still goes out, nothing cut.
    const std::string long_text(2000, 'x');
    console::printFormat("[%s]", long_text.c_str());
    CHECK(console::host::takeOutput() == "[" + long_text + "]");

    console::printFormat("%s", "");
    CHECK(console::host::takeOutput().empty());
}

TEST_CASE("console printFormat goes through the TX gate")
{
    namespace console = hal::console;
    console::host::reset();
    console::setTxGate(true);
    console::printFormat("%d\n", 12345);
    CHECK(console::host::takeOutput().empty());
    CHECK(console::txGateDroppedBytes() == 6);
    console::host::reset();
}
