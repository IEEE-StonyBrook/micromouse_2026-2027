// Host tests for hal::console against the in-memory host backend.
#include "hal/console.h"
#include "hal/host/console_test.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace
{
int failures = 0;

void check(bool ok, const char* what)
{
    std::printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        failures++;
}

void write(const char* text)
{
    hal::console::write(text, std::strlen(text));
}
} // namespace

int main()
{
    namespace console = hal::console;

    console::host::reset();
    check(console::getcNonBlocking() == -1, "getc returns -1 when no input is waiting");

    console::host::pushInput("ab");
    const int first  = console::getcNonBlocking();
    const int second = console::getcNonBlocking();
    check(first == 'a' && second == 'b', "getc returns queued bytes in order");
    check(console::getcNonBlocking() == -1, "getc returns -1 once the queue drains");

    console::host::pushInput("\xff");
    check(console::getcNonBlocking() == 0xFF, "getc returns high bytes as 0..255, not -1");

    console::host::reset();
    write("hello");
    console::write("x\0y", 3);
    check(console::host::takeOutput() == std::string("hellox\0y", 8),
          "write sends exactly len bytes, embedded NUL included");

    console::host::reset();
    check(!console::txGated(), "TX gate starts open");
    console::setTxGate(true);
    check(console::txGated(), "setTxGate(true) closes the gate");
    write("dropped");
    write("!!");
    check(console::host::takeOutput().empty(), "gated writes send nothing");
    check(console::txGateDroppedBytes() == 9, "gated writes count every dropped byte");

    console::setTxGate(false);
    write("ok");
    check(console::host::takeOutput() == "ok", "writes go through once the gate reopens");
    check(console::txGateDroppedBytes() == 9, "open-gate writes leave the drop count alone");

    console::resetTxGateDrops();
    check(console::txGateDroppedBytes() == 0, "resetTxGateDrops zeroes the count");

    return failures == 0 ? 0 : 1;
}
