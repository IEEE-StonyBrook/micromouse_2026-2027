#include "hal/console.h"
#include "hal/host/console_test.h"

#include <deque>
#include <string>

namespace hal::console
{
namespace
{
std::deque<unsigned char> input;
std::string               output;
} // namespace

int getcNonBlocking()
{
    if (input.empty())
        return -1;
    const unsigned char c = input.front();
    input.pop_front();
    return c;
}

void detail::writeBytes(const char* buf, std::size_t len)
{
    output.append(buf, len);
}

void host::pushInput(const std::string& text)
{
    input.insert(input.end(), text.begin(), text.end());
}

std::string host::takeOutput()
{
    std::string taken;
    taken.swap(output);
    return taken;
}

void host::reset()
{
    input.clear();
    output.clear();
    setTxGate(false);
    resetTxGateDrops();
}
} // namespace hal::console
