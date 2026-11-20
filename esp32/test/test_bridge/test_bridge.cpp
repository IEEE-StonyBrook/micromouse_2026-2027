#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest.h>

#include <string>

#include "bridge.h"

namespace
{

void pushUart(bridge::Bridge& b, const std::string& s, uint32_t now_ms)
{
    b.pushFromUart(reinterpret_cast<const uint8_t*>(s.data()), s.size(), now_ms);
}

// Pops one BLE chunk the way the firmware does: peek, notify, commit.
std::string takeChunk(bridge::Bridge& b, uint32_t now_ms)
{
    uint8_t buf[bridge::MAX_PAYLOAD];
    size_t  n = b.peekBleChunk(buf, now_ms);
    b.commitBleChunk(n);
    return std::string(reinterpret_cast<char*>(buf), n);
}

// Takes every ready chunk, as the firmware's loop does across polls.
std::string drainAll(bridge::Bridge& b, uint32_t now_ms)
{
    std::string out;
    for (std::string c = takeChunk(b, now_ms); !c.empty(); c = takeChunk(b, now_ms))
    {
        out += c;
    }
    return out;
}

} // namespace

TEST_CASE("a complete line goes to BLE straight away")
{
    bridge::Bridge b;
    b.set_connected(true);
    pushUart(b, "ok\r\n", 0);
    CHECK(takeChunk(b, 0) == "ok\r\n");
    CHECK(takeChunk(b, 0) == "");
}

TEST_CASE("a partial line waits for more bytes, then flushes once the UART goes idle")
{
    bridge::Bridge b;
    b.set_connected(true);
    pushUart(b, "> ", 100);
    CHECK(takeChunk(b, 100) == "");
    CHECK(takeChunk(b, 100 + bridge::FLUSH_IDLE_MS - 1) == "");
    CHECK(takeChunk(b, 100 + bridge::FLUSH_IDLE_MS) == "> ");
}

TEST_CASE("a long line leaves in payload-sized chunks for the negotiated MTU")
{
    bridge::Bridge b;
    b.set_connected(true);
    const std::string line = std::string(50, 'a') + "\n"; // 51 bytes
    pushUart(b, line, 0);

    // Default ATT MTU of 23 gives 20-byte payloads.
    CHECK(takeChunk(b, 0) == std::string(20, 'a'));
    CHECK(takeChunk(b, 0) == std::string(20, 'a'));
    CHECK(takeChunk(b, 0) == std::string(10, 'a') + "\n");

    // A 100-byte MTU (97-byte payload) carries a second line in one notify.
    b.set_mtu(100);
    pushUart(b, line, 0);
    pushUart(b, line, 0);
    CHECK(takeChunk(b, 0).size() == 97);
    CHECK(takeChunk(b, 0).size() == 5);
}

TEST_CASE("the payload clamps to the 20..244 byte range")
{
    bridge::Bridge b;
    b.set_mtu(10);
    CHECK(b.payload() == 20);
    b.set_mtu(517);
    CHECK(b.payload() == 244);
}

TEST_CASE("a failed notify keeps its bytes for the retry")
{
    bridge::Bridge b;
    b.set_connected(true);
    pushUart(b, "stats\r\n", 0);
    uint8_t buf[bridge::MAX_PAYLOAD];
    CHECK(b.peekBleChunk(buf, 0) == 7); // notify fails: no commit
    CHECK(takeChunk(b, 1) == "stats\r\n");
    CHECK(b.stats().ble_tx_bytes == 7);
}

TEST_CASE("when BLE falls behind, new UART bytes drop, are counted, and a marker shows the gap")
{
    bridge::Bridge b;
    b.set_connected(true);
    const std::string filler(bridge::UPLINK_CAPACITY, 'x');
    pushUart(b, filler, 0);
    pushUart(b, "lost", 0);
    CHECK(b.stats().uplink_dropped == 4);

    // Drain all the filler, then the next byte arrives after the gap marker.
    CHECK(drainAll(b, 10) == filler);
    pushUart(b, "ok\n", 20);
    CHECK(drainAll(b, 20) == "\r\n[bridge] dropped 4 bytes\r\nok\n");
    CHECK(b.stats().uplink_dropped == 4);
}

TEST_CASE("UART bytes with no host connected are discarded and reported on connect")
{
    bridge::Bridge b;
    pushUart(b, "boot banner\r\n", 0);
    CHECK(b.stats().uplink_discarded == 13);
    b.set_connected(true);
    CHECK(drainAll(b, 0) == "[bridge] discarded 13 bytes while disconnected\r\n");
}

TEST_CASE("a disconnect discards the unsent uplink and forgets the MTU")
{
    bridge::Bridge b;
    b.set_connected(true);
    b.set_mtu(247);
    pushUart(b, "half a li", 0);
    b.set_connected(false);
    CHECK(b.stats().uplink_discarded == 9);
    CHECK(b.payload() == bridge::MIN_PAYLOAD);
    b.set_connected(true);
    CHECK(drainAll(b, 0) == "[bridge] discarded 9 bytes while disconnected\r\n");
}

TEST_CASE("host writes reach the UART in order; overflow drops and counts")
{
    bridge::Bridge b;
    b.set_connected(true);
    const std::string cmd = "flood\r\n";
    b.pushFromBle(reinterpret_cast<const uint8_t*>(cmd.data()), cmd.size());
    uint8_t buf[64];
    size_t  n = b.popForUart(buf, sizeof buf);
    CHECK(std::string(reinterpret_cast<char*>(buf), n) == cmd);
    CHECK(b.popForUart(buf, sizeof buf) == 0);

    const std::string big(bridge::DOWNLINK_CAPACITY + 3, 'y');
    b.pushFromBle(reinterpret_cast<const uint8_t*>(big.data()), big.size());
    CHECK(b.stats().downlink_dropped == 3);
}

// PlatformIO's doctest runner counts cases from the per-assertion report, so
// successes must be printed too.
int main(int argc, char** argv)
{
    doctest::Context context;
    context.setOption("success", true);
    context.applyCommandLine(argc, argv);
    return context.run();
}
