// ESP32-C3 bridge: the STM32's console UART (1 Mbaud) <-> BLE, exposed as a
// Nordic UART Service so phone apps (nRF Connect, Serial Bluetooth Terminal)
// and tools/serial_term.py (--ble) can drive the CLI.
//
// The STM32 holds this chip's EN low from arm until stop, so every disarm is
// a cold boot: nothing here persists, advertising starts within about 1 s of
// EN rising, and the host tool reconnects on its own.
//
// All buffering, chunking and drop counting is in lib/bridge (host-tested);
// this file only moves bytes between the UART, NimBLE and that module.

#include <Arduino.h>
#include <NimBLEDevice.h>

#include <mutex>

#include "bridge.h"
#include "nimble/nimble/host/include/host/ble_gatt.h"
#include "nimble/nimble/host/include/host/ble_hs_mbuf.h"
#include "nimble/porting/nimble/include/os/os_mbuf.h"

namespace
{

// Nordic UART Service. RX is written by the host; TX notifies the host.
constexpr const char* NUS_SERVICE = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char* NUS_RX      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char* NUS_TX      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// 40 ms of input at 1 Mbaud, so a slow loop pass never overruns the driver.
constexpr size_t   UART_RX_BUFFER_BYTES = 4096;
constexpr uint32_t STATS_PERIOD_MS      = 5000;
// NimBLE queues a notification in its shared mbuf pool (12 blocks by default)
// rather than failing when the radio is busy. Leave this many blocks free so
// the host stack can still answer writes, MTU and parameter requests while
// the uplink streams.
constexpr int MSYS_HEADROOM_BLOCKS = 4;

HardwareSerial stm32_uart(1);

// NimBLE callbacks run in the NimBLE host task; loop() runs in the Arduino
// task. The mutex serialises every Bridge call between them.
std::mutex            bridge_mutex;
bridge::Bridge        bridge_state;
NimBLECharacteristic* tx_char     = nullptr;
volatile uint16_t     conn_handle = BLE_HS_CONN_HANDLE_NONE;

class ServerCallbacks : public NimBLEServerCallbacks
{
    void onDisconnect(NimBLEServer*) override
    {
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        {
            std::lock_guard<std::mutex> lock(bridge_mutex);
            bridge_state.set_connected(false);
        }
        // NimBLEServer restarts advertising itself after this returns.
    }

    void onMTUChange(uint16_t mtu, ble_gap_conn_desc*) override
    {
        std::lock_guard<std::mutex> lock(bridge_mutex);
        bridge_state.set_mtu(mtu);
    }
};

class TxCallbacks : public NimBLECharacteristicCallbacks
{
    // A host is "connected" for the bridge once it subscribes to TX: before
    // that a notification has nowhere to go.
    void onSubscribe(NimBLECharacteristic*, ble_gap_conn_desc* desc, uint16_t sub_value) override
    {
        std::lock_guard<std::mutex> lock(bridge_mutex);
        if (sub_value != 0)
        {
            conn_handle = desc->conn_handle;
            bridge_state.set_connected(true);
            bridge_state.set_mtu(NimBLEDevice::getServer()->getPeerMTU(desc->conn_handle));
        }
        else
        {
            bridge_state.set_connected(false);
        }
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic* c) override
    {
        std::string                 value = c->getValue();
        std::lock_guard<std::mutex> lock(bridge_mutex);
        bridge_state.pushFromBle(reinterpret_cast<const uint8_t*>(value.data()), value.size());
    }
};

// Sends one notification through the raw NimBLE host API, because it reports
// failure synchronously. NimBLE queues notifications until the radio sends
// them, so the uplink's flow control is the mbuf pool: below the headroom, or
// on any failure, the bytes stay in the bridge and the next loop pass retries.
bool notify(const uint8_t* data, size_t n)
{
    uint16_t handle = conn_handle;
    if (handle == BLE_HS_CONN_HANDLE_NONE)
    {
        return false;
    }
    if (os_msys_num_free() < MSYS_HEADROOM_BLOCKS)
    {
        return false;
    }
    os_mbuf* om = ble_hs_mbuf_from_flat(data, n);
    if (om == nullptr)
    {
        return false;
    }
    // ble_gattc_notify_custom() frees om on every path.
    return ble_gattc_notify_custom(handle, tx_char->getHandle(), om) == 0;
}

void startBle()
{
    NimBLEDevice::init(BRIDGE_DEVICE_NAME);
    NimBLEDevice::setMTU(bridge::MAX_PAYLOAD + 3);

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    NimBLEService* service = server->createService(NUS_SERVICE);
    tx_char                = service->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
    tx_char->setCallbacks(new TxCallbacks());
    NimBLECharacteristic* rx =
        service->createCharacteristic(NUS_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new RxCallbacks());
    service->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SERVICE);
    adv->setScanResponse(true);
    adv->start();
}

void pumpUartToBle(uint32_t now_ms)
{
    uint8_t buf[256];
    while (stm32_uart.available() > 0)
    {
        size_t                      n = stm32_uart.read(buf, sizeof buf);
        std::lock_guard<std::mutex> lock(bridge_mutex);
        bridge_state.pushFromUart(buf, n, now_ms);
    }
    uint8_t chunk[bridge::MAX_PAYLOAD];
    for (;;)
    {
        std::lock_guard<std::mutex> lock(bridge_mutex);
        size_t                      n = bridge_state.peekBleChunk(chunk, now_ms);
        if (n == 0 || !notify(chunk, n))
        {
            return;
        }
        bridge_state.commitBleChunk(n);
    }
}

void pumpBleToUart()
{
    uint8_t buf[128];
    for (;;)
    {
        // Never block on the UART: take only what its TX FIFO can accept now.
        size_t room = static_cast<size_t>(stm32_uart.availableForWrite());
        if (room == 0)
        {
            return;
        }
        size_t n;
        {
            std::lock_guard<std::mutex> lock(bridge_mutex);
            n = bridge_state.popForUart(buf, room < sizeof buf ? room : sizeof buf);
        }
        if (n == 0)
        {
            return;
        }
        stm32_uart.write(buf, n);
    }
}

void reportStats(uint32_t now_ms)
{
    static uint32_t last_ms = 0;
    if (now_ms - last_ms < STATS_PERIOD_MS)
    {
        return;
    }
    last_ms = now_ms;
    bridge::Stats s;
    bool          connected;
    {
        std::lock_guard<std::mutex> lock(bridge_mutex);
        s         = bridge_state.stats();
        connected = bridge_state.connected();
    }
    // USB Serial/JTAG only (ARDUINO_USB_CDC_ON_BOOT), never the STM32 link.
    Serial.printf("[bridge] host=%d uart_rx=%lu ble_tx=%lu dropped=%lu discarded=%lu "
                  "down_dropped=%lu\n",
                  connected ? 1 : 0, (unsigned long)s.uart_rx_bytes, (unsigned long)s.ble_tx_bytes,
                  (unsigned long)s.uplink_dropped, (unsigned long)s.uplink_discarded,
                  (unsigned long)s.downlink_dropped);
}

} // namespace

void setup()
{
    Serial.begin(115200);
    stm32_uart.setRxBufferSize(UART_RX_BUFFER_BYTES);
    stm32_uart.begin(BRIDGE_UART_BAUD, SERIAL_8N1, BRIDGE_UART_RX_PIN, BRIDGE_UART_TX_PIN);
    startBle();
}

void loop()
{
    uint32_t now_ms = millis();
    pumpUartToBle(now_ms);
    pumpBleToUart();
    reportStats(now_ms);
    // The C3 is single core: yield a tick so the NimBLE host and idle tasks
    // run (and the task watchdog stays fed). The UART buffer covers the gap.
    vTaskDelay(1);
}
