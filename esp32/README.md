# ESP32-C3 BLE bridge

Firmware for the ESP32-C3 on the STM32 mouse. It carries the CLI between the
STM32's USART3 (1 Mbaud, 8N1) and a host over BLE, as a Nordic UART Service
(NUS) advertised as `Jurababa`. Wi-Fi stays off.

- `lib/bridge/`: the bridging logic in plain C++ (buffering, MTU chunking,
  idle flush of partial lines, notify retry, drop counting). No Arduino or
  NimBLE, so it is tested on the host.
- `src/main.cpp`: Arduino-ESP32 and NimBLE glue.
- `test/test_bridge/`: doctest unit tests for `lib/bridge`.

## Build and test

PlatformIO, versions pinned in `platformio.ini`:

```bash
pio test -d esp32 -e native     # bridge logic on the host
pio run -d esp32 -e esp32c3     # firmware
pio run -d esp32 -e esp32c3 -t upload
```

The UART pins are build flags in `platformio.ini` (`BRIDGE_UART_RX_PIN`,
`BRIDGE_UART_TX_PIN`, default GPIO20/21). Set them from the schematic before
flashing. Bridge counters print every 5 s on the C3's USB port, never on the
STM32 link.

## Behaviour

- Uplink (STM32 to host): a notification leaves when a full payload
  (ATT MTU - 3, 20 to 244 bytes) is buffered, a newline is buffered, or the
  UART has been quiet for 5 ms (so the `> ` prompt arrives). A failed notify
  is retried on the next pass. If BLE falls behind the 4 KB buffer fills,
  new bytes drop, and the host sees `[bridge] dropped N bytes` at the gap.
- With no host subscribed, UART bytes are discarded; the next connection
  starts with `[bridge] discarded N bytes while disconnected`.
- Downlink (host to STM32): writes are queued (1 KB) and drained into the
  UART without blocking.
- Radio off while armed: the STM32 holds the ESP32's EN low from arm until
  stop, so every disarm is a cold boot. The firmware keeps no state and
  advertises at once; `tools/serial_term.py` reconnects on its own.

## Host

```bash
pip install -r tools/requirements.txt          # adds bleak
python tools/serial_term.py --port ble:Jurababa
```

`ble:Jurababa` is also in the port list. `tools/serial_cmd.py` works through
the terminal as before. Any NUS app (nRF Connect, Serial Bluetooth Terminal)
can connect too: write commands to `6E400002-...`, subscribe to `6E400003-...`.

## Known gaps

- No pairing or bonding: anyone in range can connect while the mouse is
  disarmed.
- The C3's ROM prints its boot log on UART0 (GPIO21 by default) at 115200
  baud. If the STM32 link uses those pins, the STM32 sees a burst of framing
  errors after every EN rise; move the link to other pins or silence the ROM
  log (eFuse / GPIO8 strap).
