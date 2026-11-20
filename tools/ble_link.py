"""BLE transport for the ESP32-C3 bridge (esp32/), shaped like a pyserial port.

The bridge exposes the mouse's CLI as a Nordic UART Service. BleTransport
gives serial_term.py the read/write/in_waiting/close calls it already uses for
a COM port, so its reconnect loop works unchanged: the STM32 holds the ESP32's
EN low while the mouse is armed, the link drops, read() raises, and the loop
opens a fresh BleTransport until the bridge has cold-booted and advertises.

Port names: "ble:" or "ble:<advertised name>" (default "Jurababa").
Needs `pip install bleak` (in tools/requirements.txt); nothing else here does.
"""

import asyncio
import queue
import threading

DEFAULT_NAME = "Jurababa"
PORT_PREFIX = "ble:"

NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # host writes
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # bridge notifies

SCAN_TIMEOUT_S = 5.0
CONNECT_TIMEOUT_S = 10.0  # connect plus subscribe, after the scan
READ_TIMEOUT_S = 0.05  # matches serial_term's pyserial timeout


def device_name(port):
    """Returns the BLE device name for a "ble:..." port, else None."""
    if not port or not port.lower().startswith(PORT_PREFIX):
        return None
    return port[len(PORT_PREFIX):] or DEFAULT_NAME


def chunks(data, payload):
    """Splits data into writes no larger than one ATT payload (MTU - 3)."""
    return [data[i:i + payload] for i in range(0, len(data), payload)]


class BleakBackend:
    """The real radio, through bleak. Imported lazily so serial use and the
    tests never need bleak installed."""

    def __init__(self, client_cls=None, scanner_cls=None):
        self._client_cls = client_cls
        self._scanner_cls = scanner_cls

    async def connect(self, name, timeout, on_data, on_disconnect):
        if self._client_cls is None or self._scanner_cls is None:
            from bleak import BleakClient, BleakScanner
            self._client_cls = self._client_cls or BleakClient
            self._scanner_cls = self._scanner_cls or BleakScanner

        device = await self._scanner_cls.find_device_by_name(name, timeout=timeout)
        if device is None:
            raise OSError(f"BLE device {name!r} not advertising")
        client = self._client_cls(device, disconnected_callback=lambda _c: on_disconnect())
        try:
            # Bounded here, not only by the caller's wait, so a stuck connect is
            # cancelled on this loop rather than finishing with no owner.
            await asyncio.wait_for(self._open(client, on_data), CONNECT_TIMEOUT_S)
        except BaseException:
            # A half-open link keeps the bridge from advertising, so every
            # later reconnect would fail: always release it.
            try:
                await client.disconnect()
            except Exception:
                pass
            raise
        return _BleakClientAdapter(client)

    @staticmethod
    async def _open(client, on_data):
        await client.connect()
        await client.start_notify(NUS_TX, lambda _h, data: on_data(bytes(data)))


class _BleakClientAdapter:
    def __init__(self, client):
        self._client = client
        # bleak reports the negotiated ATT MTU; a notification/write carries 3 less.
        self.payload = max(20, client.mtu_size - 3)

    async def write(self, data):
        # With response: a command must not vanish silently, and commands are short.
        await self._client.write_gatt_char(NUS_RX, data, response=True)

    async def disconnect(self):
        await self._client.disconnect()


class BleTransport:
    """Connects on construction (raises OSError if the bridge isn't found)."""

    def __init__(self, name=DEFAULT_NAME, backend=None, scan_timeout=SCAN_TIMEOUT_S):
        self.baudrate = None  # serial_term sets it; BLE has no baud
        self._rx = queue.Queue()
        self._lost = threading.Event()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        self._client = None
        try:
            self._client = self._call(
                (backend or BleakBackend()).connect(name, scan_timeout, self._rx.put, self._lost.set),
                # Headroom over the backend's own bounds, so they fire first.
                timeout=scan_timeout + CONNECT_TIMEOUT_S + 5.0)
        except Exception as e:
            self._stop_loop()
            raise OSError(f"BLE connect to {name!r} failed: {e}") from e

    def _call(self, coro, timeout=5.0):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout)

    @property
    def in_waiting(self):
        return self._rx.qsize()

    def read(self, _size=1):
        """Returns every notification received so far, b"" after a short
        quiet wait, or raises OSError once the link has dropped and drained."""
        try:
            out = [self._rx.get(timeout=READ_TIMEOUT_S)]
        except queue.Empty:
            if self._lost.is_set():
                raise OSError("BLE link lost")
            return b""
        while True:
            try:
                out.append(self._rx.get_nowait())
            except queue.Empty:
                return b"".join(out)

    def write(self, data):
        if self._lost.is_set():
            raise OSError("BLE link lost")
        for part in chunks(bytes(data), self._client.payload):
            self._call(self._client.write(part))
        return len(data)

    def close(self):
        if self._client is not None and not self._lost.is_set():
            try:
                self._call(self._client.disconnect())
            except Exception:
                pass  # already gone; closing must not fail
        self._stop_loop()

    def _stop_loop(self):
        # Close the loop too: while the mouse is armed this runs on every
        # failed reconnect, and an unclosed loop leaks its selector.
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=1.0)
        if not self._thread.is_alive():
            self._loop.close()
