"""Tests for ble_link.py against a fake BLE backend (no radio, no bleak needed).

Run:  python3 -m unittest discover -s tools -p 'test_*.py'
"""

import unittest

import ble_link


class FakeClient:
    def __init__(self, payload):
        self.payload = payload
        self.writes = []
        self.disconnected = False

    async def write(self, data):
        self.writes.append(bytes(data))

    async def disconnect(self):
        self.disconnected = True


class FakeBackend:
    """Stands in for bleak: 'connects' to a named device the test controls."""

    def __init__(self, present=True, payload=20):
        self.present = present
        self.payload = payload
        self.client = None
        self.on_data = None
        self.on_disconnect = None
        self.connects = 0

    async def connect(self, name, timeout, on_data, on_disconnect):
        self.connects += 1
        if not self.present:
            raise OSError(f"{name} not found")
        self.on_data, self.on_disconnect = on_data, on_disconnect
        self.client = FakeClient(self.payload)
        return self.client


class ChunkTest(unittest.TestCase):
    def test_splits_to_payload(self):
        self.assertEqual(ble_link.chunks(b"a" * 45, 20), [b"a" * 20, b"a" * 20, b"a" * 5])

    def test_empty_sends_nothing(self):
        self.assertEqual(ble_link.chunks(b"", 20), [])


class TransportTest(unittest.TestCase):
    def test_port_names(self):
        self.assertEqual(ble_link.device_name("ble:Jurababa"), "Jurababa")
        self.assertEqual(ble_link.device_name("ble:"), ble_link.DEFAULT_NAME)
        self.assertIsNone(ble_link.device_name("/dev/ttyACM0"))

    def test_notifications_read_in_order(self):
        backend = FakeBackend()
        t = ble_link.BleTransport("Jurababa", backend=backend)
        backend.on_data(b"ok\r\n")
        backend.on_data(b"> ")
        self.assertEqual(t.read(t.in_waiting or 1), b"ok\r\n> ")
        self.assertEqual(t.read(1), b"")  # times out quietly, like pyserial
        t.close()

    def test_write_is_chunked_to_the_mtu(self):
        backend = FakeBackend(payload=20)
        t = ble_link.BleTransport("Jurababa", backend=backend)
        t.write(b"x" * 30)
        self.assertEqual(backend.client.writes, [b"x" * 20, b"x" * 10])
        t.close()
        self.assertTrue(backend.client.disconnected)

    def test_missing_device_raises_so_the_caller_retries(self):
        with self.assertRaises(OSError):
            ble_link.BleTransport("Jurababa", backend=FakeBackend(present=False))

    def test_cold_boot_drop_raises_after_buffered_bytes(self):
        # The STM32 pulls ESP32 EN low on arm: the link vanishes mid-session.
        backend = FakeBackend()
        t = ble_link.BleTransport("Jurababa", backend=backend)
        backend.on_data(b"arming\r\n")
        backend.on_disconnect()
        self.assertEqual(t.read(1), b"arming\r\n")
        with self.assertRaises(OSError):
            t.read(1)
        with self.assertRaises(OSError):
            t.write(b"STOP\r\n")
        t.close()

        # After disarm the bridge cold-boots; a fresh transport reconnects.
        t2 = ble_link.BleTransport("Jurababa", backend=backend)
        backend.on_data(b"> ")
        self.assertEqual(t2.read(1), b"> ")
        self.assertEqual(backend.connects, 2)
        t2.close()


if __name__ == "__main__":
    unittest.main()
