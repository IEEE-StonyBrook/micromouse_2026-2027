"""Tests for the physics-sim GUI server: knob checks, summary parsing, /run.

    python3 -m unittest discover -s tools/physim_gui -v

The /run tests need tests/physim (`make -C tests physim`) and skip without it.
"""

import json
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import server


class BuildArgsTest(unittest.TestCase):
    def test_scenario_alone_gets_the_default_decimation(self):
        self.assertEqual(
            server.build_args({"scenario": "ip90"}),
            ["--scenario", "ip90", "--every", str(server.DEFAULT_EVERY)],
        )

    def test_knobs_pass_through_as_flags(self):
        args = server.build_args(
            {"scenario": "straight10", "friction": "0", "seed": "7", "every": "5"}
        )
        self.assertEqual(
            args,
            ["--scenario", "straight10", "--friction", "0.0", "--seed", "7", "--every", "5"],
        )

    def test_rejects_a_missing_or_unknown_scenario(self):
        for query in ({}, {"scenario": "spin"}, {"scenario": "ip90; rm -rf /"}):
            with self.assertRaises(ValueError):
                server.build_args(query)

    def test_rejects_an_unknown_knob(self):
        with self.assertRaisesRegex(ValueError, "unknown knob"):
            server.build_args({"scenario": "ip90", "pinned": "1"})

    def test_rejects_values_that_are_not_finite_numbers_in_range(self):
        for name, text in (
            ("battery", "abc"),
            ("battery", ""),
            ("battery", "nan"),
            ("battery", "inf"),
            ("battery", "99"),
            ("mu", "0"),
            ("seed", "1.5"),
            ("every", "0"),
        ):
            with self.subTest(name=name, text=text), self.assertRaises(ValueError):
                server.build_args({"scenario": "ip90", name: text})

    def test_accepts_the_range_ends(self):
        lo, hi, _ = server.KNOBS["gyro-bias"]
        server.build_args({"scenario": "ip90", "gyro-bias": str(lo)})
        server.build_args({"scenario": "ip90", "gyro-bias": str(hi)})


class ParseSummaryTest(unittest.TestCase):
    def test_parses_each_field_as_a_number(self):
        s = server.parse_summary("finished=1 duration_s=1.536 yaw_deg=93.149 junk")
        self.assertEqual(s, {"finished": 1.0, "duration_s": 1.536, "yaw_deg": 93.149})


@unittest.skipUnless(server.PHYSIM.exists(), "tests/physim not built")
class RunEndpointTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        threading.Thread(target=cls.httpd.serve_forever, daemon=True).start()
        cls.base = f"http://127.0.0.1:{cls.httpd.server_address[1]}"

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.httpd.server_close()

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=30) as r:
            return r.status, r.read()

    def test_run_returns_the_trace_and_summary(self):
        status, body = self.get("/run?scenario=ip90&friction=0&every=10")
        self.assertEqual(status, 200)
        data = json.loads(body)
        rows = data["csv"].strip().splitlines()
        self.assertTrue(rows[0].startswith("time_s,target_mm,target_deg"))
        self.assertGreater(len(rows), 20)
        self.assertEqual(data["summary"]["finished"], 1.0)
        # Without the friction the feedforward over-drives the spin: ~93 deg.
        self.assertGreater(data["summary"]["yaw_deg"], 91.5)

    def test_a_bad_knob_is_a_400_with_the_reason(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self.get("/run?scenario=ip90&battery=99")
        self.assertEqual(ctx.exception.code, 400)
        self.assertIn("battery", json.loads(ctx.exception.read())["error"])


if __name__ == "__main__":
    unittest.main()
