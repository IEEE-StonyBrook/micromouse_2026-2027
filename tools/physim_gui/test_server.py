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


class PresetsTest(unittest.TestCase):
    def test_presets_js_is_current_with_the_spec(self):
        # Edit presets_spec.json, then rerun server.py --make-presets.
        text = server.PRESETS.read_text()
        built = json.loads(text[text.index("{") : text.rindex("}") + 1])
        spec = json.loads(server.PRESET_SPEC.read_text())
        for preset in built["presets"]:
            for run in preset["runs"]:
                self.assertTrue(run.pop("result")["csv"].startswith("time_s,"))
        self.assertEqual(built, spec)


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
        with ctx.exception as err:
            self.assertEqual(err.code, 400)
            self.assertIn("battery", json.loads(err.read())["error"])

    def post(self, path, obj):
        req = urllib.request.Request(
            self.base + path, data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, json.loads(r.read())

    def test_logs_lists_the_fixture(self):
        status, body = self.get("/logs")
        self.assertEqual(status, 200)
        self.assertIn("fixtures/path_ffrfflf_capture.log", [l["name"] for l in json.loads(body)["logs"]])

    def test_realsim_replays_the_fixture_both_ways(self):
        status, data = self.post("/realsim", {"log": "fixtures/path_ffrfflf_capture.log"})
        self.assertEqual(status, 200)
        n = data["real"]["header"]["n"]
        self.assertEqual(data["path"], "FFRFFLF 400 2000 240 800")
        self.assertGreater(len(data["cmd"]["cols"]["t_ms"]), 400)
        self.assertEqual(len(data["volts"]["cols"]["t_ms"]), n)
        self.assertEqual(data["argv"]["cmd"][:3], ["physim", "--script", "FFRFFLF 400 2000 240 800"])
        # The header's battery drives both replays when the knob is blank.
        self.assertIn("8.35", data["argv"]["cmd"])

    def test_realsim_takes_an_uploaded_log_a_typed_path_and_knobs(self):
        text = server.FIXTURES.joinpath("path_ffrfflf_capture.log").read_text()
        _, nominal = self.post("/realsim", {"text": text})
        status, data = self.post(
            "/realsim", {"text": text, "path": "FFRFFLF 400 2000", "knobs": {"friction": "0.9", "mu": "0.6"}}
        )
        self.assertEqual(status, 200)
        self.assertEqual(data["path"], "FFRFFLF 400 2000")
        self.assertIn("--friction", data["argv"]["volts"])
        # The knobs that made the fixture fit its volts far better than nominal does.
        self.assertLess(sum(data["volts_rms"].values()), 0.5 * sum(nominal["volts_rms"].values()))

    def test_realsim_refuses_a_bad_request(self):
        for body in (
            {"log": "../../CLAUDE.md"},
            {"log": "fixtures/path_ffrfflf_capture.log", "path": "FF;ls"},
            {"log": "fixtures/path_ffrfflf_capture.log", "knobs": {"every": "1"}},
            {"text": "no trace here"},
            {},
        ):
            with self.subTest(body=body), self.assertRaises(urllib.error.HTTPError) as ctx:
                self.post("/realsim", body)
            with ctx.exception as err:
                self.assertEqual(err.code, 400)
                self.assertIn("error", json.loads(err.read()))

    def test_static_files_stay_inside_the_gui_folder(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self.get("/../../CLAUDE.md")
        ctx.exception.close()

    def test_serves_the_page(self):
        status, body = self.get("/index.html")
        self.assertEqual(status, 200)
        self.assertIn(b"<html", body)


class LogNameTest(unittest.TestCase):
    def test_resolves_a_listed_fixture(self):
        path = server.resolve_log("fixtures/path_ffrfflf_capture.log")
        self.assertEqual(path, server.REPO / "tools" / "fixtures" / "path_ffrfflf_capture.log")

    def test_rejects_anything_outside_the_two_folders(self):
        for name in (
            "../fixtures/path_ffrfflf_capture.log",
            "fixtures/../../CLAUDE.md",
            "fixtures/../fixtures/path_ffrfflf_capture.log",
            "/etc/passwd",
            "fixtures/%2e%2e/x.log",
            "tests/physim",
            "fixtures/missing.log",
            "fixtures/path_ffrfflf_capture.txt",
            "fixtures\\path_ffrfflf_capture.log",
            "",
        ):
            with self.subTest(name=name), self.assertRaises(ValueError):
                server.resolve_log(name)

    def test_lists_the_fixture(self):
        names = [log["name"] for log in server.list_logs()]
        self.assertIn("fixtures/path_ffrfflf_capture.log", names)
        self.assertTrue(all(n.startswith(("logs/", "fixtures/")) and n.endswith(".log") for n in names))


class RealsimArgsTest(unittest.TestCase):
    def test_path_text_is_checked_word_by_word(self):
        self.assertEqual(server.check_path("FFRFFLF 400 2000 SMOOTH"), "FFRFFLF 400 2000 SMOOTH")
        self.assertEqual(server.check_path("  ffr F2 l 600 "), "ffr F2 l 600")
        for text in ("", "FFX", "FF; rm -rf /", "FF --battery 99", "F" * 300, "FF 1e9", "FF -5"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                server.check_path(text)

    def test_only_plant_knobs_pass(self):
        self.assertEqual(server.plant_args({"mu": "0.6", "seed": "7"}), ["--mu", "0.6", "--seed", "7"])
        for query in ({"every": "2"}, {"speed": "600"}, {"script": "F"}, {"mu": "9"}):
            with self.subTest(query=query), self.assertRaises(ValueError):
                server.plant_args(query)


class ParseLogTest(unittest.TestCase):
    def test_returns_each_run_and_the_picked_one_by_column(self):
        text = server.FIXTURES.joinpath("path_ffrfflf_capture.log").read_text()
        out = server.parse_log(text, -1)
        self.assertEqual(len(out["runs"]), 1)
        self.assertEqual(out["runs"][0]["path"], "FFRFFLF 400 2000 240 800")
        real = out["real"]
        self.assertEqual(real["header"]["dt_ms"], 8)
        self.assertEqual(len(real["cols"]["t_ms"]), real["header"]["n"])
        for name in ("left_mmps", "yaw_unwrapped_deg", "x_mm", "y_mm", "segment", "ev"):
            self.assertIn(name, real["cols"])
        json.dumps(real, allow_nan=False)  # NaN goes out as null, never NaN

    def test_no_trace_is_an_error(self):
        with self.assertRaises(ValueError):
            server.parse_log("hello\n", 0)
        text = server.FIXTURES.joinpath("path_ffrfflf_capture.log").read_text()
        with self.assertRaises(ValueError):
            server.parse_log(text, 3)


if __name__ == "__main__":
    unittest.main()
