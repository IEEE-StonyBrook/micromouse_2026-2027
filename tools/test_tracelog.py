"""Tests for tools/tracelog.py. Run: python3 -m unittest tools/test_tracelog.py"""

import json
import math
import pathlib
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tracelog  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent

OLD_COLS = (
    "t_ms,target_deg,yaw_deg,rot_err_deg,rot_v,hold_err_deg,wall_degps,hold_degps,"
    "fwd_mmps,left_mmps,right_mmps,left_v,right_v,enc,tof_l_mm,tof_f_mm,tof_r_mm,pos_mm,ev"
)
NEW_COLS = OLD_COLS.replace("pos_mm,ev", "pos_mm,enc_l_mm,enc_r_mm,ev")


def old_row(t, yaw, left, right, ev=32):
    return f"T,{t},0.0,{yaw},0.00,0.00,0.00,0.0,0.0,0,{left},{right},0.50,0.50,0,8191,8191,8191,0.0,{ev}"


# The current Pico format through serial_term: [ms] stamps, [INFO] tags, a
# PATH line between header and rows that is not part of the trace, and
# straight driving at 100 mm/s, sampled every 100 ms.
OLD_LOG = "\r\n".join(
    [
        "[    1000.0] >>> [serial] PATH F",
        "[    1001.0] PATH done: segments=1",
        f"[    1002.0] [INFO] TRACE n=5 dt_ms=100 cols={OLD_COLS}",
        *(f"[    {1003 + i}.0] [INFO] " + old_row(i * 100, 0.0, 100, 100) for i in range(5)),
        "[    1010.0] [INFO] TRACE end",
        "",
    ]
)


def new_row(t, yaw, enc_l, enc_r, segment):
    ev = 32 | (segment << 8)
    return (
        f"T,{t},0.0,{yaw},0.00,0.00,0.00,0.0,0.0,0,0,0,0.50,0.50,0,8191,8191,8191,0.0,"
        f"{enc_l},{enc_r},{ev}"
    )


# The extended format: header keys, cumulative encoder columns, segment bits.
# Segment 0 drives 100 mm along +x, segment 1 turns left in place (the yaw
# crosses +180 to show unwrapping), segment 2 drives 100 mm along -x.
NEW_LOG = "\n".join(
    [
        f'TRACE n=7 dt_ms=8 cols={NEW_COLS} run=3 loop_hz=500 batt_v=7.80 wrapped=0 '
        f'cmd="PATH FBF speed=600 accel=3000 omega=240 alpha=800 smooth=0 raw=0"',
        new_row(0, 0.0, 0.0, 0.0, 0),
        new_row(8, 0.0, 100.0, 100.0, 0),
        new_row(16, 90.0, 60.0, 140.0, 1),
        new_row(24, 170.0, 25.0, 175.0, 1),
        new_row(32, -175.0, 20.0, 180.0, 1),
        new_row(40, -180.0, 20.0, 180.0, 2),
        new_row(48, -180.0, 120.0, 280.0, 2),
        "TRACE end",
    ]
)


class ParseOldFormat(unittest.TestCase):
    def setUp(self):
        runs = tracelog.parse(OLD_LOG)
        self.assertEqual(len(runs), 1)
        self.run_ = runs[0]

    def test_header_and_rows_survive_the_prefixes(self):
        self.assertEqual(self.run_.header["n"], 5)
        self.assertEqual(self.run_.header["dt_ms"], 100)
        self.assertEqual(self.run_.columns, OLD_COLS.split(","))
        self.assertEqual(len(self.run_.rows), 5)
        self.assertEqual(self.run_.rows[4]["t_ms"], 400.0)
        self.assertNotIn("cmd", self.run_.header)

    def test_distance_falls_back_to_integrated_wheel_speeds(self):
        # 100 mm/s for 4 intervals of 100 ms.
        self.assertAlmostEqual(self.run_.distance_mm[-1], 40.0)
        self.assertAlmostEqual(self.run_.x_mm[-1], 40.0)
        self.assertAlmostEqual(self.run_.y_mm[-1], 0.0)

    def test_events_decode(self):
        ev = self.run_.events[0]
        self.assertTrue(ev["forward_active"])
        self.assertFalse(ev["rotation_active"])
        self.assertEqual(ev["segment"], 0)


class ParseExtendedFormat(unittest.TestCase):
    def setUp(self):
        (self.run_,) = tracelog.parse(NEW_LOG)

    def test_header_keys(self):
        h = self.run_.header
        self.assertEqual(h["run"], 3)
        self.assertEqual(h["loop_hz"], 500)
        self.assertAlmostEqual(h["batt_v"], 7.8)
        self.assertEqual(h["wrapped"], 0)
        self.assertEqual(h["cmd"], "PATH FBF speed=600 accel=3000 omega=240 alpha=800 smooth=0 raw=0")

    def test_segments_from_event_bits(self):
        self.assertEqual(self.run_.segment, [0, 0, 1, 1, 1, 2, 2])
        self.assertTrue(self.run_.events[0]["forward_active"])

    def test_yaw_unwraps_through_180(self):
        self.assertAlmostEqual(self.run_.yaw_unwrapped_deg[4], 185.0)
        self.assertAlmostEqual(self.run_.yaw_unwrapped_deg[-1], 180.0)

    def test_distance_uses_encoders_and_dead_reckons(self):
        self.assertAlmostEqual(self.run_.distance_mm[1], 100.0)
        self.assertAlmostEqual(self.run_.distance_mm[-1], 200.0)
        # Out 100 mm along +x, spin in place, back 100 mm: home again.
        self.assertAlmostEqual(self.run_.x_mm[-1], 0.0, delta=1.0)
        self.assertAlmostEqual(self.run_.y_mm[-1], 0.0, delta=10.0)

    def test_path_args_from_cmd(self):
        self.assertEqual(tracelog.path_args(self.run_), "FBF 600 3000 240 800")
        smooth = tracelog.Run({"cmd": "PATH FFLF speed=600 accel=3000 omega=240 alpha=800 smooth=1 raw=0"}, [], [])
        self.assertEqual(tracelog.path_args(smooth), "FFLF 600 3000 240 800 SMOOTH")
        self.assertIsNone(tracelog.path_args(tracelog.Run({}, [], [])))


class ParseSeveral(unittest.TestCase):
    def test_every_block_in_a_log(self):
        runs = tracelog.parse(OLD_LOG + NEW_LOG + "\n")
        self.assertEqual([len(r.rows) for r in runs], [5, 7])

    def test_a_cut_off_dump_keeps_its_rows(self):
        cut = "\n".join(NEW_LOG.splitlines()[:4])
        (run,) = tracelog.parse(cut)
        self.assertEqual(len(run.rows), 3)


class Cli(unittest.TestCase):
    def test_csv_and_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = pathlib.Path(tmp, "latest.log")
            log.write_text(OLD_LOG + NEW_LOG)
            csv_out = pathlib.Path(tmp, "out.csv")
            json_out = pathlib.Path(tmp, "out.json")
            proc = subprocess.run(
                [sys.executable, str(HERE / "tracelog.py"), str(log), "--run", "1",
                 "--csv", str(csv_out), "--json", str(json_out)],
                capture_output=True, text=True, check=True,
            )
            self.assertIn("run 1", proc.stdout)
            lines = csv_out.read_text().splitlines()
            self.assertEqual(len(lines), 8)
            self.assertTrue(lines[0].endswith("yaw_unwrapped_deg,distance_mm,x_mm,y_mm,segment"))
            data = json.loads(json_out.read_text())
            self.assertEqual(data["header"]["run"], 3)
            self.assertTrue(math.isclose(data["x_mm"][1], 100.0))


if __name__ == "__main__":
    unittest.main()
