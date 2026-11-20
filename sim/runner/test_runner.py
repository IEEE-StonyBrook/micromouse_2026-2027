"""End-to-end tests: drive real and fake bots through the runner.

Run: make -C sim && python3 -m unittest discover -s sim/runner
"""

import os
import sys
import tempfile
import unittest

from maze_file import parse_text
from runner import (compare, compare_planners, maze_setting, run_maze, speed_route_estimate,
                    summarize)
from test_maze_file import TINY_TEXT

HERE = os.path.dirname(os.path.abspath(__file__))
BOT = os.path.join(HERE, "..", "bin", "Micromouse.out")


def fake_bot(tmp, body):
    """A stand-in bot: a Python script given the maze setting as argv[1]."""
    path = os.path.join(tmp, "bot.py")
    with open(path, "w") as f:
        f.write("import sys\n" + body)
    return [sys.executable, path]


class MazeSettingTest(unittest.TestCase):
    def test_contest_centre_becomes_a_rectangle(self):
        maze = parse_text(TINY_TEXT)
        maze.goals = [(1, 1), (2, 1), (1, 2), (2, 2)]
        self.assertEqual(maze_setting(maze), "3 1,1..2,2")

    def test_single_goal(self):
        self.assertEqual(maze_setting(parse_text(TINY_TEXT)), "3 2,2")


class FailureTest(unittest.TestCase):
    def test_bot_that_hangs_times_out(self):
        with tempfile.TemporaryDirectory() as tmp:
            bot = fake_bot(tmp, "import time\ntime.sleep(60)\n")
            result = run_maze(bot, parse_text(TINY_TEXT), timeout_s=1)
        self.assertFalse(result["ok"])
        self.assertIn("timeout", result["error"])

    def test_bot_that_drives_into_a_wall_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            bot = fake_bot(tmp, "print('turnRight', flush=True)\ninput()\n"
                                "print('moveForward', flush=True)\ninput()\n")
            result = run_maze(bot, parse_text(TINY_TEXT), timeout_s=10)
        self.assertFalse(result["ok"])
        self.assertIn("crash", result["error"])

    def test_bot_that_quits_early_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            bot = fake_bot(tmp, "print('moveForward', flush=True)\ninput()\n")
            result = run_maze(bot, parse_text(TINY_TEXT), timeout_s=10)
        self.assertFalse(result["ok"])
        self.assertIn("search", result["error"])


class BaselineTest(unittest.TestCase):
    BASE = {"m.txt": {"cells_searched": 50, "search_moves": 60,
                      "speed_path_cells": 20.5, "speed_turns": 8}}

    def run_result(self, **changes):
        r = dict(self.BASE["m.txt"], maze="m.txt", ok=True)
        r.update(changes)
        return r

    def test_equal_or_better_runs_are_quiet(self):
        self.assertEqual(compare([self.run_result(speed_path_cells=19.0)], self.BASE), [])

    def test_longer_speed_run_warns(self):
        warnings = compare([self.run_result(speed_path_cells=21.0)], self.BASE)
        self.assertEqual(len(warnings), 1)
        self.assertIn("speed_path_cells", warnings[0])

    def test_new_maze_warns(self):
        self.assertIn("not in the baseline",
                      compare([self.run_result(maze="new.txt")], self.BASE)[0])


class SummaryTest(unittest.TestCase):
    def test_counts_optimal_runs_and_means(self):
        results = [
            {"ok": True, "speed_optimal": True, "cells_searched": 10, "cells_explored": 20},
            {"ok": True, "speed_optimal": False, "cells_searched": 30, "cells_explored": 40},
            {"ok": False, "error": "crash"},
        ]
        s = summarize(results)
        self.assertEqual((s["optimal"], s["scored"]), (1, 2))
        self.assertEqual(s["optimal_rate"], 0.5)
        self.assertEqual(s["mean_cells_searched"], 20)
        self.assertEqual(s["mean_cells_explored"], 30)


class SpeedRouteEstimateTest(unittest.TestCase):
    def test_reads_the_last_speed_route_line(self):
        log = ("[INFO] Speed route: planner=length cells=9 est_run_time_s=7.0000\n"
               "[INFO] Speed route: planner=time cells=12 est_run_time_s=5.2500\n"
               "[INFO] Speed run complete\n")
        self.assertEqual(speed_route_estimate(log), 5.25)

    def test_no_line_gives_none(self):
        self.assertIsNone(speed_route_estimate("[INFO] Reached goal\n"))


class ComparePlannersTest(unittest.TestCase):
    def result(self, maze, est, ok=True):
        return {"maze": maze, "ok": ok, "error": None if ok else "crashed",
                "est_run_time_s": est}

    def test_faster_or_equal_time_planner_passes(self):
        rows, errors = compare_planners([self.result("a", 10.0), self.result("b", 4.0)],
                                        [self.result("a", 8.0), self.result("b", 4.0)])
        self.assertEqual(errors, [])
        self.assertAlmostEqual(rows[0]["improvement"], 0.2)
        self.assertAlmostEqual(rows[1]["improvement"], 0.0)

    def test_slower_time_planner_fails(self):
        _, errors = compare_planners([self.result("a", 8.0)], [self.result("a", 8.5)])
        self.assertEqual(len(errors), 1)
        self.assertIn("slower", errors[0])

    def test_invalid_route_fails(self):
        _, errors = compare_planners([self.result("a", 8.0)],
                                     [self.result("a", None, ok=False)])
        self.assertEqual(len(errors), 1)
        self.assertIn("time planner", errors[0])

    def test_missing_estimate_fails(self):
        _, errors = compare_planners([self.result("a", None)], [self.result("a", 8.0)])
        self.assertEqual(len(errors), 1)
        self.assertIn("estimate", errors[0])


@unittest.skipUnless(os.path.exists(BOT), "build the sim bot first: make -C sim")
class SimBotTest(unittest.TestCase):
    def test_bot_solves_tiny_maze(self):
        result = run_maze([BOT], parse_text(TINY_TEXT), timeout_s=30)
        self.assertTrue(result["ok"], result["error"])
        # The only route is N N E S E N, six cells walked square; cutting the
        # zigzag diagonally can only shorten it.
        self.assertLessEqual(result["speed_path_cells"], 6)
        self.assertGreaterEqual(result["cells_searched"], 7)
        self.assertEqual(result["optimum_cells"], 6)
        self.assertTrue(result["speed_optimal"])

    def test_both_planners_solve_tiny_maze(self):
        for planner in ("length", "time"):
            result = run_maze([BOT, "--planner", planner], parse_text(TINY_TEXT), timeout_s=30)
            self.assertTrue(result["ok"], result["error"])
            self.assertGreater(result["est_run_time_s"], 0)


if __name__ == "__main__":
    unittest.main()
