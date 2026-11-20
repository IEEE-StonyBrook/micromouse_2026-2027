"""End-to-end tests: drive real and fake bots through the runner.

Run: make -C sim && python3 -m unittest discover -s sim/runner
"""

import os
import sys
import tempfile
import unittest

from maze_file import parse_text
from runner import maze_setting, run_maze
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


@unittest.skipUnless(os.path.exists(BOT), "build the sim bot first: make -C sim")
class SimBotTest(unittest.TestCase):
    def test_bot_solves_tiny_maze(self):
        result = run_maze([BOT], parse_text(TINY_TEXT), timeout_s=30)
        self.assertTrue(result["ok"], result["error"])
        # The only route is N N E S E N, six cells walked square; cutting the
        # zigzag diagonally can only shorten it.
        self.assertLessEqual(result["speed_path_cells"], 6)
        self.assertGreaterEqual(result["cells_searched"], 7)


if __name__ == "__main__":
    unittest.main()
