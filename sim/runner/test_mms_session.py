"""Tests for the mms protocol handler. Run: python3 -m unittest discover -s sim/runner"""

import unittest

from maze_file import parse_text
from mms_session import MmsSession
from test_maze_file import TINY_TEXT

OPEN_2X2 = """\
o---o---o
|     G |
o   o   o
| S     |
o---o---o
"""


def drive(session, commands):
    return [session.handle(c) for c in commands]


class WallQueryTest(unittest.TestCase):
    def test_start_cell_walls_follow_heading(self):
        s = MmsSession(parse_text(TINY_TEXT))
        self.assertEqual(drive(s, ["wallFront", "wallLeft", "wallRight"]), ["false", "true", "true"])
        self.assertEqual(s.handle("turnRight"), "ack")
        self.assertEqual(drive(s, ["wallFront", "wallLeft", "wallRight"]), ["true", "false", "true"])
        self.assertIsNone(s.failure)

    def test_maze_size_queries(self):
        s = MmsSession(parse_text(TINY_TEXT))
        self.assertEqual(drive(s, ["mazeWidth", "mazeHeight"]), ["3", "3"])


class MoveTest(unittest.TestCase):
    def test_move_into_wall_crashes_and_stays_put(self):
        s = MmsSession(parse_text(TINY_TEXT))
        s.handle("turnRight")
        self.assertEqual(s.handle("moveForward"), "crash")
        self.assertEqual((s.cell, s.heading), ((0, 0), "e"))
        self.assertIn("crash", s.failure)

    def test_multi_cell_move_through_a_wall_crashes(self):
        s = MmsSession(parse_text(TINY_TEXT))
        self.assertEqual(s.handle("moveForward 3"), "crash")
        self.assertEqual(s.cell, (0, 0))

    def test_open_move_acks_and_advances(self):
        s = MmsSession(parse_text(TINY_TEXT))
        self.assertEqual(s.handle("moveForward 2"), "ack")
        self.assertEqual((s.cell, s.heading), ((0, 2), "n"))
        self.assertIsNone(s.failure)

    def test_diagonal_half_steps_cut_the_corner(self):
        s = MmsSession(parse_text(OPEN_2X2))
        replies = drive(s, ["moveForwardHalf", "turnRight45", "moveForwardHalf",
                            "turnRight45", "moveForwardHalf"])
        self.assertEqual(replies, ["ack"] * 5)
        self.assertEqual((s.cell, s.heading), ((1, 1), "e"))

    def test_diagonal_from_cell_centre_hits_a_post(self):
        s = MmsSession(parse_text(OPEN_2X2))
        s.handle("turnRight45")
        self.assertEqual(s.handle("moveForwardHalf"), "crash")


class CommandTest(unittest.TestCase):
    def test_display_commands_get_no_reply(self):
        s = MmsSession(parse_text(TINY_TEXT))
        replies = drive(s, ["setWall 0 0 n", "clearWall 0 0 n", "setColor 1 1 y",
                            "clearColor 1 1", "clearAllColor", "setText 0 0 hi",
                            "clearText 0 0", "clearAllText"])
        self.assertEqual(replies, [None] * 8)
        self.assertIsNone(s.failure)

    def test_unknown_command_is_a_failure(self):
        s = MmsSession(parse_text(TINY_TEXT))
        self.assertIsNone(s.handle("arcTurnLeft90"))
        self.assertIn("arcTurnLeft90", s.failure)


class PhaseTest(unittest.TestCase):
    # Shortest route in TINY_TEXT: N N E S E N, three cells' worth of turns.
    ROUTE = ["moveForward 2", "turnRight", "moveForward", "turnRight",
             "moveForward", "turnLeft", "moveForward", "turnLeft", "moveForward"]
    BACK = ["turnLeft", "turnLeft", "moveForward", "turnRight", "moveForward",
            "turnRight", "moveForward", "turnLeft", "moveForward", "turnLeft",
            "moveForward 2"]

    def test_search_return_and_speed_run_are_scored(self):
        s = MmsSession(parse_text(TINY_TEXT))
        drive(s, self.ROUTE)
        self.assertEqual(s.phase, "return")
        drive(s, self.BACK)
        self.assertEqual(s.phase, "speed")
        # The return run ends facing south, so the speed run opens with a 180.
        drive(s, ["turnRight", "turnRight"] + self.ROUTE)
        self.assertEqual(s.phase, "done")
        r = s.report()
        self.assertTrue(r["search_reached"])
        self.assertTrue(r["return_reached"])
        self.assertTrue(r["speed_reached"])
        self.assertEqual(r["cells_searched"], 7)
        self.assertEqual(r["search_moves"], 6)
        self.assertEqual(r["speed_path_cells"], 6)
        self.assertEqual(r["speed_turns"], 6)
        self.assertEqual(r["speed_route_cells"], 6)
        # The return run retraces the search route, so nothing new is explored.
        self.assertEqual(r["cells_explored"], 7)

    def test_cells_explored_counts_search_and_return(self):
        s = MmsSession(parse_text(OPEN_2X2))
        drive(s, ["moveForward", "turnRight", "moveForward"])  # (0,1) then goal (1,1)
        drive(s, ["turnRight", "moveForward", "turnRight", "moveForward"])  # (1,0), start
        r = s.report()
        self.assertEqual(r["cells_searched"], 3)
        self.assertEqual(r["cells_explored"], 4)

    def test_diagonal_speed_run_is_measured_in_cells(self):
        s = MmsSession(parse_text(OPEN_2X2))
        drive(s, ["moveForward", "turnRight", "moveForward"])  # search to (1, 1)
        drive(s, ["turnRight", "moveForward", "turnRight", "moveForward"])  # back to start
        self.assertEqual(s.phase, "speed")
        drive(s, ["turnRight", "moveForwardHalf", "turnRight45", "moveForwardHalf",
                  "turnRight45", "moveForwardHalf"])
        self.assertEqual(s.phase, "done")
        # Two straight half cells plus one diagonal half cell, 0.5 * sqrt(2).
        self.assertEqual(s.report()["speed_path_cells"], 1.707)
        self.assertEqual(s.report()["speed_turns"], 3)
        # The diagonal cuts through (0,0), (0,1) and (1,1): a two-cell route.
        self.assertEqual(s.report()["speed_route_cells"], 2)

    def test_unfinished_run_is_reported_as_not_reached(self):
        s = MmsSession(parse_text(TINY_TEXT))
        s.handle("moveForward")
        r = s.report()
        self.assertFalse(r["search_reached"])
        self.assertFalse(r["speed_reached"])


if __name__ == "__main__":
    unittest.main()
