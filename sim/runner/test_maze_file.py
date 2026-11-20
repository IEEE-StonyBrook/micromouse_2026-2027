"""Tests for maze file parsing. Run: python3 -m unittest discover -s sim/runner"""

import os
import tempfile
import unittest

from maze_file import Maze, load_maze, parse_num, parse_text

# A 3x3 maze, north at the top. (0, 0) is the bottom-left start cell, whose
# east wall forces the mouse north first; the goal G is the top-right cell,
# reached by (0,0) N N E S E N.
TINY_TEXT = """\
o---o---o---o
|       | G |
o   o   o   o
|   |       |
o   o   o---o
| S |       |
o---o---o---o
"""

# The same maze in num format: one "x y N E S W" line per cell.
TINY_NUM = """\
0 0 0 1 1 1
0 1 0 1 0 1
0 2 1 0 0 1
1 0 0 0 1 1
1 1 0 0 0 1
1 2 1 1 0 0
2 0 1 1 1 0
2 1 0 1 1 0
2 2 1 1 0 1
"""


class TextFormatTest(unittest.TestCase):
    def test_reads_size_and_walls(self):
        maze = parse_text(TINY_TEXT)
        self.assertEqual((maze.width, maze.height), (3, 3))
        # Start cell: walls on east, south and west, open to the north.
        self.assertEqual(
            [maze.wall(0, 0, d) for d in "nesw"], [False, True, True, True]
        )
        self.assertFalse(maze.wall(1, 2, "s"))
        self.assertTrue(maze.wall(0, 1, "e"))
        self.assertTrue(maze.wall(2, 2, "w"))

    def test_shared_walls_agree_from_both_sides(self):
        maze = parse_text(TINY_TEXT)
        self.assertEqual(maze.wall(1, 1, "n"), maze.wall(1, 2, "s"))
        self.assertEqual(maze.wall(0, 0, "e"), maze.wall(1, 0, "w"))

    def test_reads_goal_and_start_markers(self):
        maze = parse_text(TINY_TEXT)
        self.assertEqual(maze.goals, [(2, 2)])
        self.assertEqual(maze.start, (0, 0))

    def test_plus_posts_parse_like_o_posts(self):
        maze = parse_text(TINY_TEXT.replace("o", "+"))
        self.assertTrue(maze.wall(0, 1, "e"))
        self.assertEqual((maze.width, maze.height), (3, 3))

    def test_unmarked_16x16_defaults_to_contest_centre(self):
        row_walls = "o" + "---o" * 16 + "\n"
        row_open = "|" + "    " * 15 + "   |\n"
        text = row_walls + (row_open + "o" + "   o" * 16 + "\n") * 15
        text += row_open + row_walls
        maze = parse_text(text)
        self.assertEqual(sorted(maze.goals), [(7, 7), (7, 8), (8, 7), (8, 8)])

    def test_rejects_ragged_grid(self):
        with self.assertRaises(ValueError):
            parse_text("o---o\n|   |\no---\n")


class NumFormatTest(unittest.TestCase):
    def test_matches_text_format(self):
        text_maze = parse_text(TINY_TEXT)
        num_maze = parse_num(TINY_NUM)
        self.assertEqual((num_maze.width, num_maze.height), (3, 3))
        for x in range(3):
            for y in range(3):
                for d in "nesw":
                    self.assertEqual(
                        num_maze.wall(x, y, d), text_maze.wall(x, y, d), (x, y, d)
                    )

    def test_rejects_missing_cell(self):
        with self.assertRaises(ValueError):
            parse_num("\n".join(TINY_NUM.splitlines()[:-1]))


class LoadMazeTest(unittest.TestCase):
    def test_picks_format_by_extension(self):
        with tempfile.TemporaryDirectory() as tmp:
            num_path = os.path.join(tmp, "tiny.num")
            with open(num_path, "w") as f:
                f.write(TINY_NUM)
            self.assertIsInstance(load_maze(num_path), Maze)
            self.assertTrue(load_maze(num_path).wall(0, 0, "e"))


if __name__ == "__main__":
    unittest.main()
