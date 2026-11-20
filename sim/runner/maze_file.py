"""Load micromouse maze files in the text (map) and num formats.

Cells use the firmware's convention: (0, 0) is the start corner, x grows east
and y grows north. Walls are stored once per cell side and mirrored onto the
neighbouring cell, so a wall always reads the same from both sides.
"""

import os

DIRECTIONS = "nesw"
_STEP = {"n": (0, 1), "e": (1, 0), "s": (0, -1), "w": (-1, 0)}
_OPPOSITE = {"n": "s", "e": "w", "s": "n", "w": "e"}


class Maze:
    def __init__(self, width, height):
        if width < 1 or height < 1:
            raise ValueError("maze must be at least 1x1")
        self.width = width
        self.height = height
        self._walls = {(x, y): set() for x in range(width) for y in range(height)}
        self.goals = []
        self.start = (0, 0)
        for x in range(width):
            self.set_wall(x, 0, "s")
            self.set_wall(x, height - 1, "n")
        for y in range(height):
            self.set_wall(0, y, "w")
            self.set_wall(width - 1, y, "e")

    def inside(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def set_wall(self, x, y, d):
        self._walls[(x, y)].add(d)
        dx, dy = _STEP[d]
        if self.inside(x + dx, y + dy):
            self._walls[(x + dx, y + dy)].add(_OPPOSITE[d])

    def wall(self, x, y, d):
        return d in self._walls[(x, y)]


def centre_goals(width, height):
    """Contest goal: the centre 2x2 block on an even maze, the centre cell on an odd one."""

    def middle(n):
        return [n // 2 - 1, n // 2] if n % 2 == 0 else [n // 2]

    return [(x, y) for x in middle(width) for y in middle(height)]


def parse_text(text):
    """Parse the ASCII format: posts every 4 columns and 2 rows, any non-space is a wall.

    A cell centre marked G is a goal and S the start. Without G marks the goal is
    the maze centre, as in the classic contest.
    """
    lines = [line.rstrip("\r") for line in text.splitlines() if line.strip()]
    if len(lines) < 3 or len(lines) % 2 == 0:
        raise ValueError("text maze needs an odd number (>= 3) of non-blank lines")
    height = (len(lines) - 1) // 2
    post_len = len(lines[0])
    if (post_len - 1) % 4 != 0 or post_len < 5:
        raise ValueError("text maze rows must be 4*width+1 characters wide")
    width = (post_len - 1) // 4
    for i, line in enumerate(lines):
        if i % 2 == 0 and len(line) != post_len:
            raise ValueError("post row %d is %d wide, expected %d" % (i, len(line), post_len))
    lines = [line.ljust(post_len) for line in lines]

    maze = Maze(width, height)
    for y in range(height):
        row = 2 * (height - 1 - y) + 1  # text row of this cell's centre; north is up
        for x in range(width):
            col = 4 * x + 2
            if lines[row - 1][col] != " ":
                maze.set_wall(x, y, "n")
            if lines[row][col - 2] != " ":
                maze.set_wall(x, y, "w")
            mark = lines[row][col]
            if mark in "Gg":
                maze.goals.append((x, y))
            elif mark in "Ss":
                maze.start = (x, y)
    if not maze.goals:
        maze.goals = centre_goals(width, height)
    return maze


def parse_num(text):
    """Parse the num format: one `x y N E S W` line per cell, 1 marking a wall."""
    cells = {}
    for number, line in enumerate(text.splitlines(), 1):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 6:
            raise ValueError("num line %d: expected 6 fields" % number)
        x, y, *walls = (int(f) for f in fields)
        cells[(x, y)] = walls
    if not cells:
        raise ValueError("num maze is empty")
    width = max(x for x, _ in cells) + 1
    height = max(y for _, y in cells) + 1
    missing = [(x, y) for x in range(width) for y in range(height) if (x, y) not in cells]
    if missing:
        raise ValueError("num maze is missing cells, e.g. %s" % (missing[0],))
    maze = Maze(width, height)
    for (x, y), walls in cells.items():
        for d, present in zip(DIRECTIONS, walls):
            if present:
                maze.set_wall(x, y, d)
    maze.goals = centre_goals(width, height)
    return maze


def load_maze(path):
    """Load a maze, choosing the parser by extension: .num, else the text format."""
    with open(path) as f:
        text = f.read()
    if os.path.splitext(path)[1].lower() == ".num":
        return parse_num(text)
    return parse_text(text)
