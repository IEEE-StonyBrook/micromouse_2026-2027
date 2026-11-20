"""A headless stand-in for the mms simulator: answers one mouse's commands.

Implements the subset of the mms mouse API (https://github.com/mackorone/mms)
that the sim bot uses, from the README's command semantics. The mouse pose is
kept in half-cell units so 45-degree turns and half steps work:

- a cell centre (x, y) is the point (2x + 1, 2y + 1);
- a point with one even coordinate is the middle of a cell edge, which the
  mouse may only occupy when that wall is absent;
- a point with both coordinates even is a post, which the mouse never reaches.

A diagonal half step therefore runs from one edge midpoint to the next through
the corner of a cell, and a diagonal step from a cell centre hits a post.

The session also scores the run in the bot's three phases: the search run
ends on first reaching a goal cell, the return run on reaching the start cell
again, and the speed run on reaching a goal cell after that.
"""

HEADINGS = ["n", "ne", "e", "se", "s", "sw", "w", "nw"]
_VECTOR = {
    "n": (0, 1), "ne": (1, 1), "e": (1, 0), "se": (1, -1),
    "s": (0, -1), "sw": (-1, -1), "w": (-1, 0), "nw": (-1, 1),
}
# 45-degree steps clockwise for each turn command.
_TURNS = {
    "turnRight": 2, "turnRight90": 2, "turnLeft": -2, "turnLeft90": -2,
    "turnRight45": 1, "turnLeft45": -1,
}
# Wall query → 45-degree steps from the heading to the side being asked about.
_WALL_QUERIES = {"wallFront": 0, "wallRight": 2, "wallBack": 4, "wallLeft": -2}
_NO_REPLY = {"setWall", "clearWall", "setColor", "clearColor", "clearAllColor",
             "setText", "clearText", "clearAllText"}
PHASES = ["search", "return", "speed", "done"]


class _PhaseStats:
    def __init__(self):
        self.half_steps = 0
        self.turns = 0
        self.cells = set()


class MmsSession:
    def __init__(self, maze):
        self.maze = maze
        sx, sy = maze.start
        self._pos = (2 * sx + 1, 2 * sy + 1)
        self._heading = 0
        self.phase = "search"
        self.failure = None
        self.commands = 0
        self._stats = {p: _PhaseStats() for p in PHASES}
        self._stats["search"].cells.add(maze.start)

    @property
    def heading(self):
        return HEADINGS[self._heading]

    @property
    def cell(self):
        """The cell whose centre the mouse is on, or None between centres."""
        px, py = self._pos
        if px % 2 and py % 2:
            return ((px - 1) // 2, (py - 1) // 2)
        return None

    def handle(self, line):
        """Apply one command line; return the reply, or None for no reply."""
        self.commands += 1
        parts = line.split()
        if not parts:
            return self._fail("empty command")
        name, args = parts[0], parts[1:]
        if name in _NO_REPLY:
            return None
        if name == "mazeWidth":
            return str(self.maze.width)
        if name == "mazeHeight":
            return str(self.maze.height)
        if name in _WALL_QUERIES:
            return self._wall_query(name, args)
        if name in _TURNS:
            self._heading = (self._heading + _TURNS[name]) % 8
            self._stats[self.phase].turns += 1
            return "ack"
        if name in ("moveForward", "moveForwardHalf"):
            count = self._count(args)
            if count is None:
                self._fail("bad step count in %r" % line)
                return "crash"
            per_step = 2 if name == "moveForward" else 1
            return self._move(count * per_step, line)
        if name == "wasReset":
            return "false"
        if name == "ackReset":
            return "ack"
        return self._fail("unsupported command %r" % line)

    def report(self):
        reached = PHASES.index(self.phase)
        s, r, f = self._stats["search"], self._stats["return"], self._stats["speed"]
        return {
            "search_reached": reached > 0,
            "cells_searched": len(s.cells),
            "search_moves": _cells(s.half_steps),
            "search_turns": s.turns,
            "return_reached": reached > 1,
            "return_moves": _cells(r.half_steps),
            "speed_reached": reached > 2,
            "speed_path_cells": _cells(f.half_steps),
            "speed_turns": f.turns,
        }

    def _fail(self, message):
        if self.failure is None:
            self.failure = message
        return None

    @staticmethod
    def _count(args):
        if not args:
            return 1
        if len(args) == 1 and args[0].isdigit() and int(args[0]) >= 1:
            return int(args[0])
        return None

    def _wall_query(self, name, args):
        if args and args != ["1"]:
            return self._fail("%s with a distance is unsupported" % name)
        cell = self.cell
        if cell is None or self._heading % 2:
            return self._fail("%s asked away from a cell centre or off a cardinal heading" % name)
        side = HEADINGS[(self._heading + _WALL_QUERIES[name]) % 8]
        return "true" if self.maze.wall(cell[0], cell[1], side) else "false"

    def _move(self, half_steps, line):
        dx, dy = _VECTOR[self.heading]
        px, py = self._pos
        path = [(px + dx * i, py + dy * i) for i in range(1, half_steps + 1)]
        for point in path:
            blocked = self._blocked(point)
            if blocked:
                self._fail("crash on %r at %s facing %s: %s"
                           % (line, self.cell or self._pos, self.heading, blocked))
                return "crash"
        stats = self._stats[self.phase]
        for point in path:
            self._pos = point
            stats.half_steps += 1
            cell = self.cell
            if cell is not None:
                stats.cells.add(cell)
                self._advance_phase(cell)
                stats = self._stats[self.phase]
        return "ack"

    def _advance_phase(self, cell):
        if self.phase in ("search", "speed") and cell in self.maze.goals:
            self.phase = "return" if self.phase == "search" else "done"
        elif self.phase == "return" and cell == self.maze.start:
            self.phase = "speed"

    def _blocked(self, point):
        """Why the mouse cannot stand on this half-cell point, or None if it can."""
        px, py = point
        w, h = self.maze.width, self.maze.height
        if not (0 < px < 2 * w and 0 < py < 2 * h):
            return "outside the maze"
        if px % 2 == 0 and py % 2 == 0:
            return "hits a post"
        if py % 2 == 0:  # horizontal edge between (x, y) and (x, y + 1)
            if self.maze.wall((px - 1) // 2, py // 2 - 1, "n"):
                return "wall"
        elif px % 2 == 0:  # vertical edge between (x, y) and (x + 1, y)
            if self.maze.wall(px // 2 - 1, (py - 1) // 2, "e"):
                return "wall"
        return None


def _cells(half_steps):
    """Half steps as cells, an int when whole."""
    return half_steps // 2 if half_steps % 2 == 0 else half_steps / 2
