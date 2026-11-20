# Maze corpus

The headless runner (`sim/runner/runner.py`) runs the sim bot over every maze
here. Its CI job fails if any maze fails, and warns when a run is worse than
`baseline.json`.

## classic/

45 16x16 contest and test mazes copied unchanged from
[micromouseonline/micromouse_maze_tool](https://github.com/micromouseonline/micromouse_maze_tool),
`mazefiles/text/`, at commit `12118e413f6be8341ea7f64c100e7625178cd6dc`.
They are © 2016 micromouseonline under the MIT licence in
[`classic/LICENSE`](classic/LICENSE). These files carry no goal marks, so the
goal is the centre 2x2 block, (7,7)..(8,8).

Training mazes from that collection whose small active area leaves the centre
unreachable were left out on purpose.

## small/

Our own small mazes. `3x3-tiny` is the parser and protocol test maze, in both
the text and num formats; the text copy marks the top-right cell as the goal,
while the num format has no goal marks, so the num copy uses the centre cell.
`5x5-a`, `8x8-a` and `8x8-b` were generated from fixed random seeds (a
depth-first maze with a few extra openings for loops and an open centre goal
block), with G marking the goal cells.

## Formats

- Text (`.txt`, `.map`): posts every 4 columns and 2 rows, north at the top;
  any non-space character at a wall position is a wall. `G` and `S` in a cell
  centre mark goal and start cells; without `G` the goal is the centre.
- Num (`.num`): one `x y N E S W` line per cell, `1` for a wall; the goal is
  the centre.

## Updating the baseline

After a planner change that is meant to change the numbers:

```bash
make -C sim
python3 sim/runner/runner.py --write-baseline sim/mazes/baseline.json
```

## Comparing route planners

The speed run picks its route by fewest cells (`--planner length`, the
default) or by least estimated run time (`--planner time`): merged straights
as rest-to-rest trapezoidal moves at the fast-run speed and acceleration, and
90 degree spins at the spin-turn limits, all from `config/motion.h`. The
comparison mode runs the corpus under both and fails unless both pass every
maze and the time planner's route is never estimated slower:

```bash
make -C sim
python3 sim/runner/runner.py --compare-planners
```
