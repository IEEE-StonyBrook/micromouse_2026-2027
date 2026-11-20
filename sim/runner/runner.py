#!/usr/bin/env python3
"""Run the sim bot headless over a maze corpus and score each run.

Each maze gets a fresh bot process that speaks the mms protocol over
stdin/stdout to an MmsSession. A maze passes when the bot completes its
search run to the goal, its return run to the start and its speed run to the
goal without crashing, within the time limit, and then exits cleanly.

    python3 sim/runner/runner.py --bot sim/bin/Micromouse.out \\
        --mazes sim/mazes --baseline sim/mazes/baseline.json --json results.json

Exits 1 if any maze fails. A speed run longer than the baseline's, or more
cells searched, prints a warning but does not fail.

Each run is also scored against the maze's true optimum, the shortest
start-to-goal route with every wall known, in the cells-entered cost the bot's
default (length) speed-run planner minimises. `--min-optimal` fails the corpus when too few
speed runs match it.

--planner length|time picks the bot's speed-run route planner. With
--compare-planners the runner runs the corpus under both planners instead and
exits 1 unless both pass every maze and the time planner's estimated run time
is never above the length planner's.
"""

import argparse
import concurrent.futures
import csv
import json
import os
import re
import statistics
import subprocess
import sys
import tempfile
import threading

from maze_file import load_maze, shortest_route_cells
from mms_session import MmsSession

DEFAULT_TIMEOUT_S = 60
PLANNERS = ("length", "time")
# The bot logs its estimates to 4 decimal places.
ESTIMATE_TOLERANCE_S = 1e-3
SPEED_ROUTE = re.compile(r"Speed route: planner=\S+ cells=\d+ est_run_time_s=([0-9.]+)")
# Fields kept in the baseline; lower is better for every one of them.
BASELINE_FIELDS = ["cells_searched", "cells_explored", "search_moves", "speed_path_cells",
                   "speed_turns"]
CSV_FIELDS = ["maze", "ok", "error", "search_reached", "cells_searched", "cells_explored",
              "search_moves", "search_turns", "return_reached", "return_moves", "speed_reached",
              "speed_path_cells", "speed_turns", "speed_route_cells", "optimum_cells",
              "speed_optimal", "est_run_time_s", "commands"]


def speed_route_estimate(log):
    """The estimated run time (s) of the bot's last planned speed route, or None."""
    found = SPEED_ROUTE.findall(log)
    return float(found[-1]) if found else None


def maze_setting(maze):
    """The bot's `<size> <goals>` argument (the firmware MAZE text) for a maze."""
    if maze.width != maze.height:
        raise ValueError("the bot only supports square mazes")
    if maze.start != (0, 0):
        raise ValueError("the bot always starts at (0, 0)")
    xs = [x for x, _ in maze.goals]
    ys = [y for _, y in maze.goals]
    lo, hi = (min(xs), min(ys)), (max(xs), max(ys))
    block = {(x, y) for x in range(lo[0], hi[0] + 1) for y in range(lo[1], hi[1] + 1)}
    if block != set(maze.goals):
        raise ValueError("goal cells must form one rectangle")
    goals = "%d,%d" % lo if lo == hi else "%d,%d..%d,%d" % (lo + hi)
    return "%d %s" % (maze.width, goals)


def run_maze(bot, maze, timeout_s=DEFAULT_TIMEOUT_S):
    """Run one bot process on one maze; return the session report plus ok/error."""
    session = MmsSession(maze)
    timed_out = threading.Event()
    with tempfile.TemporaryFile("w+") as log:
        proc = subprocess.Popen(
            list(bot) + [maze_setting(maze)], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=log, text=True, bufsize=1)

        def expire():
            timed_out.set()
            proc.kill()

        timer = threading.Timer(timeout_s, expire)
        timer.start()
        try:
            for line in proc.stdout:
                reply = session.handle(line.strip())
                if session.failure:
                    break
                if reply is not None:
                    proc.stdin.write(reply + "\n")
                    proc.stdin.flush()
        except (BrokenPipeError, OSError):
            pass  # The bot exited mid-reply; its exit status tells the story.
        finally:
            timer.cancel()
            if proc.poll() is None and (session.failure or timed_out.is_set()):
                proc.kill()
            for stream in (proc.stdin, proc.stdout):
                try:
                    stream.close()
                except OSError:
                    pass
            returncode = proc.wait()
        log.seek(0)
        log_text = log.read()
        log_tail = log_text[-2000:]

    result = session.report()
    result["est_run_time_s"] = speed_route_estimate(log_text)
    result["commands"] = session.commands
    result["optimum_cells"] = shortest_route_cells(maze)
    result["speed_optimal"] = (result["speed_reached"]
                               and result["speed_route_cells"] == result["optimum_cells"])
    error = None
    if timed_out.is_set():
        error = "timeout after %ss in the %s phase" % (timeout_s, session.phase)
    elif session.failure:
        error = session.failure
    elif not result["search_reached"]:
        error = "search run never reached the goal"
    elif not result["return_reached"]:
        error = "return run never reached the start"
    elif not result["speed_reached"]:
        error = "speed run never reached the goal"
    elif returncode != 0:
        error = "bot exited with status %d" % returncode
    result["ok"] = error is None
    result["error"] = error
    if error:
        result["log_tail"] = log_tail
    return result


def find_mazes(root):
    paths = []
    for directory, _, files in os.walk(root):
        for name in files:
            if name.endswith((".txt", ".num", ".map")):
                paths.append(os.path.join(directory, name))
    return sorted(paths)


def run_corpus(bot, root, timeout_s, jobs):
    """Run bot (an argv list) on every maze under root."""
    paths = find_mazes(root)

    def one(path):
        name = os.path.relpath(path, root)
        try:
            result = run_maze(bot, load_maze(path), timeout_s)
        except (OSError, ValueError) as exc:
            result = {"ok": False, "error": "%s: %s" % (type(exc).__name__, exc)}
        result["maze"] = name
        return result

    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        return list(pool.map(one, paths))


def compare(results, baseline):
    """Warnings for runs that got worse than the baseline, and for unknown mazes."""
    warnings = []
    for r in results:
        base = baseline.get(r["maze"])
        if base is None:
            warnings.append("%s: not in the baseline" % r["maze"])
            continue
        if not r["ok"]:
            continue
        for field in BASELINE_FIELDS:
            if field in base and r[field] > base[field]:
                warnings.append("%s: %s %s, baseline %s"
                                % (r["maze"], field, r[field], base[field]))
    return warnings


def summarize(results):
    """Corpus totals over the mazes that passed: optimal speed runs and mean cells."""
    passed = [r for r in results if r["ok"]]
    n = len(passed)
    optimal = sum(1 for r in passed if r["speed_optimal"])
    return {
        "scored": n,
        "optimal": optimal,
        "optimal_rate": optimal / n if n else 0.0,
        "mean_cells_searched": sum(r["cells_searched"] for r in passed) / n if n else 0.0,
        "mean_cells_explored": sum(r["cells_explored"] for r in passed) / n if n else 0.0,
    }


def compare_planners(length_results, time_results):
    """Pair each maze's runs under both planners.

    Returns (rows, errors): a row per maze with both estimates and the time
    planner's fractional improvement, and an error for every failed run,
    missing estimate, or time-planner route estimated slower than the
    length-planner route.
    """
    by_maze = {r["maze"]: r for r in time_results}
    rows, errors = [], []
    for length in length_results:
        maze = length["maze"]
        timed = by_maze.get(maze)
        bad = False
        for name, r in (("length", length), ("time", timed)):
            if r is None or not r["ok"]:
                errors.append("%s: %s planner run failed: %s"
                              % (maze, name, "missing" if r is None else r["error"]))
                bad = True
            elif r.get("est_run_time_s") is None:
                errors.append("%s: %s planner logged no run time estimate" % (maze, name))
                bad = True
        if bad:
            continue
        t_length, t_time = length["est_run_time_s"], timed["est_run_time_s"]
        if t_time > t_length + ESTIMATE_TOLERANCE_S:
            errors.append("%s: time planner route is slower: %.4f s vs %.4f s"
                          % (maze, t_time, t_length))
        rows.append({"maze": maze, "length_s": t_length, "time_s": t_time,
                     "improvement": (t_length - t_time) / t_length if t_length else 0.0})
    return rows, errors


def run_planner_comparison(bot, root, timeout_s, jobs):
    results = {p: run_corpus(bot + ["--planner", p], root, timeout_s, jobs) for p in PLANNERS}
    rows, errors = compare_planners(results["length"], results["time"])
    print("%-40s %10s %10s %8s" % ("maze", "length_s", "time_s", "gain"))
    for row in rows:
        print("%-40s %10.3f %10.3f %7.1f%%" % (row["maze"], row["length_s"], row["time_s"],
                                               100 * row["improvement"]))
    if rows:
        gains = [row["improvement"] for row in rows]
        print("mean estimated run time improvement %.2f%% (max %.2f%%), "
              "%d/%d mazes faster, total %.1f s -> %.1f s"
              % (100 * statistics.mean(gains), 100 * max(gains),
                 sum(g > 0 for g in gains), len(rows),
                 sum(row["length_s"] for row in rows), sum(row["time_s"] for row in rows)))
    for error in errors:
        print("::error::" + error)
    return rows, errors


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bot", default=os.path.join(here, "..", "bin", "Micromouse.out"),
                        help="bot executable (default: sim/bin/Micromouse.out)")
    parser.add_argument("--mazes", default=os.path.join(here, "..", "mazes"),
                        help="directory searched for .txt/.map/.num mazes")
    parser.add_argument("--baseline", help="baseline JSON to compare against")
    parser.add_argument("--write-baseline", metavar="PATH",
                        help="write this run's results as the new baseline")
    parser.add_argument("--json", metavar="PATH", help="write full results as JSON")
    parser.add_argument("--csv", metavar="PATH", help="write results as CSV")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help="seconds allowed per maze (default %(default)s)")
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    parser.add_argument("--min-optimal", type=float, metavar="FRACTION",
                        help="fail unless this fraction of passing mazes get an optimal speed run")
    parser.add_argument("--planner", choices=PLANNERS,
                        help="speed-run route planner (default: the bot's, length)")
    parser.add_argument("--compare-planners", action="store_true",
                        help="run both planners and check time never loses to length")
    args = parser.parse_args(argv)

    if args.compare_planners:
        rows, errors = run_planner_comparison([args.bot], args.mazes, args.timeout, args.jobs)
        return 1 if errors or not rows else 0

    bot = [args.bot] + (["--planner", args.planner] if args.planner else [])
    results = run_corpus(bot, args.mazes, args.timeout, args.jobs)
    if not results:
        print("no mazes found under %s" % args.mazes, file=sys.stderr)
        return 1

    print("%-40s %-4s %8s %8s %8s %8s %6s %8s" % (
        "maze", "ok", "searched", "explored", "moves", "speed", "turns", "route"))
    for r in results:
        route = "-"
        if r.get("speed_reached"):
            route = "%s/%s%s" % (r["speed_route_cells"], r["optimum_cells"],
                                 "" if r["speed_optimal"] else "*")
        print("%-40s %-4s %8s %8s %8s %8s %6s %8s" % (
            r["maze"], "ok" if r["ok"] else "FAIL", r.get("cells_searched", "-"),
            r.get("cells_explored", "-"), r.get("search_moves", "-"),
            r.get("speed_path_cells", "-"), r.get("speed_turns", "-"), route))
    failures = [r for r in results if not r["ok"]]
    for r in failures:
        print("::error::%s: %s" % (r["maze"], r["error"]))
        if r.get("log_tail"):
            print("    bot log tail:\n    " + r["log_tail"].replace("\n", "\n    "))

    if args.baseline:
        with open(args.baseline) as f:
            for warning in compare(results, json.load(f)):
                print("::warning::" + warning)
    if args.write_baseline:
        baseline = {r["maze"]: {k: r[k] for k in BASELINE_FIELDS}
                    for r in results if r["ok"]}
        with open(args.write_baseline, "w") as f:
            json.dump(baseline, f, indent=2, sort_keys=True)
            f.write("\n")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=2)
            f.write("\n")
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)

    summary = summarize(results)
    print("speed run optimal on %d/%d mazes (%.1f%%); mean cells searched %.1f, explored %.1f"
          % (summary["optimal"], summary["scored"], 100 * summary["optimal_rate"],
             summary["mean_cells_searched"], summary["mean_cells_explored"]))
    print("%d/%d mazes passed" % (len(results) - len(failures), len(results)))
    if args.min_optimal is not None and summary["optimal_rate"] < args.min_optimal:
        print("::error::speed run optimal on %.1f%% of mazes, below %.1f%%"
              % (100 * summary["optimal_rate"], 100 * args.min_optimal))
        return 1
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
