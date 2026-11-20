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
"""

import argparse
import concurrent.futures
import csv
import json
import os
import subprocess
import sys
import tempfile
import threading

from maze_file import load_maze
from mms_session import MmsSession

DEFAULT_TIMEOUT_S = 60
# Fields kept in the baseline; lower is better for every one of them.
BASELINE_FIELDS = ["cells_searched", "search_moves", "speed_path_cells", "speed_turns"]
CSV_FIELDS = ["maze", "ok", "error", "search_reached", "cells_searched", "search_moves",
              "search_turns", "return_reached", "return_moves", "speed_reached",
              "speed_path_cells", "speed_turns", "commands"]


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
        log_tail = log.read()[-2000:]

    result = session.report()
    result["commands"] = session.commands
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
    args = parser.parse_args(argv)

    results = run_corpus([args.bot], args.mazes, args.timeout, args.jobs)
    if not results:
        print("no mazes found under %s" % args.mazes, file=sys.stderr)
        return 1

    print("%-40s %-4s %8s %8s %8s %6s" % ("maze", "ok", "searched", "moves", "speed", "turns"))
    for r in results:
        print("%-40s %-4s %8s %8s %8s %6s" % (
            r["maze"], "ok" if r["ok"] else "FAIL", r.get("cells_searched", "-"),
            r.get("search_moves", "-"), r.get("speed_path_cells", "-"),
            r.get("speed_turns", "-")))
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

    print("%d/%d mazes passed" % (len(results) - len(failures), len(results)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
