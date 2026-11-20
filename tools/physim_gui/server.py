#!/usr/bin/env python3
"""Serves the physics-sim GUI and runs tests/physim for it.

    python3 tools/physim_gui/server.py [--port 8765] [--no-browser]

Builds tests/physim with make (incremental, so a fresh build only costs the
first launch), listens on 127.0.0.1 only and opens the page. GET /run takes
the scenario and knobs as query parameters, checks each against KNOBS, runs
the sim with an argv list (no shell) and returns JSON: the CSV trace, the
summary line parsed into numbers, and the argv it ran.

Standard library only.
"""

import argparse
import json
import math
import subprocess
import sys
import threading
import urllib.parse
import webbrowser
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
PHYSIM = REPO / "tests" / "physim"

SCENARIOS = ("straight10", "ip90", "ss90")

# Knob name (the CLI flag without --) -> (min, max, integer?). The ranges keep
# the plant physical and every run bounded; tests/sim/physim_main.cpp has the
# meaning and units of each flag.
KNOBS = {
    "seed": (0, 2**31 - 1, True),
    "speed": (50.0, 3000.0, False),  # mm/s
    "accel": (100.0, 20000.0, False),  # mm/s^2
    "battery": (3.0, 12.6, False),  # V
    "friction": (0.0, 3.0, False),  # V
    "mu": (0.02, 2.0, False),
    "mu-left": (0.02, 2.0, False),
    "mu-right": (0.02, 2.0, False),
    "slip-rate": (0.0, 50.0, False),  # events/s per wheel
    "slip-duration": (0.0, 1.0, False),  # s
    "slip-traction": (0.0, 1.0, False),
    "counts-per-mm": (0.2, 200.0, False),
    "gyro-noise": (0.0, 20.0, False),  # deg/s/sqrt(Hz)
    "gyro-bias": (-50.0, 50.0, False),  # deg/s
    "seam-at": (0.0, 2000.0, False),  # mm
    "seam-skew": (0.0, 100.0, False),  # mm
    "seam-duration": (0.0, 0.5, False),  # s
    "seam-traction": (0.0, 1.0, False),
    "seam-encoder": (0.0, 50.0, False),  # mm
    "seam-pitch": (0.0, 2000.0, False),  # deg/s
    "seam-accel": (0.0, 200.0, False),  # m/s^2
    "every": (1, 50, True),  # print every Nth 2 ms tick
}

DEFAULT_EVERY = 2  # 4 ms rows: smooth to animate, ~800 rows for the straight
RUN_TIMEOUT_S = 20.0


def build_args(query):
    """Turns a {name: value-string} query into a physim argv tail.

    Raises ValueError on a missing or unknown scenario, an unknown knob, or a
    value that is not a finite number inside the knob's range.
    """
    scenario = query.get("scenario")
    if scenario not in SCENARIOS:
        raise ValueError(f"scenario must be one of {', '.join(SCENARIOS)}")
    args = ["--scenario", scenario]
    if "every" not in query:
        args += ["--every", str(DEFAULT_EVERY)]
    for name, text in query.items():
        if name == "scenario":
            continue
        if name not in KNOBS:
            raise ValueError(f"unknown knob: {name}")
        lo, hi, integer = KNOBS[name]
        try:
            value = int(text) if integer else float(text)
        except (TypeError, ValueError):
            raise ValueError(f"{name}: not a number: {text!r}") from None
        if not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f"{name}: {text} is outside {lo}..{hi}")
        args += [f"--{name}", str(value)]
    return args


def parse_summary(line):
    """Parses physim's stderr summary (`key=value ...`) into numbers."""
    out = {}
    for field in line.split():
        key, sep, value = field.partition("=")
        if not sep:
            continue
        try:
            out[key] = float(value)
        except ValueError:
            pass
    return out


def run_sim(args, binary=PHYSIM, timeout_s=RUN_TIMEOUT_S):
    """Runs physim with a validated argv tail; returns the /run JSON dict."""
    proc = subprocess.run(
        [str(binary), *args],
        capture_output=True,
        text=True,
        timeout=timeout_s,
        check=False,
    )
    # Exit 1 means the run hit the guard time: still a trace worth showing.
    if proc.returncode not in (0, 1) or not proc.stdout.startswith("time_s,"):
        raise RuntimeError(f"physim exited {proc.returncode}: {proc.stderr.strip()}")
    lines = proc.stderr.strip().splitlines()
    return {
        "argv": ["physim", *args],
        "csv": proc.stdout,
        "summary": parse_summary(lines[-1] if lines else ""),
    }


def build_physim():
    """Builds tests/physim; make skips it when nothing changed."""
    subprocess.run(["make", "-C", str(REPO / "tests"), "physim"], check=True)


class Handler(SimpleHTTPRequestHandler):
    """Serves the GUI's static files and /run."""

    binary = PHYSIM

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(HERE), **kwargs)

    def do_GET(self):
        url = urllib.parse.urlsplit(self.path)
        if url.path != "/run":
            super().do_GET()
            return
        query = dict(urllib.parse.parse_qsl(url.query, keep_blank_values=True))
        try:
            body = run_sim(build_args(query), self.binary)
        except ValueError as e:
            self._json(HTTPStatus.BAD_REQUEST, {"error": str(e)})
            return
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as e:
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(e)})
            return
        self._json(HTTPStatus.OK, body)

    def _json(self, status, obj):
        data = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def end_headers(self):
        # A rebuilt page must never be served stale mid-demo.
        if not self.path.startswith("/run"):
            self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, fmt, *args):
        if "/run" in self.path:
            super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    opts = parser.parse_args()

    build_physim()
    server = ThreadingHTTPServer(("127.0.0.1", opts.port), Handler)
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"physim GUI on {url} (Ctrl-C to stop)")
    if not opts.no_browser:
        threading.Timer(0.3, webbrowser.open, (url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    sys.exit(main())
