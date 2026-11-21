#!/usr/bin/env python3
"""Serves the physics-sim GUI and runs tests/physim for it.

    python3 tools/physim_gui/server.py [--port 8765] [--no-browser]

Builds tests/physim with make (incremental, so a fresh build only costs the
first launch), listens on 127.0.0.1 only and opens the page. GET /run takes
the scenario and knobs as query parameters, checks each against KNOBS, runs
the sim with an argv list (no shell) and returns JSON: the CSV trace, the
summary line parsed into numbers, and the argv it ran.

    python3 tools/physim_gui/server.py --make-presets

regenerates presets.js from presets_spec.json: the story runs, precomputed,
which the page falls back on when the server or the sim is unavailable.
GET /logs lists the TRACE captures in tools/logs/ and tools/fixtures/, and
POST /realsim takes one ({"log": "fixtures/x.log"} or {"text": "<capture>"},
optional "run", "path", "knobs"), parses it with tools/tracelog.py and
replays it in physim twice, by command (the PATH from its header, or the
typed "path") and by volts, for the GUI's real-vs-sim page.

Standard library only.
"""

import argparse
import json
import math
import re
import subprocess
import sys
import tempfile
import threading
import urllib.parse
import webbrowser
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
PHYSIM = REPO / "tests" / "physim"
PRESET_SPEC = HERE / "presets_spec.json"
PRESETS = HERE / "presets.js"
LOG_DIRS = {"logs": REPO / "tools" / "logs", "fixtures": REPO / "tools" / "fixtures"}
FIXTURES = LOG_DIRS["fixtures"]

sys.path.insert(0, str(REPO / "tools"))
import tracelog  # noqa: E402

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

# The plant knobs a real-vs-sim replay may set: the profile comes from the
# logged command, and a trace has one row per run log sample.
PLANT_KNOBS = tuple(k for k in KNOBS if k not in ("speed", "accel", "every") and not k.startswith("seam"))

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
    knobs = {k: v for k, v in query.items() if k != "scenario"}
    return args + knob_args(knobs, KNOBS)


def plant_args(knobs):
    """physim flags for a real-vs-sim replay's knobs: PLANT_KNOBS only."""
    return knob_args(knobs, PLANT_KNOBS)


def knob_args(knobs, allowed):
    """Checks each {name: value} against KNOBS and turns it into flags."""
    args = []
    for name, text in knobs.items():
        if name not in allowed:
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


# ---------- real vs sim ----------

LOG_NAME = re.compile(r"^(logs|fixtures)/([A-Za-z0-9][A-Za-z0-9._-]*\.log)$")
MAX_LOG_BYTES = 8 << 20
PATH_WORD = re.compile(r"^(?:[FLRB0-9#]+|\d{1,5}(?:\.\d{1,3})?|SMOOTH|RAW)$", re.IGNORECASE)


def list_logs():
    """The captures the page may open by name: *.log in tools/logs and tools/fixtures."""
    out = []
    for folder, path in LOG_DIRS.items():
        if path.is_dir():
            for f in sorted(path.glob("*.log"), key=lambda f: -f.stat().st_mtime):
                if LOG_NAME.match(f"{folder}/{f.name}") and f.is_file():
                    out.append({"name": f"{folder}/{f.name}", "bytes": f.stat().st_size, "mtime": f.stat().st_mtime})
    return out


def resolve_log(name):
    """`logs/<file>.log` or `fixtures/<file>.log` as a path inside that folder.

    Raises ValueError for any other name, one that leaves the folder (`..`, a
    link) or a file that does not exist.
    """
    m = LOG_NAME.match(name or "")
    if not m or ".." in name:
        raise ValueError("log must be logs/<name>.log or fixtures/<name>.log")
    folder = LOG_DIRS[m.group(1)].resolve()
    path = (folder / m.group(2)).resolve()
    if path.parent != folder or not path.is_file():
        raise ValueError(f"no such log: {name}")
    return LOG_DIRS[m.group(1)] / m.group(2)


def check_path(text):
    """A PATH argument string (sequence, numbers, SMOOTH/RAW), word by word."""
    words = (text or "").split()
    if words[:1] and words[0].upper() == "PATH":
        words = words[1:]
    if not words or len(words) > 16 or len(" ".join(words)) > 160:
        raise ValueError("path must be a PATH sequence, e.g. FFRFFLF 600 3000 SMOOTH")
    for w in words:
        if not PATH_WORD.match(w):
            raise ValueError(f"path: bad word {w!r}")
    return " ".join(words)


def _clean(v):
    return None if isinstance(v, float) and not math.isfinite(v) else v


def run_json(run):
    """A tracelog Run as columns ({name: [values]}), NaN as null."""
    cols = {c: [_clean(r.get(c, math.nan)) for r in run.rows] for c in run.columns}
    for d in run.DERIVED:
        cols[d] = [_clean(v) for v in getattr(run, d)]
    header = {k: _clean(v) for k, v in run.header.items()}
    header["n"] = len(run.rows)
    return {"header": header, "cols": cols}


def parse_log(text, index):
    """Every run in a capture, summarised, plus run `index` (-1: the last) in full."""
    runs = tracelog.parse(text)
    if not runs:
        raise ValueError("no TRACE dump in that log")
    try:
        picked = runs[index]
    except IndexError:
        raise ValueError(f"no run {index}; the log has {len(runs)}") from None
    summaries = [
        {"index": i, "n": len(r.rows), "run": r.header.get("run"), "cmd": r.header.get("cmd"),
         "path": tracelog.path_args(r)}
        for i, r in enumerate(runs)
    ]
    return {"runs": summaries, "index": runs.index(picked), "real": run_json(picked), "picked": picked}


def dump_text(run):
    """A Run back as a raw TRACE dump, for physim --replay-volts."""
    keys = " ".join(f"{k}={v}" for k, v in run.header.items() if k in ("run", "loop_hz", "batt_v", "wrapped"))
    lines = [f"TRACE n={len(run.rows)} dt_ms={run.header.get('dt_ms', 0)} cols={','.join(run.columns)} {keys}"]
    lines += ["T," + ",".join(repr(r.get(c, math.nan)) for c in run.columns) for r in run.rows]
    lines.append("TRACE end")
    return "\n".join(lines) + "\n"


def physim_trace(args, binary, timeout_s=RUN_TIMEOUT_S):
    """Runs physim for a TRACE; returns (Run, stderr's last line)."""
    proc = subprocess.run([str(binary), *args], capture_output=True, text=True, timeout=timeout_s, check=False)
    runs = tracelog.parse(proc.stdout) if proc.returncode in (0, 1) else []
    if not runs:
        raise RuntimeError(f"physim exited {proc.returncode}: {proc.stderr.strip()[-300:]}")
    lines = proc.stderr.strip().splitlines()
    return runs[-1], lines[-1] if lines else ""


def realsim(body, binary=PHYSIM):
    """The /realsim reply: the logged run and its command and volts replays."""
    if not isinstance(body, dict):
        raise ValueError("body must be a JSON object")
    if body.get("log"):
        text = resolve_log(str(body["log"])).read_text(encoding="utf-8", errors="replace")
    elif isinstance(body.get("text"), str):
        text = body["text"]
    else:
        raise ValueError("give a log name or the log text")
    try:
        index = int(body.get("run", -1))
    except (TypeError, ValueError):
        raise ValueError("run must be a number") from None
    knobs = body.get("knobs") or {}
    if not isinstance(knobs, dict):
        raise ValueError("knobs must be an object")
    knobs = {str(k): str(v) for k, v in knobs.items() if v not in ("", None)}
    plant = plant_args(knobs)
    parsed = parse_log(text, index)
    real = parsed.pop("picked")
    path = body.get("path") or tracelog.path_args(real)
    path = check_path(path) if path else None
    batt = real.header.get("batt_v")
    if "battery" not in knobs and isinstance(batt, (int, float)) and math.isfinite(batt) and 3.0 <= batt <= 12.6:
        plant += ["--battery", str(float(batt))]

    out = dict(parsed, path=path, argv={}, errors={})
    if path:
        args = ["--script", path, "--format", "trace", *plant]
        out["argv"]["cmd"] = ["physim", *args]
        try:
            out["cmd"] = run_json(physim_trace(args, binary)[0])
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as e:
            out["errors"]["cmd"] = str(e)
    else:
        out["errors"]["cmd"] = "the log names no PATH command: type the PATH string"
    with tempfile.TemporaryDirectory() as tmp:
        dump = Path(tmp, "real.log")
        dump.write_text(dump_text(real))
        args = ["--replay-volts", str(dump), *plant]
        out["argv"]["volts"] = ["physim", "--replay-volts", "<run>", *plant]
        try:
            volts, summary = physim_trace(args, binary)
            out["volts"] = run_json(volts)
            s = parse_summary(summary)
            out["volts_rms"] = {k: s[k] for k in ("rms_left_mmps", "rms_right_mmps") if k in s}
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as e:
            out["errors"]["volts"] = str(e)
    return out


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
        if url.path == "/logs":
            self._json(HTTPStatus.OK, {"logs": list_logs()})
            return
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

    def do_POST(self):
        if urllib.parse.urlsplit(self.path).path != "/realsim":
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError:
            self._json(HTTPStatus.LENGTH_REQUIRED, {"error": "Content-Length required"})
            return
        if not 0 < length <= MAX_LOG_BYTES:
            self._json(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, {"error": f"body must be 1..{MAX_LOG_BYTES} bytes"})
            return
        try:
            body = realsim(json.loads(self.rfile.read(length)), self.binary)
        except (ValueError, UnicodeDecodeError) as e:
            self._json(HTTPStatus.BAD_REQUEST, {"error": str(e)})
            return
        self._json(HTTPStatus.OK, body)

    def _json(self, status, obj):
        data = json.dumps(obj, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def end_headers(self):
        # A rebuilt page must never be served stale mid-demo.
        if not self.path.startswith(("/run", "/realsim", "/logs")):
            self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, fmt, *args):
        if self.path.startswith(("/run", "/realsim")):
            super().log_message(fmt, *args)


def make_presets():
    """Runs every story in presets_spec.json and writes presets.js.

    presets.js is the spec with each run's result added, as a script that sets
    window.PHYSIM_PRESETS: a script, not JSON, so the stories still play when
    index.html is opened straight from disk with no server at all.
    """
    spec = json.loads(PRESET_SPEC.read_text())
    for preset in spec["presets"]:
        for run in preset["runs"]:
            query = {k: str(v) for k, v in run["knobs"].items()}
            query["scenario"] = run["scenario"]
            query["every"] = str(spec["every"])
            run["result"] = run_sim(build_args(query))
    body = json.dumps(spec, separators=(",", ":"))
    PRESETS.write_text(
        f"// Generated by server.py --make-presets; do not edit.\nwindow.PHYSIM_PRESETS = {body};\n"
    )
    print(f"wrote {PRESETS} ({PRESETS.stat().st_size // 1024} KiB)")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--make-presets", action="store_true")
    opts = parser.parse_args()

    build_physim()
    if opts.make_presets:
        make_presets()
        return
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
