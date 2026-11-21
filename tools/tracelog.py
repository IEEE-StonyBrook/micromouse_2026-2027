#!/usr/bin/env python3
"""Parses the firmware's TRACE dumps (the run log) out of a serial capture.

    python3 tools/tracelog.py tools/logs/latest.log [--run N] [--csv out.csv] [--json out.json]

A dump is a header line, one `T,...` row per sample, then `TRACE end`:

    TRACE n=<n> dt_ms=<dt> cols=<comma cols> [run=.. loop_hz=.. batt_v=.. wrapped=0|1 cmd="..."]

The keys after cols= and the enc_l_mm/enc_r_mm columns are newer than the
first logs, so everything goes by key and column name and old logs still
parse. serial_term's `[<ms>] ` stamps and the Pico's `[INFO] ` tags are
stripped. Each run gets derived columns: unwrapped yaw, distance (mean
encoder travel, or integrated wheel speeds when the encoder columns are
missing), dead-reckoned x/y in mm (x along the heading at the first sample,
y to its left, from IMU yaw plus distance) and the PATH segment index (event
bits 8-13). Stdlib only.
"""

import argparse
import csv
import json
import math
import re
import sys

PREFIX = re.compile(r"^\s*(?:\[[^\]]*\]\s*)+")
HEADER_KEY = re.compile(r'(\w+)=("(?:[^"\\]|\\.)*"|\S+)')

# RunLog::Event bits (firmware/control/run_log.h).
EVENT_BITS = {
    "encoder_rotation": 1 << 0,
    "wall_left": 1 << 1,
    "wall_front": 1 << 2,
    "wall_right": 1 << 3,
    "volts_saturated": 1 << 4,
    "forward_active": 1 << 5,
    "rotation_active": 1 << 6,
    "clipped": 1 << 15,
}
SEGMENT_SHIFT = 8
SEGMENT_MASK = 0x3F


def _number(text):
    try:
        return int(text)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            return text


def _float(text):
    try:
        return float(text)
    except ValueError:
        return math.nan


def decode_events(ev):
    ev = int(ev) if not math.isnan(ev) else 0
    out = {name: bool(ev & bit) for name, bit in EVENT_BITS.items()}
    out["segment"] = (ev >> SEGMENT_SHIFT) & SEGMENT_MASK
    return out


class Run:
    """One TRACE dump: header dict, column names, rows as {column: float}."""

    def __init__(self, header, columns, rows):
        self.header = header
        self.columns = columns
        self.rows = rows
        self._derive()

    def _derive(self):
        rows = self.rows
        dt_s = self.header.get("dt_ms", 0) / 1000.0
        self.events = [decode_events(r.get("ev", 0.0)) for r in rows]
        self.segment = [e["segment"] for e in self.events]

        self.yaw_unwrapped_deg = []
        prev = None
        for r in rows:
            yaw = r.get("yaw_deg", 0.0)
            if prev is not None:
                yaw = prev + (yaw - prev + 180.0) % 360.0 - 180.0
            self.yaw_unwrapped_deg.append(yaw)
            prev = yaw

        self.distance_mm = []
        has_enc = "enc_l_mm" in self.columns and "enc_r_mm" in self.columns
        for i, r in enumerate(rows):
            if has_enc:
                d = 0.5 * (r["enc_l_mm"] + r["enc_r_mm"] - rows[0]["enc_l_mm"] - rows[0]["enc_r_mm"])
            elif i == 0:
                d = 0.0
            else:
                p = rows[i - 1]
                d = self.distance_mm[-1] + 0.5 * (p.get("left_mmps", 0.0) + p.get("right_mmps", 0.0)) * dt_s
            self.distance_mm.append(d)

        self.x_mm, self.y_mm = [], []
        x = y = 0.0
        yaw0 = self.yaw_unwrapped_deg[0] if rows else 0.0
        for i in range(len(rows)):
            if i:
                step = self.distance_mm[i] - self.distance_mm[i - 1]
                mid = math.radians(0.5 * (self.yaw_unwrapped_deg[i] + self.yaw_unwrapped_deg[i - 1]) - yaw0)
                x += step * math.cos(mid)
                y += step * math.sin(mid)
            self.x_mm.append(x)
            self.y_mm.append(y)

    DERIVED = ("yaw_unwrapped_deg", "distance_mm", "x_mm", "y_mm", "segment")

    def table(self):
        """Columns plus derived columns, one list per row."""
        names = list(self.columns) + list(self.DERIVED)
        body = [
            [r.get(c, math.nan) for c in self.columns] + [getattr(self, d)[i] for d in self.DERIVED]
            for i, r in enumerate(self.rows)
        ]
        return names, body

    def to_dict(self):
        out = {"header": self.header, "columns": self.columns, "rows": self.rows, "events": self.events}
        for d in self.DERIVED:
            out[d] = getattr(self, d)
        return out


def _parse_header(line):
    header = {}
    for key, value in HEADER_KEY.findall(line[len("TRACE"):]):
        if value.startswith('"'):
            header[key] = value[1:-1].replace('\\"', '"').replace("\\\\", "\\")
        elif key == "cols":
            header[key] = value
        else:
            header[key] = _number(value)
    return header


def parse(text):
    """Every TRACE dump in a serial_term log or a raw dump, oldest first."""
    runs = []
    header = columns = rows = None

    def close():
        if header is not None:
            runs.append(Run(header, columns, rows))

    for raw in text.splitlines():
        line = PREFIX.sub("", raw.strip("\r\n")).strip()
        if line.startswith("TRACE ") and " cols=" in line and "n=" in line:
            close()
            header = _parse_header(line)
            columns = header.pop("cols").split(",")
            rows = []
        elif header is not None and line.startswith("T,"):
            values = line[2:].split(",")
            if len(values) == len(columns):
                rows.append({c: _float(v) for c, v in zip(columns, values)})
        elif header is not None and line == "TRACE end":
            close()
            header = None
    close()
    return runs


def path_args(run):
    """The `cmd` header as physim --script arguments, or None when the run
    has no PATH command (old logs: supply the PATH string by hand)."""
    cmd = run.header.get("cmd", "")
    words = cmd.split()
    if not words or words[0] != "PATH":
        return None
    keys = dict(w.split("=", 1) for w in words[1:] if "=" in w)
    sequence = [w for w in words[1:] if "=" not in w]
    out = sequence + [keys[k] for k in ("speed", "accel", "omega", "alpha") if k in keys]
    if keys.get("smooth") in ("1", "true"):
        out.append("SMOOTH")
    if keys.get("raw") in ("1", "true"):
        out.append("RAW")
    return " ".join(out)


def summary(index, run):
    h = run.header
    line = f"run {index}: n={len(run.rows)} dt_ms={h.get('dt_ms')}"
    for key in ("run", "batt_v", "wrapped"):
        if key in h:
            line += f" {key}={h[key]}"
    if run.rows:
        line += (
            f" end x={run.x_mm[-1]:.1f} y={run.y_mm[-1]:.1f} mm yaw={run.yaw_unwrapped_deg[-1]:.1f} deg"
            f" distance={run.distance_mm[-1]:.1f} mm"
        )
    if "cmd" in h:
        line += f' cmd="{h["cmd"]}"'
    return line


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--run", type=int, default=-1, help="which dump (default: the last)")
    ap.add_argument("--csv", help="write the run's rows plus derived columns")
    ap.add_argument("--json", help="write the run as JSON")
    args = ap.parse_args(argv)

    with open(args.log, encoding="utf-8", errors="replace") as f:
        runs = parse(f.read())
    if not runs:
        print("no TRACE dump found", file=sys.stderr)
        return 1
    for i, run in enumerate(runs):
        print(summary(i, run))
    try:
        run = runs[args.run]
    except IndexError:
        print(f"no run {args.run}; {len(runs)} found", file=sys.stderr)
        return 1
    if args.csv:
        names, body = run.table()
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(names)
            w.writerows(body)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(run.to_dict(), f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
