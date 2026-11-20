// Physics-sim viewer: runs tests/physim through server.py's /run (or falls back
// on the precomputed stories in presets.js) and plays the trace back on a
// top-down floor view and four time-synced uPlot charts.
//
// Frames: the sim reports path length (distance_mm) and heading (yaw_deg,
// + = CCW), not x/y, so the floor view integrates them. The mouse starts at
// the centre of a cell facing +y (north on screen); heading 0 points up.
"use strict";

// From firmware/config: geometry.h and config.h. Drawing only.
const CELL_MM = 180;
const POST_MM = 12;
const WHEEL_BASE_MM = 80.4;
const WHEEL_DIAMETER_MM = 37;
const ROBOT_LENGTH_MM = 93.2;
const ROBOT_WIDTH_MM = 74; // body between the tyres; not in config, drawing only
const BODY_AHEAD_OF_AXLE_MM = ROBOT_LENGTH_MM / 2 - WHEEL_DIAMETER_MM / 2;
const DEFAULT_BATTERY_V = 8.35;
const RUN_TIMEOUT_MS = 8000;

// Knobs shown in the panel. Blank = the sim's nominal value (the placeholder).
const KNOBS = [
  { key: "friction", label: "Friction", unit: "V", min: 0, max: 3, step: 0.05, hint: "0.69 / 0.57 (kS)" },
  { key: "mu", label: "Grip μ", unit: "", min: 0.02, max: 2, step: 0.05, hint: "1.0" },
  { key: "slip-rate", label: "Slip events", unit: "/s", min: 0, max: 50, step: 1, hint: "0" },
  { key: "counts-per-mm", label: "Encoder", unit: "cnt/mm", min: 0.2, max: 200, step: 0.5, hint: "3.08" },
  { key: "gyro-noise", label: "Gyro noise", unit: "°/s√Hz", min: 0, max: 20, step: 0.1, hint: "0" },
  { key: "gyro-bias", label: "Gyro bias", unit: "°/s", min: -50, max: 50, step: 0.5, hint: "0" },
  { key: "battery", label: "Battery", unit: "V", min: 3, max: 12.6, step: 0.1, hint: "8.35" },
];
// Seam knobs the panel does not show but always sends with the seam on, so
// the advanced pitch and z-accel charts have a bump to show.
const SEAM_EXTRAS = { "seam-pitch": 300, "seam-accel": 20 };

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

const state = {
  a: null, // pinned run (blue), or null
  cur: null, // latest run: blue alone, orange dashed when A is pinned
  t: 0,
  playing: false,
  speed: 1,
  lastFrame: 0,
  charts: [],
  joined: null, // [x, ...] the charts' shared time axis
  advanced: false,
  follow: false,
  preset: null,
  req: 0, // bumped by every Run, story, Pin and Unpin; a stale reply is dropped
};

// ---------- data ----------

function parseCsv(text) {
  const lines = text.trim().split("\n");
  const names = lines[0].split(",");
  const cols = {};
  names.forEach((n) => (cols[n] = new Float64Array(lines.length - 1)));
  for (let i = 1; i < lines.length; i++) {
    const f = lines[i].split(",");
    for (let j = 0; j < names.length; j++) cols[names[j]][i - 1] = +f[j];
  }
  return cols;
}

// Builds a run from a /run result: columns plus the derived pose and errors.
function makeRun(result, scenario, knobs) {
  const c = parseCsv(result.csv);
  const n = c.time_s.length;
  // The profile restarts at 0 mm on each move (ss90 has three); unwrap it so
  // the target is path length like distance_mm.
  const target = new Float64Array(n);
  let offset = 0;
  for (let i = 0; i < n; i++) {
    if (i > 0 && c.target_mm[i] < c.target_mm[i - 1] - 1) offset += c.target_mm[i - 1] - c.target_mm[i];
    target[i] = c.target_mm[i] + offset;
  }
  const pose = (dist, yaw) => {
    const x = new Float64Array(n);
    const y = new Float64Array(n);
    let px = CELL_MM / 2;
    let py = CELL_MM / 2;
    for (let i = 0; i < n; i++) {
      if (i > 0) {
        const dd = dist[i] - dist[i - 1];
        const h = (((yaw[i] + yaw[i - 1]) / 2) * Math.PI) / 180;
        px += -Math.sin(h) * dd;
        py += Math.cos(h) * dd;
      }
      x[i] = px;
      y[i] = py;
    }
    return { x, y };
  };
  const act = pose(c.distance_mm, c.yaw_deg);
  const tgt = pose(target, c.target_deg);
  const err = new Float64Array(n);
  const slipMag = new Float64Array(n);
  let peakSlip = 0;
  let slipTicks = 0;
  for (let i = 0; i < n; i++) {
    err[i] = c.distance_mm[i] - target[i];
    const s = Math.max(
      Math.abs(c.wheel_left_mmps[i] - c.ground_left_mmps[i]),
      Math.abs(c.wheel_right_mmps[i] - c.ground_right_mmps[i])
    );
    slipMag[i] = s;
    peakSlip = Math.max(peakSlip, s);
    if (c.slip_left[i] || c.slip_right[i]) slipTicks++;
  }
  return {
    scenario,
    knobs,
    c,
    n,
    target,
    err,
    act,
    tgt,
    peakSlip,
    slipFrac: slipTicks / n,
    summary: result.summary || {},
    battery: knobs.battery !== undefined ? +knobs.battery : DEFAULT_BATTERY_V,
    seamAt: knobs["seam-at"] !== undefined ? +knobs["seam-at"] : null,
    duration: c.time_s[n - 1],
    source: result.source || "live",
  };
}

async function fetchRun(scenario, knobs) {
  const q = new URLSearchParams({ scenario });
  for (const [k, v] of Object.entries(knobs)) q.set(k, String(v));
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), RUN_TIMEOUT_MS);
  try {
    const r = await fetch("run?" + q, { signal: ctl.signal, cache: "no-store" });
    const body = await r.json().catch(() => ({ error: `HTTP ${r.status}` }));
    if (!r.ok) throw new Error(body.error || `HTTP ${r.status}`);
    return makeRun(body, scenario, knobs);
  } finally {
    clearTimeout(timer);
  }
}

// ---------- knob panel ----------

function buildKnobPanel() {
  const list = $("knobList");
  for (const k of KNOBS) {
    const row = document.createElement("label");
    row.className = "row";
    row.title = `${k.label}${k.unit ? " (" + k.unit + ")" : ""}; blank = nominal ${k.hint}`;
    row.innerHTML = `<span>${k.label}${k.unit ? ' <span class="unit">' + k.unit + "</span>" : ""}</span>`;
    const input = document.createElement("input");
    Object.assign(input, { type: "number", min: k.min, max: k.max, step: k.step, placeholder: k.hint });
    input.id = "k-" + k.key;
    input.addEventListener("input", () => row.classList.toggle("changed", input.value !== ""));
    row.appendChild(input);
    list.appendChild(row);
  }
}

function readKnobs() {
  const knobs = {};
  for (const k of KNOBS) {
    const v = $("k-" + k.key).value;
    if (v === "") continue;
    const num = +v;
    if (!Number.isFinite(num) || num < k.min || num > k.max)
      throw new Error(`${k.label} must be ${k.min} to ${k.max}`);
    knobs[k.key] = num;
  }
  if ($("seamOn").checked) {
    knobs["seam-at"] = +$("seamAt").value;
    knobs["seam-encoder"] = +$("seamEncoder").value;
    knobs["seam-traction"] = +$("seamTraction").value;
    Object.assign(knobs, SEAM_EXTRAS);
  }
  return knobs;
}

function writeKnobs(scenario, knobs) {
  $("scenario").value = scenario;
  for (const k of KNOBS) {
    const input = $("k-" + k.key);
    input.value = knobs[k.key] !== undefined ? knobs[k.key] : "";
    input.parentElement.classList.toggle("changed", input.value !== "");
  }
  const seam = knobs["seam-at"] !== undefined;
  $("seamOn").checked = seam;
  if (seam) {
    $("seamAt").value = knobs["seam-at"];
    $("seamEncoder").value = knobs["seam-encoder"] ?? 0;
    $("seamTraction").value = knobs["seam-traction"] ?? 0.3;
  }
}

function setStatus(text, error = false) {
  $("status").textContent = text;
  $("status").classList.toggle("error", error);
}

function setSource() {
  const runs = [state.a, state.cur].filter(Boolean);
  const pre = runs.some((r) => r.source !== "live");
  $("source").textContent = pre ? "precomputed runs (server unavailable)" : "live sim";
}

async function runFromPanel() {
  let knobs;
  try {
    knobs = readKnobs();
  } catch (e) {
    setStatus(e.message, true);
    return;
  }
  const scenario = $("scenario").value;
  setStatus("Running…");
  $("run").disabled = true;
  try {
    const req = ++state.req;
    const run = await fetchRun(scenario, knobs);
    if (req !== state.req) return; // a later click won
    state.preset = null;
    markPreset();
    show(state.a, run, true);
    const set = Object.entries(knobs).map(([k, v]) => `${k} ${v}`);
    $("caption").textContent = `Your run: ${scenario}${set.length ? ", " + set.join(", ") : ", nominal knobs"}${state.a ? " (B, against the pinned A)" : ""}.`;
    setStatus(`Ran ${scenario} in the sim${state.a ? " as B" : ""}.`);
  } catch (e) {
    setStatus(`Run failed: ${e.message}`, true);
  } finally {
    $("run").disabled = false;
  }
}

// ---------- presets ----------

function buildPresets() {
  const spec = window.PHYSIM_PRESETS;
  const nav = $("presets");
  if (!spec) {
    nav.textContent = "(presets.js missing)";
    return;
  }
  for (const p of spec.presets) {
    const b = document.createElement("button");
    b.type = "button";
    b.textContent = p.label;
    b.title = p.caption;
    b.dataset.id = p.id;
    b.addEventListener("click", () => loadPreset(p));
    nav.appendChild(b);
  }
}

function markPreset() {
  for (const b of $("presets").children) b.classList.toggle("active", b.dataset.id === state.preset);
}

// Runs a story live; any failure falls back on its precomputed result.
async function loadPreset(p) {
  state.preset = p.id;
  markPreset();
  $("caption").textContent = p.caption;
  setStatus("Running…");
  const req = ++state.req;
  const runs = await Promise.all(
    p.runs.map(async (spec) => {
      try {
        return await fetchRun(spec.scenario, spec.knobs);
      } catch (e) {
        return makeRun({ ...spec.result, source: "preset" }, spec.scenario, spec.knobs);
      }
    })
  );
  if (req !== state.req) return; // a later click won
  const last = p.runs[p.runs.length - 1];
  writeKnobs(last.scenario, last.knobs);
  if (runs.length > 1) show(runs[0], runs[1], true);
  else show(null, runs[0], true);
  setStatus(runs.length > 1 ? "A = first run (blue), B = second (orange, dashed)." : "");
  play();
}

// ---------- charts ----------

const DASH_TARGET = [6, 4];

// A y-range at least `span` wide, so a near-flat trace (0.02 deg of noise)
// does not fill the chart and look like a fault.
const minSpan = (span) => (u, lo, hi) => {
  if (lo == null || hi == null) return [-span / 2, span / 2];
  const pad = (hi - lo) * 0.08;
  lo -= pad;
  hi += pad;
  if (hi - lo < span) {
    const mid = (hi + lo) / 2;
    return [mid - span / 2, mid + span / 2];
  }
  return [lo, hi];
};
const DASH_B = [5, 4];

// Each chart: series as [label, colour var, run ("a"|"b"), getter, opts].
function chartSpecs() {
  const specs = [
    {
      title: "Distance (mm): target vs actual; error on the right axis",
      scales: { mm: { range: minSpan(20) }, err: { range: minSpan(4) } },
      axes: [{ scale: "mm", label: "" }, { scale: "err", side: 1, grid: false }],
      series: [
        ["target", "--target", "a", (r) => r.target, { scale: "mm", dash: DASH_TARGET }],
        ["actual", "--actual", "a", (r) => r.c.distance_mm, { scale: "mm" }],
        ["error", "--slip", "a", (r) => r.err, { scale: "err", width: 1.5 }],
        ["B actual", "--runb", "b", (r) => r.c.distance_mm, { scale: "mm", dash: DASH_B }],
        ["B error", "--runb", "b", (r) => r.err, { scale: "err", dash: [2, 3], width: 1.5 }],
      ],
    },
    {
      title: "Heading: target vs truth vs IMU (°)",
      scales: { deg: { range: minSpan(4) } },
      axes: [{ scale: "deg" }],
      series: [
        ["target", "--target", "a", (r) => r.c.target_deg, { scale: "deg", dash: DASH_TARGET }],
        ["truth", "--actual", "a", (r) => r.c.yaw_deg, { scale: "deg" }],
        ["IMU", "--imu", "a", (r) => r.c.imu_yaw_deg, { scale: "deg", width: 1.5 }],
        ["B truth", "--runb", "b", (r) => r.c.yaw_deg, { scale: "deg", dash: DASH_B }],
        ["B IMU", "--imu", "b", (r) => r.c.imu_yaw_deg, { scale: "deg", dash: [2, 3], width: 1.5 }],
      ],
    },
    {
      title: "Wheel speed (mm/s): tyre vs ground; hatched = slipping",
      scales: { v: { range: minSpan(100) } },
      axes: [{ scale: "v" }],
      slipBands: true,
      series: [
        ["L tyre", "--actual", "a", (r) => r.c.wheel_left_mmps, { scale: "v", width: 1.5 }],
        ["L gnd", "--actual", "a", (r) => r.c.ground_left_mmps, { scale: "v", dash: [1, 3], width: 2 }],
        ["R tyre", "--right", "a", (r) => r.c.wheel_right_mmps, { scale: "v", width: 1.5 }],
        ["R gnd", "--right", "a", (r) => r.c.ground_right_mmps, { scale: "v", dash: [1, 3], width: 2 }],
        ["B L gnd", "--runb", "b", (r) => r.c.ground_left_mmps, { scale: "v", dash: DASH_B, width: 1.5 }],
        ["B R gnd", "--runb", "b", (r) => r.c.ground_right_mmps, { scale: "v", dash: [2, 3], width: 1.5 }],
      ],
    },
    {
      title: "Motor volts (V); dotted grey = battery ceiling",
      scales: { V: {} },
      axes: [{ scale: "V" }],
      series: [
        ["L", "--actual", "a", (r) => r.c.volts_left, { scale: "V", width: 1.5 }],
        ["R", "--right", "a", (r) => r.c.volts_right, { scale: "V", width: 1.5 }],
        ["battery", "--target", "a", (r) => r.c.time_s.map(() => r.battery), { scale: "V", dash: [2, 4], width: 1 }],
        ["−battery", "--target", "a", (r) => r.c.time_s.map(() => -r.battery), { scale: "V", dash: [2, 4], width: 1 }],
        ["B L", "--runb", "b", (r) => r.c.volts_left, { scale: "V", dash: DASH_B, width: 1.5 }],
        ["B R", "--runb", "b", (r) => r.c.volts_right, { scale: "V", dash: [2, 3], width: 1.5 }],
        ["B battery", "--runb", "b", (r) => r.c.time_s.map(() => r.battery), { scale: "V", dash: [1, 5], width: 1, onlyIf: (a, b) => a && b && a.battery !== b.battery }],
      ],
    },
  ];
  if (state.advanced) {
    specs.push(
      {
        title: "Encoders (mm of wheel travel)",
        scales: { mm: {} },
        axes: [{ scale: "mm" }],
        series: [
          ["enc L", "--actual", "a", (r) => r.c.enc_left_mm, { scale: "mm", width: 1.5 }],
          ["enc R", "--right", "a", (r) => r.c.enc_right_mm, { scale: "mm", width: 1.5 }],
          ["truth", "--target", "a", (r) => r.c.distance_mm, { scale: "mm", dash: DASH_TARGET, width: 1 }],
          ["B enc L", "--runb", "b", (r) => r.c.enc_left_mm, { scale: "mm", dash: DASH_B, width: 1.5 }],
          ["B enc R", "--runb", "b", (r) => r.c.enc_right_mm, { scale: "mm", dash: [2, 3], width: 1.5 }],
        ],
      },
      {
        title: "Seam bump: pitch rate (°/s) and z-accel (m/s², right axis)",
        scales: { p: { range: minSpan(20) }, z: { range: minSpan(2) } },
        axes: [{ scale: "p" }, { scale: "z", side: 1, grid: false }],
        series: [
          ["pitch rate", "--actual", "a", (r) => r.c.pitch_rate_degps, { scale: "p", width: 1.5 }],
          ["z-accel", "--imu", "a", (r) => r.c.accel_z_mps2, { scale: "z", width: 1.5 }],
          ["B pitch rate", "--runb", "b", (r) => r.c.pitch_rate_degps, { scale: "p", dash: DASH_B, width: 1.5 }],
          ["B z-accel", "--runb", "b", (r) => r.c.accel_z_mps2, { scale: "z", dash: [2, 3], width: 1.5 }],
        ],
      }
    );
  }
  return specs;
}

// Runs of ticks where a wheel slips, as [t0, t1] pairs.
function slipIntervals(run) {
  const out = [];
  let start = null;
  const c = run.c;
  for (let i = 0; i < run.n; i++) {
    const s = c.slip_left[i] || c.slip_right[i];
    if (s && start === null) start = c.time_s[i];
    if (!s && start !== null) {
      out.push([start, c.time_s[i]]);
      start = null;
    }
  }
  if (start !== null) out.push([start, c.time_s[run.n - 1]]);
  return out;
}

function hatch(ctx, colour) {
  const p = document.createElement("canvas");
  p.width = p.height = 8;
  const g = p.getContext("2d");
  g.strokeStyle = colour;
  g.lineWidth = 1.5;
  g.beginPath();
  g.moveTo(0, 8);
  g.lineTo(8, 0);
  g.stroke();
  return ctx.createPattern(p, "repeat");
}

// The A run (blue) is the pinned one when pinned, else the latest; B is the
// latest run while one is pinned.
function runsAB() {
  return state.a ? [state.a, state.cur] : [state.cur, null];
}

function buildCharts() {
  const host = $("charts");
  for (const ch of state.charts) ch.u.destroy();
  host.innerHTML = "";
  state.charts = [];
  const [a, b] = runsAB();
  if (!a) return;

  const tables = [];
  const seriesMeta = [];
  const specs = chartSpecs();
  for (const spec of specs) {
    const used = spec.series.filter(([, , who, , o]) => (who === "a" || b) && (!o.onlyIf || o.onlyIf(a, b)));
    seriesMeta.push(used);
  }
  // One shared x axis for A and B (their ticks can differ in count).
  const xa = Array.from(a.c.time_s);
  const xb = b ? Array.from(b.c.time_s) : null;
  const xs = Array.from(new Set(xb ? xa.concat(xb) : xa)).sort((p, q) => p - q);
  state.joined = xs;
  const align = (run, arr) => {
    // Values at the joined times; null after this run ends.
    const out = new Array(xs.length).fill(null);
    let j = 0;
    const t = run.c.time_s;
    for (let i = 0; i < xs.length; i++) {
      while (j < run.n - 1 && t[j + 1] <= xs[i] + 1e-9) j++;
      if (xs[i] <= t[run.n - 1] + 1e-9) out[i] = arr[j];
    }
    return out;
  };

  const grid = css("--cell");
  const text = css("--muted");
  specs.forEach((spec, ci) => {
    const used = seriesMeta[ci];
    const data = [xs, ...used.map(([, , who, get]) => align(who === "a" ? a : b, get(who === "a" ? a : b)))];
    const box = document.createElement("div");
    box.className = "chart";
    box.innerHTML = `<div class="head"><h2>${spec.title}</h2></div><div class="plot"></div>`;
    host.appendChild(box);
    const plotEl = box.querySelector(".plot");
    const bandsA = spec.slipBands ? slipIntervals(a) : [];
    const bandsB = spec.slipBands && b ? slipIntervals(b) : [];
    const opts = {
      width: 400,
      height: 100,
      cursor: { sync: { key: "physim", setSeries: false }, drag: { x: false, y: false }, points: { size: 6 } },
      legend: { live: false },
      scales: { x: { time: false }, ...spec.scales },
      axes: [
        {
          stroke: text,
          grid: { stroke: grid },
          ticks: { stroke: grid },
          // Time labels on the bottom chart only; the others share its axis.
          size: ci === specs.length - 1 ? 26 : 6,
          values: ci === specs.length - 1 ? undefined : (u, splits) => splits.map(() => ""),
          font: "12px system-ui",
        },
        ...spec.axes.map((ax) => ({
          scale: ax.scale,
          side: ax.side ?? 3,
          stroke: text,
          grid: ax.grid === false ? { show: false } : { stroke: grid },
          ticks: { stroke: grid },
          size: 52,
          font: "12px system-ui",
        })),
      ],
      series: [
        { label: "t (s)", value: (u, v) => (v == null ? "" : v.toFixed(3)) },
        ...used.map(([label, colour, , , o]) => ({
          label,
          stroke: css(colour),
          width: o.width ?? 2,
          dash: o.dash,
          scale: o.scale,
          spanGaps: false,
          points: { show: false },
          show: true,
          value: (u, v) => (v == null ? "–" : v.toFixed(1)),
        })),
      ],
      hooks: {
        drawClear: [
          (u) => {
            if (!bandsA.length && !bandsB.length) return;
            const ctx = u.ctx;
            const { top, height } = u.bbox;
            ctx.save();
            const draw = (bands, colour, y0, h) => {
              ctx.fillStyle = hatch(ctx, colour);
              for (const [t0, t1] of bands) {
                const x0 = u.valToPos(t0, "x", true);
                const x1 = Math.max(u.valToPos(t1, "x", true), x0 + 2);
                ctx.fillRect(x0, y0, x1 - x0, h);
              }
            };
            draw(bandsA, css("--slip"), top, b ? height * 0.6 : height);
            draw(bandsB, css("--runb"), top + height * 0.6, height * 0.4);
            ctx.restore();
          },
        ],
      },
    };
    const u = new uPlot(opts, data, plotEl);
    // Labels only, beside the title: the plot keeps the height.
    box.querySelector(".head").appendChild(u.root.querySelector(".u-legend"));
    const head = document.createElement("div");
    head.className = "playhead";
    u.over.appendChild(head);
    // Click a chart to move the playhead there.
    u.over.addEventListener("click", () => {
      const t = u.posToVal(u.cursor.left, "x");
      if (Number.isFinite(t)) seek(t);
    });
    state.charts.push({ u, head, box });
  });
  sizeCharts();
}

function sizeCharts() {
  const host = $("charts");
  const n = state.charts.length;
  if (!n) return;
  const gap = 4;
  const fit = Math.floor((host.clientHeight - gap * (n - 1)) / n);
  const each = Math.max(fit, 110);
  for (const ch of state.charts) {
    ch.box.style.flex = "none";
    ch.box.style.height = each + "px";
    const title = ch.box.querySelector(".head").offsetHeight;
    const w = ch.box.clientWidth - 8;
    ch.u.setSize({ width: Math.max(w, 200), height: Math.max(each - title - 6, 60) });
  }
  updatePlayhead();
}

function updatePlayhead() {
  const xs = state.joined;
  if (!xs || !xs.length) return;
  for (const ch of state.charts) {
    const x = ch.u.valToPos(state.t, "x");
    ch.head.style.left = x + "px";
  }
}

function lowerBound(arr, v) {
  let lo = 0;
  let hi = arr.length;
  while (lo < hi) {
    const m = (lo + hi) >> 1;
    if (arr[m] < v) lo = m + 1;
    else hi = m;
  }
  return lo;
}

// ---------- floor view ----------

// Linear interpolation of a run's columns at time t.
function sampleAt(run, t) {
  const time = run.c.time_s;
  const tc = Math.min(Math.max(t, time[0]), time[run.n - 1]);
  let i = Math.max(lowerBound(time, tc) - 1, 0);
  const j = Math.min(i + 1, run.n - 1);
  const f = j > i ? (tc - time[i]) / (time[j] - time[i]) : 0;
  const L = (arr) => arr[i] + (arr[j] - arr[i]) * Math.min(Math.max(f, 0), 1);
  const k = f < 0.5 ? i : j;
  return {
    i: k,
    x: L(run.act.x),
    y: L(run.act.y),
    yaw: L(run.c.yaw_deg),
    tx: L(run.tgt.x),
    ty: L(run.tgt.y),
    tyaw: L(run.c.target_deg),
    slipL: !!run.c.slip_left[k],
    slipR: !!run.c.slip_right[k],
  };
}

function viewBounds(runs) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const r of runs) {
    for (const arr of [r.act, r.tgt]) {
      for (let i = 0; i < r.n; i++) {
        x0 = Math.min(x0, arr.x[i]);
        x1 = Math.max(x1, arr.x[i]);
        y0 = Math.min(y0, arr.y[i]);
        y1 = Math.max(y1, arr.y[i]);
      }
    }
  }
  const pad = 75; // the mouse's reach around its axle centre
  // At least 2.5 cells each way, so a spin in place still shows its cell.
  const grow = (lo, hi) => {
    const need = Math.max(0, 2.5 * CELL_MM - (hi - lo - 2 * pad)) / 2;
    return [lo - pad - need, hi + pad + need];
  };
  [x0, x1] = grow(x0, x1);
  [y0, y1] = grow(y0, y1);
  return { x0, y0, x1, y1 };
}

function drawFloor() {
  const canvas = $("floor");
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth;
  const H = canvas.clientHeight;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = css("--floor");
  ctx.fillRect(0, 0, W, H);
  const [a, b] = runsAB();
  if (!a) return;
  const runs = b ? [a, b] : [a];
  const sa = sampleAt(a, state.t);
  const sb = b ? sampleAt(b, state.t) : null;

  // World mm -> screen px, y up.
  let bb;
  if (state.follow) {
    const span = 2.2 * CELL_MM;
    const cx = sb ? (sa.x + sb.x) / 2 : sa.x;
    const cy = sb ? (sa.y + sb.y) / 2 : sa.y;
    bb = { x0: cx - span / 2, x1: cx + span / 2, y0: cy - span / 2, y1: cy + span / 2 };
  } else {
    bb = viewBounds(runs);
  }
  const margin = 18;
  const s = Math.min((W - 2 * margin) / (bb.x1 - bb.x0), (H - 2 * margin - 18) / (bb.y1 - bb.y0));
  const cx = (bb.x0 + bb.x1) / 2;
  const cy = (bb.y0 + bb.y1) / 2;
  const ox = W / 2 - cx * s;
  const oy = (H - 18) / 2 + cy * s;
  ctx.save();
  ctx.setTransform(dpr * s, 0, 0, -dpr * s, dpr * ox, dpr * oy);
  const px = 1 / s; // one screen pixel in mm

  // Visible world rectangle, for the grid.
  const vx0 = (0 - ox) / s, vx1 = (W - ox) / s;
  const vy0 = (oy - H) / s, vy1 = oy / s;
  ctx.strokeStyle = css("--cell");
  ctx.lineWidth = px;
  ctx.beginPath();
  for (let gx = Math.floor(vx0 / CELL_MM) * CELL_MM; gx <= vx1; gx += CELL_MM) {
    ctx.moveTo(gx, vy0);
    ctx.lineTo(gx, vy1);
  }
  for (let gy = Math.floor(vy0 / CELL_MM) * CELL_MM; gy <= vy1; gy += CELL_MM) {
    ctx.moveTo(vx0, gy);
    ctx.lineTo(vx1, gy);
  }
  ctx.stroke();
  ctx.fillStyle = css("--post");
  for (let gx = Math.floor(vx0 / CELL_MM) * CELL_MM; gx <= vx1; gx += CELL_MM)
    for (let gy = Math.floor(vy0 / CELL_MM) * CELL_MM; gy <= vy1; gy += CELL_MM)
      ctx.fillRect(gx - POST_MM / 2, gy - POST_MM / 2, POST_MM, POST_MM);

  // Seam: a hatched strip across the path at its distance.
  for (const r of runs) if (r.seamAt !== null) drawSeam(ctx, r, px);

  // Target path (faint) and actual trails up to now.
  const trail = (xs, ys, upto, colour, width, dash, alpha = 1) => {
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = colour;
    ctx.lineWidth = width * px;
    ctx.setLineDash(dash.map((d) => d * px));
    ctx.beginPath();
    for (let i = 0; i <= upto; i++) (i ? ctx.lineTo : ctx.moveTo).call(ctx, xs[i], ys[i]);
    ctx.stroke();
    ctx.restore();
  };
  trail(a.tgt.x, a.tgt.y, a.n - 1, css("--target"), 1.5, [6, 5], 0.5);
  if (b) trail(b.tgt.x, b.tgt.y, b.n - 1, css("--target"), 1.5, [6, 5], 0.35);
  trail(a.act.x, a.act.y, sa.i, css("--actual"), 2.5, []);
  if (b) trail(b.act.x, b.act.y, sb.i, css("--runb"), 2.5, [7, 5]);

  // Ghost of the target pose, then the mice on top.
  drawMouse(ctx, sa.tx, sa.ty, sa.tyaw, px, { ghost: true, colour: css("--target") });
  drawMouse(ctx, sa.x, sa.y, sa.yaw, px, { colour: css("--actual"), slipL: sa.slipL, slipR: sa.slipR });
  if (sb) drawMouse(ctx, sb.x, sb.y, sb.yaw, px, { colour: css("--runb"), dashed: true, slipL: sb.slipL, slipR: sb.slipR });

  ctx.restore();
  // Yaw-error arcs and labels in screen space so text is upright.
  const toScreen = (x, y) => [ox + x * s, oy - y * s];
  drawYawError(ctx, toScreen(sa.x, sa.y), sa.yaw, sa.tyaw, css("--actual"), sb ? "A" : "");
  if (sb) drawYawError(ctx, toScreen(sb.x, sb.y), sb.yaw, sb.tyaw, css("--runb"), "B", 1);
  drawScaleBar(ctx, s, W, H);
}

function drawSeam(ctx, run, px) {
  // Where the target path reaches seam-at mm of path length.
  const i = Math.min(lowerBound(run.target, run.seamAt), run.n - 1);
  const x = run.tgt.x[i];
  const y = run.tgt.y[i];
  const h = (run.c.target_deg[i] * Math.PI) / 180;
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(h);
  ctx.fillStyle = hatch(ctx, css("--slip"));
  // Pattern space is the world here; scale it back to screen pixels.
  ctx.save();
  ctx.scale(px, px);
  ctx.fillRect(-CELL_MM / 2 / px, -5 / px, CELL_MM / px, 10 / px);
  ctx.restore();
  ctx.strokeStyle = css("--slip");
  ctx.lineWidth = 1.2 * px;
  ctx.strokeRect(-CELL_MM / 2, -5, CELL_MM, 10);
  ctx.restore();
}

function drawMouse(ctx, x, y, yawDeg, px, o) {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate((yawDeg * Math.PI) / 180);
  // Local frame: +y forward, +x right; origin at the axle centre.
  const w = ROBOT_WIDTH_MM;
  const l = ROBOT_LENGTH_MM;
  const by = BODY_AHEAD_OF_AXLE_MM;
  ctx.lineWidth = 2 * px;
  ctx.strokeStyle = o.colour;
  if (o.ghost) {
    ctx.globalAlpha = 0.55;
    ctx.setLineDash([5 * px, 4 * px]);
    ctx.fillStyle = "rgba(128,128,128,0.12)";
  } else {
    ctx.fillStyle = o.colour;
    ctx.globalAlpha = 0.28;
  }
  if (o.dashed) ctx.setLineDash([6 * px, 4 * px]);
  roundRect(ctx, -w / 2, by - l / 2, w, l, 14);
  ctx.fill();
  ctx.globalAlpha = o.ghost ? 0.55 : 1;
  ctx.stroke();
  ctx.setLineDash([]);
  if (!o.ghost) {
    // Wheels: grey, or the slip colour with a "!" when slipping.
    for (const [side, slip] of [[-1, o.slipL], [1, o.slipR]]) {
      const wx = (side * WHEEL_BASE_MM) / 2;
      ctx.fillStyle = slip ? css("--slip") : css("--post");
      ctx.fillRect(wx - 5, -WHEEL_DIAMETER_MM / 2, 10, WHEEL_DIAMETER_MM);
      if (slip) {
        ctx.save();
        ctx.translate(wx + side * 14, 0);
        ctx.scale(px, -px);
        ctx.fillStyle = css("--slip");
        ctx.font = "bold 16px system-ui";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("!", 0, 0);
        ctx.restore();
      }
    }
  }
  // Heading arrow from the axle centre.
  ctx.strokeStyle = o.ghost ? o.colour : css("--text");
  ctx.fillStyle = ctx.strokeStyle;
  ctx.lineWidth = 2 * px;
  const tip = by + l / 2 + 22;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(0, tip);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(0, tip + 4);
  ctx.lineTo(-9, tip - 12);
  ctx.lineTo(9, tip - 12);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

// Arc from the target heading to the true heading, labelled with the error.
function drawYawError(ctx, [sx, sy], yaw, tyaw, colour, tag, row = 0) {
  const err = yaw - tyaw;
  // Screen angle of a heading: 0 deg = up, CCW positive, canvas y down.
  const ang = (deg) => -Math.PI / 2 - (deg * Math.PI) / 180;
  ctx.save();
  const r = 46;
  if (Math.abs(err) >= 0.2) {
    ctx.strokeStyle = css("--slip");
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(sx, sy, r, ang(tyaw), ang(yaw), err > 0);
    ctx.stroke();
  }
  const label = `${tag ? tag + " " : ""}heading error ${err >= 0 ? "+" : "−"}${Math.abs(err).toFixed(1)}°`;
  ctx.font = "600 14px system-ui";
  const tw = ctx.measureText(label).width;
  // Labels stack in the top-left corner so A's and B's never collide.
  const lx = 12;
  const ly = 24 + row * 22;
  ctx.fillStyle = css("--floor");
  ctx.globalAlpha = 0.8;
  ctx.fillRect(lx - 3, ly - 13, tw + 6, 18);
  ctx.globalAlpha = 1;
  ctx.fillStyle = colour;
  ctx.fillText(label, lx, ly);
  ctx.restore();
}

function drawScaleBar(ctx, s, W, H) {
  const len = CELL_MM * s;
  ctx.save();
  ctx.strokeStyle = css("--muted");
  ctx.fillStyle = css("--muted");
  ctx.lineWidth = 2;
  const x = W - len - 12;
  const y = 28;
  ctx.beginPath();
  ctx.moveTo(x, y - 5);
  ctx.lineTo(x, y);
  ctx.lineTo(x + len, y);
  ctx.lineTo(x + len, y - 5);
  ctx.stroke();
  ctx.font = "13px system-ui";
  ctx.textAlign = "right";
  ctx.fillText("1 cell = 180 mm", x + len, y - 7);
  ctx.restore();
}

function floorKey() {
  const b = state.a && state.cur;
  $("floorKey").innerHTML =
    `<span><i class="swatch" style="border-color:var(--actual)"></i>${b ? "A" : "actual"}</span>` +
    (b ? `<span><i class="swatch" style="border-color:var(--runb);border-top-style:dashed"></i>B</span>` : "") +
    `<span><i class="swatch" style="border-color:var(--target);border-top-style:dashed"></i>target (ghost)</span>` +
    `<span style="color:var(--slip)">! slipping wheel</span>`;
}

// ---------- summary cards ----------

function cards() {
  const [a, b] = runsAB();
  const host = $("cards");
  if (!a) {
    host.innerHTML = "";
    return;
  }
  const last = (r, arr) => arr[r.n - 1];
  const f = (v, unit, d = 1) => `${v >= 0 ? "" : "−"}${Math.abs(v).toFixed(d)}${unit}`;
  const sgn = (v, unit, d = 1) => `${v >= 0 ? "+" : "−"}${Math.abs(v).toFixed(d)}${unit}`;
  const items = [
    ["Final distance", (r) => last(r, r.c.distance_mm), " mm", 1],
    ["Final heading", (r) => last(r, r.c.yaw_deg), "°", 2],
    ["Error at stop", null],
    ["Peak slip", (r) => r.peakSlip, " mm/s", 0],
  ];
  host.innerHTML = items
    .map(([title, get, unit, d]) => {
      if (!get) {
        const e = (r) => [last(r, r.err), last(r, r.c.yaw_deg) - last(r, r.c.target_deg)];
        const [ea, ya] = e(a);
        const bad = (v, lim) => (Math.abs(v) > lim ? "bad" : "");
        let html = `<div class="v"><span class="${bad(ea, 3)}">${sgn(ea, " mm")}</span> <span class="${bad(ya, 1)}">${sgn(ya, "°")}</span></div>`;
        if (b) {
          const [eb, yb] = e(b);
          html += `<div class="b">B ${sgn(eb, " mm")} ${sgn(yb, "°")}</div>`;
        }
        return `<div class="card"><h3>${title}</h3>${html}</div>`;
      }
      let html = `<div class="v">${b ? "A " : ""}${f(get(a), unit, d)}</div>`;
      if (b) {
        html += `<div class="b">B ${f(get(b), unit, d)}</div>`;
        html += `<div class="d" title="B minus A">Δ ${sgn(get(b) - get(a), unit, d)}</div>`;
      }
      return `<div class="card"><h3>${title}</h3>${html}</div>`;
    })
    .join("");
}

// ---------- transport ----------

function duration() {
  const [a, b] = runsAB();
  if (!a) return 0;
  return Math.max(a.duration, b ? b.duration : 0);
}

function seek(t) {
  state.t = Math.min(Math.max(t, 0), duration());
  render();
}

function render() {
  const d = duration();
  $("scrub").value = d ? Math.round((state.t / d) * 1000) : 0;
  $("clock").textContent = `${state.t.toFixed(3)} s / ${d.toFixed(2)} s`;
  drawFloor();
  updatePlayhead();
}

function play() {
  if (state.t >= duration() - 1e-6) state.t = 0;
  state.playing = true;
  state.lastFrame = performance.now();
  $("play").textContent = "❚❚ Pause";
  $("play").setAttribute("aria-label", "Pause");
  requestAnimationFrame(tick);
}

function pause() {
  state.playing = false;
  $("play").textContent = "▶ Play";
  $("play").setAttribute("aria-label", "Play");
}

function tick(now) {
  if (!state.playing) return;
  const dt = Math.min((now - state.lastFrame) / 1000, 0.1);
  state.lastFrame = now;
  state.t += dt * state.speed;
  if (state.t >= duration()) {
    state.t = duration();
    render();
    pause();
    return;
  }
  render();
  requestAnimationFrame(tick);
}

// Puts runs on screen: a (pinned, or null) and cur. fromStart rewinds.
function show(a, cur, fromStart) {
  state.a = a;
  state.cur = cur;
  $("pin").disabled = !cur;
  $("unpin").disabled = !a;
  if (fromStart) state.t = 0;
  state.t = Math.min(state.t, duration());
  buildCharts();
  cards();
  floorKey();
  setSource();
  render();
}

// ---------- wiring ----------

function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  $("theme").textContent = theme === "light" ? "Dark" : "Light";
  try {
    localStorage.setItem("physim-theme", theme);
  } catch (e) {
    /* storage blocked: the toggle still works for this visit */
  }
  if (state.a || state.cur) {
    buildCharts();
    render();
  }
}

function init() {
  buildKnobPanel();
  buildPresets();
  let theme = "dark";
  try {
    theme = localStorage.getItem("physim-theme") || "dark";
  } catch (e) {
    /* default */
  }
  applyTheme(theme);

  $("run").addEventListener("click", runFromPanel);
  $("reset").addEventListener("click", () => writeKnobs($("scenario").value, {}));
  $("pin").addEventListener("click", () => {
    // The pinned run shows alone until the next Run adds B.
    state.req++;
    const run = state.cur || state.a;
    if (!run) return;
    show(run, null, false);
    setStatus("Pinned as A. Change knobs and press Run for B.");
  });
  $("unpin").addEventListener("click", () => {
    state.req++;
    show(null, state.cur || state.a, false);
    setStatus("");
  });
  $("theme").addEventListener("click", () =>
    applyTheme(document.documentElement.dataset.theme === "light" ? "dark" : "light")
  );
  $("play").addEventListener("click", () => (state.playing ? pause() : play()));
  $("scrub").addEventListener("input", (e) => {
    pause();
    seek((e.target.value / 1000) * duration());
  });
  $("speed").addEventListener("change", (e) => (state.speed = +e.target.value));
  $("advanced").addEventListener("change", (e) => {
    state.advanced = e.target.checked;
    buildCharts();
    render();
  });
  $("follow").addEventListener("change", (e) => {
    state.follow = e.target.checked;
    render();
  });
  document.addEventListener("keydown", (e) => {
    if (e.target.matches("input[type=number], select")) return;
    if (e.code === "Space") {
      e.preventDefault();
      state.playing ? pause() : play();
    } else if (e.code === "ArrowRight" || e.code === "ArrowLeft") {
      pause();
      seek(state.t + (e.code === "ArrowRight" ? 0.05 : -0.05));
    }
  });
  let resizeTimer = 0;
  new ResizeObserver(() => {
    cancelAnimationFrame(resizeTimer);
    resizeTimer = requestAnimationFrame(() => {
      sizeCharts();
      drawFloor();
    });
  }).observe(document.body);

  const spec = window.PHYSIM_PRESETS;
  if (spec && spec.presets.length) loadPreset(spec.presets[0]);
  else runFromPanel();
}

init();
