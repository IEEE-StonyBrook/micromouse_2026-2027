// Real vs sim: a TRACE capture of the mouse against physim's two replays of
// it, from server.py's POST /realsim.
//
//   real   the logged run (tools/tracelog.py's columns and dead reckoning)
//   cmd    command replay: physim drives the same PATH through the real Motion
//   volts  volts replay: physim holds the logged volts on the plant, open loop
//
// The volts replay shares the log's sample clock. The command replay starts
// on its own, so it is shifted to start moving (the first FORWARD_ACTIVE or
// ROTATION_ACTIVE sample) when the real run does, then optionally nudged to
// the lag that best correlates the two forward speeds. All series are drawn
// on the real run's sample index; x/y are in the log's frame (x along the
// first heading, y to its left).
"use strict";

const CELL_MM = 180; // firmware/config/geometry.h
const POST_MM = 12;
const WHEEL_BASE_MM = 80.4;
const MOVING = (1 << 5) | (1 << 6); // RunLog FORWARD_ACTIVE | ROTATION_ACTIVE
const NUDGE_MAX = 25; // samples (200 ms at 8 ms) the correlation may shift
const RERUN_DELAY_MS = 250;

// Plant knob sliders; unset = the sim's nominal value (battery: the log's).
const SLIDERS = [
  { key: "friction", label: "Friction", unit: "V", min: 0, max: 3, step: 0.05, nominal: 0.63 },
  { key: "mu", label: "Grip μ", unit: "", min: 0.05, max: 2, step: 0.05, nominal: 1 },
  { key: "battery", label: "Battery", unit: "V", min: 6, max: 9, step: 0.05, nominal: null },
  { key: "counts-per-mm", label: "Encoder", unit: "cnt/mm", min: 1, max: 10, step: 0.02, nominal: 3.08 },
  { key: "gyro-bias", label: "Gyro bias", unit: "°/s", min: -5, max: 5, step: 0.1, nominal: 0 },
  { key: "gyro-noise", label: "Gyro noise", unit: "°/s√Hz", min: 0, max: 2, step: 0.05, nominal: 0 },
  { key: "slip-rate", label: "Slip events", unit: "/s", min: 0, max: 20, step: 0.5, nominal: 0 },
];

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

const state = {
  src: null, // {log: name} or {text, name}
  resp: null, // the last /realsim reply
  knobs: {}, // slider key -> value, set ones only
  view: null, // aligned series, rebuilt from resp
  charts: [],
  hover: null, // real sample index under the chart cursor
  req: 0,
  rerunTimer: 0,
};

// ---------- data ----------

function trace(t) {
  // A /realsim trace: header + {column: [values]} with nulls for NaN.
  const c = t.cols;
  const n = t.header.n;
  const num = (name) => Float64Array.from(c[name] || new Array(n).fill(null), (v) => (v == null ? NaN : v));
  const out = { header: t.header, n, dt: (t.header.dt_ms || 8) / 1000, ev: num("ev"), seg: num("segment") };
  for (const k of ["left_mmps", "right_mmps", "left_v", "right_v", "yaw_unwrapped_deg", "distance_mm", "x_mm", "y_mm"])
    out[k] = num(k);
  out.fwd = out.left_mmps.map((l, i) => (l + out.right_mmps[i]) / 2);
  // Yaw rates, deg/s: from the wheels (what the encoders say the body turned)
  // and from the gyro's heading. A gap between them is a wheel slipping.
  out.encRate = out.left_mmps.map((l, i) => (((out.right_mmps[i] - l) / WHEEL_BASE_MM) * 180) / Math.PI);
  const y = out.yaw_unwrapped_deg;
  out.gyroRate = y.map((_, i) => {
    const a = Math.max(0, i - 1);
    const b = Math.min(n - 1, i + 1);
    return b > a ? (y[b] - y[a]) / ((b - a) * out.dt) : 0;
  });
  return out;
}

function firstMoving(t) {
  for (let i = 0; i < t.n; i++) if (Number.isFinite(t.ev[i]) && t.ev[i] & MOVING) return i;
  return 0;
}

// The lag (cmd index = real index - lag) near `lag0` that best correlates the
// forward speeds, Pearson over the overlap.
function bestLag(real, sim, lag0) {
  let best = lag0;
  let bestR = -Infinity;
  for (let lag = lag0 - NUDGE_MAX; lag <= lag0 + NUDGE_MAX; lag++) {
    let n = 0, sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    for (let i = Math.max(0, lag); i < real.n && i - lag < sim.n; i++) {
      const a = real.fwd[i];
      const b = sim.fwd[i - lag];
      if (!Number.isFinite(a) || !Number.isFinite(b)) continue;
      n++; sa += a; sb += b; saa += a * a; sbb += b * b; sab += a * b;
    }
    if (n < 20) continue;
    const r = (n * sab - sa * sb) / Math.sqrt((n * saa - sa * sa) * (n * sbb - sb * sb) || 1);
    if (r > bestR + 1e-9) {
      bestR = r;
      best = lag;
    }
  }
  return best;
}

// Everything the panels draw, on the real run's sample index.
function buildView(resp) {
  const real = trace(resp.real);
  const cmd = resp.cmd ? trace(resp.cmd) : null;
  const volts = resp.volts ? trace(resp.volts) : null;
  let lag = 0, lag0 = 0;
  if (cmd) {
    lag0 = firstMoving(real) - firstMoving(cmd);
    lag = $("nudge").checked ? bestLag(real, cmd, lag0) : lag0;
  }
  const n = Math.max(real.n, cmd ? cmd.n + lag : 0);
  const x = Array.from({ length: n }, (_, i) => i * real.dt);
  const at = (t, arr, shift = 0) =>
    x.map((_, i) => {
      const j = i - shift;
      return t && j >= 0 && j < t.n && Number.isFinite(arr[j]) ? arr[j] : null;
    });
  return { real, cmd, volts, lag, lag0, n, x, at };
}

// ---------- numbers ----------

function rmse(pairs) {
  let s = 0, n = 0;
  for (const [a, b] of pairs)
    if (Number.isFinite(a) && Number.isFinite(b)) {
      s += (a - b) ** 2;
      n++;
    }
  return n ? Math.sqrt(s / n) : NaN;
}

// [real, sim] pairs of column `k` over real indices [i0, i1].
function pairs(v, sim, shift, k, i0 = 0, i1 = v.real.n - 1) {
  const out = [];
  if (!sim) return out;
  for (let i = i0; i <= i1; i++) {
    const j = i - shift;
    if (j >= 0 && j < sim.n) out.push([v.real[k][i], sim[k][j]]);
  }
  return out;
}

const wheelRmse = (v, sim, shift, i0, i1) =>
  Math.sqrt(
    (rmse(pairs(v, sim, shift, "left_mmps", i0, i1)) ** 2 + rmse(pairs(v, sim, shift, "right_mmps", i0, i1)) ** 2) / 2
  );

// Each segment's [first, last] sample, in the order they ran.
function spans(t) {
  const out = new Map();
  for (let i = 0; i < t.n; i++) {
    const s = t.seg[i];
    if (!Number.isFinite(s)) continue;
    if (!out.has(s)) out.set(s, [i, i]);
    else out.get(s)[1] = i;
  }
  return out;
}

function segmentRows(v) {
  const rs = spans(v.real);
  const cs = v.cmd ? spans(v.cmd) : new Map();
  const rows = [];
  for (const [s, [i0, i1]] of rs) {
    const c = cs.get(s);
    const dur = (i1 - i0 + 1) * v.real.dt;
    const dist = v.real.distance_mm[i1] - v.real.distance_mm[i0];
    const row = { s, dur, dist, simDur: NaN, simDist: NaN, yawErr: NaN };
    if (c) {
      row.simDur = (c[1] - c[0] + 1) * v.cmd.dt;
      row.simDist = v.cmd.distance_mm[c[1]] - v.cmd.distance_mm[c[0]];
      row.yawErr = v.cmd.yaw_unwrapped_deg[c[1]] - v.real.yaw_unwrapped_deg[i1];
    }
    row.cmdRmse = wheelRmse(v, v.cmd, v.lag, i0, i1);
    row.voltsRmse = wheelRmse(v, v.volts, 0, i0, i1);
    rows.push(row);
  }
  return rows;
}

const f = (x, d = 0) => (Number.isFinite(x) ? x.toFixed(d) : "–");
const sign = (x, d = 0) => (Number.isFinite(x) ? (x >= 0 ? "+" : "") + x.toFixed(d) : "–");

function renderTable(v) {
  const rows = segmentRows(v);
  const name = (s) => (s === 0 ? "0 start" : String(s));
  $("segs").innerHTML =
    `<thead><tr><th>seg</th><th>time s<br><span class="unit">real / sim</span></th>` +
    `<th>dist mm<br><span class="unit">real / sim</span></th><th>end yaw<br><span class="unit">sim−real °</span></th>` +
    `<th>RMSE cmd<br><span class="unit">mm/s</span></th><th>RMSE volts<br><span class="unit">mm/s</span></th></tr></thead><tbody>` +
    rows
      .map(
        (r) =>
          `<tr><td>${name(r.s)}</td><td>${f(r.dur, 2)} / ${f(r.simDur, 2)}</td><td>${f(r.dist)} / ${f(r.simDist)}</td>` +
          `<td class="${Math.abs(r.yawErr) > 3 ? "bad" : ""}">${sign(r.yawErr, 1)}</td><td>${f(r.cmdRmse)}</td><td>${f(r.voltsRmse)}</td></tr>`
      )
      .join("") +
    "</tbody>";
}

function renderCards(v) {
  const { real, cmd, volts } = v;
  const end = (t) => (t ? [t.x_mm[t.n - 1], t.y_mm[t.n - 1], t.yaw_unwrapped_deg[t.n - 1]] : [NaN, NaN, NaN]);
  const [rx, ry, ryaw] = end(real);
  const [cx, cy, cyaw] = end(cmd);
  const slips = slipBands(v).reduce((s, [a, b]) => s + (b - a), 0);
  const card = (title, value, sub, bad = false) =>
    `<div class="card"><h3>${title}</h3><div class="v ${bad ? "bad" : ""}">${value}</div><div class="d">${sub}</div></div>`;
  const cmdL = rmse(pairs(v, cmd, v.lag, "left_mmps"));
  const cmdR = rmse(pairs(v, cmd, v.lag, "right_mmps"));
  const vL = rmse(pairs(v, volts, 0, "left_mmps"));
  const vR = rmse(pairs(v, volts, 0, "right_mmps"));
  const endGap = Math.hypot(cx - rx, cy - ry);
  $("cards").innerHTML = [
    card("Cmd replay wheel RMSE", `${f(cmdL)} / ${f(cmdR)} mm/s`, "left / right"),
    card("Volts replay wheel RMSE", `${f(vL)} / ${f(vR)} mm/s`, "plant only, open loop"),
    card("Cmd yaw / volts RMSE", `${f(rmse(pairs(v, cmd, v.lag, "yaw_unwrapped_deg")), 1)}° · ${f(rmse(pairs(v, cmd, v.lag, "left_v")), 2)} V`, "yaw · left volts"),
    card("End pose, sim − real", `${f(endGap)} mm · ${sign(cyaw - ryaw, 1)}°`, `real ends ${f(rx)}, ${f(ry)} mm`, endGap > 30),
    card("Duration", `${f(real.n * real.dt, 2)} / ${cmd ? f(cmd.n * cmd.dt, 2) : "–"} s`, "real / cmd replay"),
    card("Slip flagged", `${f(slips, 2)} s`, `encoder vs gyro yaw rate over ${$("slipThr").value}°/s`, slips > 0),
  ].join("");
}

// Spans (s) where the wheels' yaw rate and the gyro's disagree by more than the threshold.
function slipBands(v) {
  const thr = +$("slipThr").value || 40;
  const r = v.real;
  const out = [];
  let start = null;
  for (let i = 0; i < r.n; i++) {
    const bad = Math.abs(r.encRate[i] - r.gyroRate[i]) > thr;
    if (bad && start === null) start = i;
    if (!bad && start !== null) {
      out.push([start * r.dt, i * r.dt]);
      start = null;
    }
  }
  if (start !== null) out.push([start * r.dt, r.n * r.dt]);
  return out;
}

// ---------- floor ----------

function drawFloor() {
  const canvas = $("floor");
  const v = state.view;
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth;
  const H = canvas.clientHeight;
  canvas.width = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = css("--floor");
  ctx.fillRect(0, 0, W, H);
  if (!v) return;
  const paths = [
    [v.real, "--actual", [], 2.5, 0],
    [v.cmd, "--runb", [6, 4], 2, v.lag],
    [v.volts, "--imu", [2, 3], 1.5, 0],
  ].filter(([t]) => t);

  // The first cell's centre is where segment 1 starts; without segments, the start.
  const s1 = Array.from(v.real.seg).indexOf(1);
  const cx0 = s1 >= 0 ? v.real.x_mm[s1] : 0;
  const cy0 = s1 >= 0 ? v.real.y_mm[s1] : 0;
  let lo = [Infinity, Infinity], hi = [-Infinity, -Infinity];
  for (const [t] of paths)
    for (let i = 0; i < t.n; i++) {
      if (!Number.isFinite(t.x_mm[i])) continue;
      lo = [Math.min(lo[0], t.x_mm[i]), Math.min(lo[1], t.y_mm[i])];
      hi = [Math.max(hi[0], t.x_mm[i]), Math.max(hi[1], t.y_mm[i])];
    }
  // Whole cells around everything, plus one.
  const gx0 = cx0 + CELL_MM * (Math.floor((lo[0] - cx0) / CELL_MM - 0.5) - 0.5);
  const gx1 = cx0 + CELL_MM * (Math.ceil((hi[0] - cx0) / CELL_MM + 0.5) + 0.5);
  const gy0 = cy0 + CELL_MM * (Math.floor((lo[1] - cy0) / CELL_MM - 0.5) - 0.5);
  const gy1 = cy0 + CELL_MM * (Math.ceil((hi[1] - cy0) / CELL_MM + 0.5) + 0.5);
  const pad = 10;
  const s = Math.min((W - 2 * pad) / (gx1 - gx0), (H - 2 * pad) / (gy1 - gy0));
  const ox = (W - s * (gx1 - gx0)) / 2;
  const oy = (H - s * (gy1 - gy0)) / 2;
  // x right, y (the log's left) up.
  const P = (x, y) => [ox + (x - gx0) * s, H - oy - (y - gy0) * s];

  ctx.strokeStyle = css("--cell");
  ctx.lineWidth = 1;
  for (let x = gx0; x <= gx1 + 1e-6; x += CELL_MM) {
    ctx.beginPath();
    ctx.moveTo(...P(x, gy0));
    ctx.lineTo(...P(x, gy1));
    ctx.stroke();
  }
  for (let y = gy0; y <= gy1 + 1e-6; y += CELL_MM) {
    ctx.beginPath();
    ctx.moveTo(...P(gx0, y));
    ctx.lineTo(...P(gx1, y));
    ctx.stroke();
  }
  ctx.fillStyle = css("--post");
  const ps = Math.max(POST_MM * s, 2);
  for (let x = gx0; x <= gx1 + 1e-6; x += CELL_MM)
    for (let y = gy0; y <= gy1 + 1e-6; y += CELL_MM) {
      const [px, py] = P(x, y);
      ctx.fillRect(px - ps / 2, py - ps / 2, ps, ps);
    }

  for (const [t, colour, dash, width] of paths) {
    ctx.strokeStyle = css(colour);
    ctx.lineWidth = width;
    ctx.setLineDash(dash);
    ctx.beginPath();
    let pen = false;
    for (let i = 0; i < t.n; i++) {
      if (!Number.isFinite(t.x_mm[i])) {
        pen = false;
        continue;
      }
      const p = P(t.x_mm[i], t.y_mm[i]);
      pen ? ctx.lineTo(...p) : ctx.moveTo(...p);
      pen = true;
    }
    ctx.stroke();
    ctx.setLineDash([]);
    // Segment markers: a dot where each segment starts, numbered on the real path.
    ctx.fillStyle = css(colour);
    const labels = []; // [px, py, text]: a spin starts the next segment on the same spot
    for (let i = 1; i < t.n; i++) {
      if (t.seg[i] === t.seg[i - 1] || !Number.isFinite(t.x_mm[i])) continue;
      const [px, py] = P(t.x_mm[i], t.y_mm[i]);
      ctx.beginPath();
      ctx.arc(px, py, t === v.real ? 4 : 3, 0, 2 * Math.PI);
      ctx.fill();
      if (t !== v.real) continue;
      const near = labels.find(([lx, ly]) => Math.hypot(lx - px, ly - py) < 12);
      if (near) near[2] += "," + t.seg[i];
      else labels.push([px, py, String(t.seg[i])]);
    }
    ctx.font = "12px system-ui";
    ctx.fillStyle = css("--text");
    for (const [px, py, text] of labels) ctx.fillText(text, px + 6, py - 6);
  }
  // The chart cursor's instant on each path.
  if (state.hover != null) {
    for (const [t, colour, , , shift] of paths) {
      const j = state.hover - shift;
      if (j < 0 || j >= t.n || !Number.isFinite(t.x_mm[j])) continue;
      const [px, py] = P(t.x_mm[j], t.y_mm[j]);
      const h = (t.yaw_unwrapped_deg[j] - t.yaw_unwrapped_deg[0]) * (Math.PI / 180);
      ctx.strokeStyle = css(colour);
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(px, py, 7, 0, 2 * Math.PI);
      ctx.moveTo(px, py);
      ctx.lineTo(px + 14 * Math.cos(h), py - 14 * Math.sin(h));
      ctx.stroke();
    }
  }
  $("floorKey").innerHTML =
    `<span><i class="swatch" style="border-color:${css("--actual")}"></i>real</span>` +
    (v.cmd ? `<span><i class="swatch dashed" style="border-color:${css("--runb")}"></i>cmd replay</span>` : "") +
    (v.volts ? `<span><i class="swatch dotted" style="border-color:${css("--imu")}"></i>volts replay</span>` : "") +
    `<span>${CELL_MM} mm cells</span>`;
}

// ---------- charts ----------

function chartSpecs(v) {
  const { real, cmd, volts, lag, at } = v;
  const three = (k) => [
    ["real", "--actual", at(real, real[k]), {}],
    ["cmd", "--runb", at(cmd, cmd ? cmd[k] : [], lag), { dash: [6, 4] }],
    ["volts", "--imu", at(volts, volts ? volts[k] : []), { dash: [2, 3], width: 1.5 }],
  ];
  return [
    { title: "Left wheel (mm/s): real vs cmd replay vs volts replay", series: three("left_mmps") },
    { title: "Right wheel (mm/s)", series: three("right_mmps") },
    {
      title: "Volts: real vs cmd replay (volts replay holds the real ones)",
      series: [
        ["real L", "--actual", at(real, real.left_v), {}],
        ["real R", "--right", at(real, real.right_v), {}],
        ["sim L", "--runb", at(cmd, cmd ? cmd.left_v : [], lag), { dash: [6, 4] }],
        ["sim R", "--slip", at(cmd, cmd ? cmd.right_v : [], lag), { dash: [6, 4] }],
      ],
    },
    { title: "Yaw (°, unwrapped)", series: three("yaw_unwrapped_deg") },
    {
      title: "Slip check, real: wheel yaw rate vs gyro yaw rate (°/s); hatched where they disagree",
      bands: slipBands(v),
      series: [
        ["wheels", "--runb", at(real, real.encRate), { width: 1.5 }],
        ["gyro", "--imu", at(real, real.gyroRate), {}],
      ],
    },
  ];
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

function buildCharts() {
  const host = $("charts");
  host.style.minHeight = host.offsetHeight + "px"; // the page keeps its height, and its scroll
  for (const ch of state.charts) ch.destroy();
  state.charts = [];
  host.innerHTML = "";
  const v = state.view;
  if (!v) return;
  const grid = css("--cell");
  const text = css("--muted");
  const specs = chartSpecs(v);
  specs.forEach((spec) => {
    const box = document.createElement("div");
    box.className = "chart";
    box.innerHTML = `<div class="head"><h2>${spec.title}</h2></div><div class="plot"></div>`;
    host.appendChild(box);
    const used = spec.series.filter(([, , data]) => data.some((d) => d != null));
    const opts = {
      width: Math.max(box.clientWidth - 8, 200),
      height: 150,
      cursor: { sync: { key: "realsim", setSeries: false }, drag: { x: true, y: false }, points: { size: 6 } },
      legend: { live: true },
      scales: { x: { time: false } },
      axes: [
        { stroke: text, grid: { stroke: grid }, ticks: { stroke: grid }, size: 28, font: "12px system-ui" },
        { stroke: text, grid: { stroke: grid }, ticks: { stroke: grid }, size: 48, font: "12px system-ui" },
      ],
      series: [
        { label: "t (s)", value: (u, x) => (x == null ? "" : x.toFixed(3)) },
        ...used.map(([label, colour, , o]) => ({
          label,
          stroke: css(colour),
          width: o.width ?? 2,
          dash: o.dash,
          spanGaps: false,
          points: { show: false },
          value: (u, x) => (x == null ? "–" : x.toFixed(1)),
        })),
      ],
      hooks: {
        drawClear: [
          (u) => {
            if (!spec.bands || !spec.bands.length) return;
            const { top, height } = u.bbox;
            u.ctx.save();
            u.ctx.fillStyle = hatch(u.ctx, css("--slip"));
            for (const [t0, t1] of spec.bands) {
              const x0 = u.valToPos(t0, "x", true);
              const x1 = Math.max(u.valToPos(t1, "x", true), x0 + 2);
              u.ctx.fillRect(x0, top, x1 - x0, height);
            }
            u.ctx.restore();
          },
        ],
        setCursor: [
          (u) => {
            const i = u.cursor.idx;
            if (i !== state.hover) {
              state.hover = i ?? null;
              drawFloor();
            }
          },
        ],
      },
    };
    const u = new uPlot(opts, [v.x, ...used.map(([, , data]) => data)], box.querySelector(".plot"));
    box.querySelector(".head").appendChild(u.root.querySelector(".u-legend"));
    state.charts.push(u);
  });
  host.style.minHeight = "";
}

function sizeCharts() {
  for (const u of state.charts) {
    const w = u.root.parentElement.parentElement.clientWidth - 8;
    u.setSize({ width: Math.max(w, 200), height: 150 });
  }
}

// ---------- loading ----------

function setStatus(text, error = false) {
  $("status").textContent = text;
  $("status").classList.toggle("error", error);
}

function render() {
  const v = state.view;
  if (!v) return;
  $("align").textContent = v.cmd
    ? `Cmd replay shifted ${sign(v.lag * v.real.dt * 1000)} ms (first motion ${sign(v.lag0 * v.real.dt * 1000)} ms` +
      (v.lag !== v.lag0 ? `, nudged ${sign((v.lag - v.lag0) * v.real.dt * 1000)} ms)` : ")")
    : "";
  renderCards(v);
  renderTable(v);
  buildCharts();
  drawFloor();
}

async function load() {
  if (!state.src) return;
  const req = ++state.req;
  const body = { ...state.src, knobs: state.knobs };
  const run = $("runPick").value;
  if (run !== "") body.run = +run;
  const path = $("path").value.trim();
  if (path && path !== $("path").dataset.fromLog) body.path = path;
  setStatus("Replaying in the sim…");
  $("rerun").disabled = true;
  try {
    const r = await fetch("realsim", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
      cache: "no-store",
    });
    const data = await r.json().catch(() => ({ error: `HTTP ${r.status}` }));
    if (req !== state.req) return; // a later request won
    if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
    state.resp = data;
    fillRuns(data);
    if (!path || path === $("path").dataset.fromLog) {
      $("path").value = data.path || "";
      $("path").dataset.fromLog = data.path || "";
    }
    SLIDERS.forEach(showSlider); // the battery's default is the log's
    state.view = buildView(data);
    render();
    const errs = Object.entries(data.errors || {}).map(([k, e]) => `${k}: ${e}`);
    const h = data.real.header;
    $("caption").textContent =
      `${state.src.log || state.src.name}, run ${data.index}: ${h.cmd ? `"${h.cmd}"` : "no cmd in the header"}, ` +
      `${h.n} samples at ${h.dt_ms} ms${Number.isFinite(h.batt_v) ? `, ${h.batt_v} V` : ""}${h.wrapped ? " (wrapped: the run's start is lost)" : ""}.`;
    setStatus(errs.length ? errs.join("; ") : `Replayed with ${knobText()}.`, errs.length > 0);
    $("source").textContent = "live sim";
  } catch (e) {
    if (req !== state.req) return;
    setStatus(`Failed: ${e.message}`, true);
    $("source").textContent = "server unavailable";
  } finally {
    if (req === state.req) $("rerun").disabled = false;
  }
}

function knobText() {
  const set = Object.entries(state.knobs).map(([k, v]) => `${k} ${v}`);
  return set.length ? set.join(", ") : "nominal knobs";
}

function fillRuns(data) {
  const pick = $("runPick");
  const want = String(data.index);
  pick.innerHTML = data.runs
    .map((r) => `<option value="${r.index}">#${r.index}${r.run != null ? " run " + r.run : ""}: ${r.n} rows${r.cmd ? " · " + escapeHtml(r.cmd.slice(0, 40)) : ""}</option>`)
    .join("");
  pick.value = want;
  pick.disabled = data.runs.length < 2;
}

function escapeHtml(s) {
  return s.replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]);
}

function newSource(src) {
  state.src = src;
  $("runPick").innerHTML = "";
  $("path").value = "";
  $("path").dataset.fromLog = "";
  load();
}

async function listLogs() {
  const pick = $("logPick");
  try {
    const r = await fetch("logs", { cache: "no-store" });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const { logs } = await r.json();
    pick.innerHTML =
      `<option value="">(choose)</option>` + logs.map((l) => `<option value="${escapeHtml(l.name)}">${escapeHtml(l.name)}</option>`).join("");
    const first = logs.find((l) => l.name === "logs/latest.log") || logs.find((l) => l.name.startsWith("fixtures/"));
    if (first) {
      pick.value = first.name;
      newSource({ log: first.name });
    }
  } catch (e) {
    pick.innerHTML = `<option value="">(no server)</option>`;
    $("source").textContent = "server unavailable";
    setStatus("No server: run python3 tools/physim_gui/server.py and reload. This view has no offline fallback.", true);
  }
}

function readFile(file) {
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    $("logPick").value = "";
    newSource({ text: String(reader.result), name: file.name });
  };
  reader.readAsText(file);
}

// ---------- knobs ----------

function buildSliders() {
  const host = $("sliders");
  for (const k of SLIDERS) {
    const row = document.createElement("div");
    row.className = "slider";
    row.innerHTML =
      `<div class="sl-head"><span>${k.label} <span class="unit">${k.unit}</span></span>` +
      `<output id="o-${k.key}"></output><button type="button" class="ghost tiny" data-k="${k.key}" title="Back to nominal">×</button></div>` +
      `<input type="range" id="s-${k.key}" min="${k.min}" max="${k.max}" step="${k.step}" aria-label="${k.label}">`;
    host.appendChild(row);
    const input = row.querySelector("input");
    input.addEventListener("input", () => {
      state.knobs[k.key] = +input.value;
      showSlider(k);
    });
    input.addEventListener("change", scheduleRerun);
    row.querySelector("button").addEventListener("click", () => {
      delete state.knobs[k.key];
      showSlider(k);
      scheduleRerun();
    });
    showSlider(k);
  }
}

function showSlider(k) {
  const set = k.key in state.knobs;
  const input = $("s-" + k.key);
  const nominal = k.nominal ?? state.resp?.real?.header?.batt_v ?? (k.min + k.max) / 2;
  if (!set) input.value = nominal;
  $("o-" + k.key).textContent = set ? String(state.knobs[k.key]) : k.nominal == null ? "log's" : "nominal";
  input.parentElement.classList.toggle("changed", set);
}

function scheduleRerun() {
  clearTimeout(state.rerunTimer);
  state.rerunTimer = setTimeout(load, RERUN_DELAY_MS);
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
  if (state.view) render();
}

function init() {
  let theme = "dark";
  try {
    theme = localStorage.getItem("physim-theme") || "dark";
  } catch (e) {
    /* default */
  }
  applyTheme(theme);
  buildSliders();

  $("theme").addEventListener("click", () =>
    applyTheme(document.documentElement.dataset.theme === "light" ? "dark" : "light")
  );
  $("logPick").addEventListener("change", (e) => e.target.value && newSource({ log: e.target.value }));
  $("file").addEventListener("change", (e) => readFile(e.target.files[0]));
  $("runPick").addEventListener("change", () => {
    $("path").value = "";
    $("path").dataset.fromLog = "";
    load();
  });
  $("path").addEventListener("keydown", (e) => e.key === "Enter" && load());
  $("rerun").addEventListener("click", load);
  $("nominal").addEventListener("click", () => {
    state.knobs = {};
    SLIDERS.forEach(showSlider);
    load();
  });
  $("nudge").addEventListener("change", () => {
    if (state.resp) {
      state.view = buildView(state.resp);
      render();
    }
  });
  $("slipThr").addEventListener("change", () => state.view && render());
  for (const ev of ["dragenter", "dragover"])
    document.addEventListener(ev, (e) => {
      e.preventDefault();
      $("drop").classList.add("over");
    });
  for (const ev of ["dragleave", "drop"])
    document.addEventListener(ev, (e) => {
      e.preventDefault();
      $("drop").classList.remove("over");
    });
  document.addEventListener("drop", (e) => readFile(e.dataTransfer.files[0]));

  let resizeTimer = 0;
  new ResizeObserver(() => {
    cancelAnimationFrame(resizeTimer);
    resizeTimer = requestAnimationFrame(() => {
      sizeCharts();
      drawFloor();
    });
  }).observe(document.body);
  listLogs();
}

init();
