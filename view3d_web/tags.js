// tags.js - the PLC tag monitor of the 3D page: every PLC tag of the layout
// (its name in the AX code, TIA address, type, comment) with its value now,
// and charts of them (Chart.js): tags over the simulated time, or one tag
// against another as an x/y trajectory (a tracked position). "PLC values"
// are tags the PLC writes only to be shown (config_parser.PlcValue);
// "Tracked loads" the slots the PLC shows its tracked loads in, with the
// simulator's check of each (config_parser.TrackedLoadView: alongError,
// lateralError over time, or loadX / loadY as a trajectory).
//
// The values come from the simulator (view3d.py): GET tags/meta once per
// layout, GET tags/data?since=N twice a second while the panel is open
// (20 samples a second, the last 2 minutes kept there).
//
//   import * as tags from "./tags.js";
//   tags.toggle();      // show / hide the panel (button "Tags", key P)

const POLL_MS = 500;
const COLORS = ["#3898d6", "#de4c40", "#40b268", "#e2a82c", "#9b6cd6", "#3ab5b0", "#d6803a", "#c9c04a"];

let panel = null;
let meta = null;                 // {version, summary, tags: [...]}
let samples = [];                // [seq, simTime, [numbers]]
let texts = [];
let seq = 0;
let timer = null;
let chart = null;
const ui = { view: "all", filter: "", plotted: new Set(), mode: "time", x: -1, y: -1, window: 30, paused: false };

function el(tag, attrs = {}, ...children) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") e.className = v;
    else if (k.startsWith("on")) e[k] = v;
    else e.setAttribute(k, v);
  }
  for (const c of children) e.append(c);
  return e;
}

async function loadMeta() {
  meta = await (await fetch("tags/meta", { cache: "no-store" })).json();
  samples = [];
  seq = 0;
  // Nothing is ticked for you: the chart plots what you tick (and keeps it across a reload of the tags).
  ui.plotted = new Set([...ui.plotted].filter((i) => i < meta.tags.length));
  const values = meta.tags.map((t, i) => (t.shownOnly ? i : -1)).filter((i) => i >= 0);
  if (ui.x < 0 && values.length >= 2) { ui.x = values[0]; ui.y = values[1]; }
  render();
}

// The "Tracked loads" view: its x / y chart starts on the first slot's
// position (loadX, loadY), the tracked path of what that slot shows.
function trackedDefaults() {
  if (ui.view !== "tracked" || !meta) return;
  const find = (name) => meta.tags.findIndex((t) => t.tracked && t.name.endsWith(name));
  const x = find("_loadX"), y = find("_loadY");
  if (x >= 0 && y >= 0 && !(meta.tags[ui.x] || {}).tracked) { ui.x = x; ui.y = y; }
}

async function poll() {
  try {
    if (!meta) await loadMeta();
    const data = await (await fetch(`tags/data?since=${seq}`, { cache: "no-store" })).json();
    if (data.version !== meta.version) { await loadMeta(); return; }
    for (const s of data.samples) samples.push(s);
    if (samples.length > 4000) samples.splice(0, samples.length - 4000);
    seq = data.seq;
    texts = data.texts || texts;
    updateTable();
    if (!ui.paused) updateChart();
  } catch (err) {
    const status = panel && panel.querySelector(".tstatus");
    if (status) status.textContent = `no data: ${err}`;
  }
}

function shown() {
  if (!meta) return [];
  const f = ui.filter.toLowerCase();
  return meta.tags.map((t, i) => [t, i]).filter(([t]) =>
    (ui.view === "all" || (ui.view === "inputs" && t.area === "I") || (ui.view === "outputs" && t.area === "Q")
     || (ui.view === "values" && t.shownOnly) || (ui.view === "tracked" && t.tracked))
    && (!f || t.name.toLowerCase().includes(f) || (t.comment || "").toLowerCase().includes(f)));
}

function updateTable() {
  if (!panel) return;
  for (const row of panel.querySelectorAll("tr[data-i]")) {
    const i = +row.dataset.i;
    const cell = row.querySelector(".tval");
    const text = texts[i] ?? "-";
    if (cell.textContent !== text) {
      cell.textContent = text;
      row.classList.remove("tchanged");
      void row.offsetWidth;                          // restart the highlight
      row.classList.add("tchanged");
    }
    cell.classList.toggle("ton", text === "TRUE" || (text.length > 1 && /^[01]+$/.test(text) && text.includes("1")));
  }
  const status = panel.querySelector(".tstatus");
  const last = samples[samples.length - 1];
  if (status) status.textContent = last ? `sim ${last[1].toFixed(1)} s · ${samples.length} samples` : "waiting for data";
}

const GAP_S = 0.5;               // samples further apart than this: not recorded in between (the line breaks)

function windowed() {
  if (!samples.length) return [];
  const end = samples[samples.length - 1][1];
  return samples.filter((s) => s[1] >= end - ui.window);
}

// Points for one series, with a null where the record has a gap (or the
// simulation was reset): Chart.js breaks the line there instead of drawing
// a flat line across time nobody recorded.
function points(rows, point) {
  const out = [];
  let last = null;
  for (const s of rows) {
    if (last !== null && (s[1] - last > GAP_S || s[1] < last)) out.push({ x: (last + s[1]) / 2, y: null });
    out.push(point(s));
    last = s[1];
  }
  return out;
}

function updateChart() {
  if (!chart) return;
  const rows = windowed();
  if (ui.mode === "time") {
    const ids = [...ui.plotted];
    chart.data.datasets = ids.map((i, k) => ({
      label: meta.tags[i].name, borderColor: COLORS[k % COLORS.length], backgroundColor: COLORS[k % COLORS.length],
      data: points(rows, (s) => ({ x: s[1], y: s[2][i] })), pointRadius: 0, borderWidth: 1.5, spanGaps: false,
      stepped: meta.tags[i].type === "Bool" ? "before" : false,
    }));
    chart.options.scales.x.title.text = "simulated time (s)";
    chart.options.scales.y.title.text = "";
  } else if (ui.x >= 0 && ui.y >= 0) {
    const pts = rows.map((s) => ({ x: s[2][ui.x], y: s[2][ui.y] }));
    chart.data.datasets = [
      { label: `${meta.tags[ui.y].name} over ${meta.tags[ui.x].name}`, data: pts, showLine: true, borderWidth: 1.5,
        pointRadius: 0, borderColor: COLORS[0], backgroundColor: COLORS[0] },
      { label: "now", data: pts.slice(-1), pointRadius: 5, borderColor: COLORS[1], backgroundColor: COLORS[1] },
    ];
    const unit = (i) => { const m = /\[([^\],]+)/.exec(meta.tags[i].comment || ""); return m ? ` (${m[1]})` : ""; };
    chart.options.scales.x.title.text = meta.tags[ui.x].name + unit(ui.x);
    chart.options.scales.y.title.text = meta.tags[ui.y].name + unit(ui.y);
  }
  chart.update("none");
}

function makeChart(canvas) {
  if (chart) chart.destroy();
  chart = null;
  if (!window.Chart) {
    canvas.replaceWith(el("div", { class: "tstatus" }, "Charts need Chart.js (no internet? put chart.umd.min.js in view3d_web/vendor/chart.js)"));
    return;
  }
  const grid = "rgba(140,150,160,0.18)", text = "#8b949e";
  chart = new window.Chart(canvas, {
    type: ui.mode === "time" ? "line" : "scatter",
    data: { datasets: [] },
    options: {
      animation: false, responsive: true, maintainAspectRatio: false, parsing: false, normalized: true,
      plugins: { legend: { labels: { color: "#d6dbe1", boxWidth: 12 } } },
      scales: {
        x: { type: "linear", grid: { color: grid }, ticks: { color: text }, title: { display: true, color: text, text: "" } },
        y: { type: "linear", grid: { color: grid }, ticks: { color: text }, title: { display: true, color: text, text: "" } },
      },
    },
  });
  updateChart();
}

function render() {
  if (!panel || !meta) return;
  panel.innerHTML = "";
  const tagOptions = (selected) => meta.tags.map((t, i) => {
    const o = new Option(t.name, i);
    o.selected = i === selected;
    return o;
  });
  const head = el("div", { class: "thead" },
    el("b", {}, "PLC tags"),
    el("span", { class: "tstatus" }, "loading"),
    el("button", { onclick: () => toggle() }, "x"));
  const views = el("div", { class: "tbar" },
    ...[["all", "All"], ["inputs", "Inputs %I"], ["outputs", "Outputs %Q"], ["values", "PLC values"],
        ["tracked", "Tracked loads"]].map(([v, label]) =>
      el("button", { class: ui.view === v ? "on" : "", onclick: () => { ui.view = v; trackedDefaults(); render(); } }, label)),
    (() => {
      const f = el("input", { type: "search", placeholder: "filter", value: ui.filter });
      f.oninput = () => { ui.filter = f.value; renderRows(); };
      return f;
    })());
  const rows = el("tbody");
  const table = el("table", { class: "ttable" },
    el("thead", {}, el("tr", {}, el("th", {}, "plot"), el("th", {}, "Name"), el("th", {}, "Address"),
                       el("th", {}, "Type"), el("th", {}, "Value"))), rows);
  function renderRows() {
    rows.innerHTML = "";
    for (const [t, i] of shown()) {
      const box = el("input", { type: "checkbox", title: "plot over time" });
      box.checked = ui.plotted.has(i);
      box.onchange = () => { box.checked ? ui.plotted.add(i) : ui.plotted.delete(i); updateChart(); };
      const tr = el("tr", { "data-i": i, title: t.comment || "" },
        el("td", {}, box), el("td", { class: t.shownOnly ? "tshown" : "" }, t.name), el("td", { class: "tdim" }, t.address),
        el("td", { class: "tdim" }, t.type), el("td", { class: "tval" }, texts[i] ?? "-"));
      rows.append(tr);
    }
    updateTable();
  }
  const modeSel = el("select", {}, new Option("over time", "time"), new Option("x / y trajectory", "xy"));
  modeSel.value = ui.mode;
  modeSel.onchange = () => { ui.mode = modeSel.value; render(); };
  const xSel = el("select", {}, ...tagOptions(ui.x));
  const ySel = el("select", {}, ...tagOptions(ui.y));
  xSel.onchange = () => { ui.x = +xSel.value; updateChart(); };
  ySel.onchange = () => { ui.y = +ySel.value; updateChart(); };
  const win = el("input", { type: "number", min: 2, max: 120, step: 1, value: ui.window, title: "seconds shown" });
  win.onchange = () => { ui.window = Math.max(2, Math.min(120, +win.value || 30)); updateChart(); };
  const pause = el("button", { class: ui.paused ? "on" : "" }, ui.paused ? "Paused" : "Pause");
  pause.onclick = () => { ui.paused = !ui.paused; pause.textContent = ui.paused ? "Paused" : "Pause"; pause.classList.toggle("on", ui.paused); };
  const chartBar = el("div", { class: "tbar" }, "Chart ", modeSel,
    ...(ui.mode === "xy" ? [" x ", xSel, " y ", ySel] : [el("span", { class: "tdim" }, " the ticked tags")]),
    " last ", win, " s ", pause);
  const canvas = el("canvas");
  panel.append(head, views, el("div", { class: "twrap" }, table), chartBar, el("div", { class: "tchart" }, canvas));
  renderRows();
  makeChart(canvas);
}

export function toggle() {
  panel = panel || document.getElementById("tags");
  const open = panel.style.display !== "block";
  panel.style.display = open ? "block" : "none";
  clearInterval(timer);
  timer = null;
  if (open) {
    meta = null;
    loadMeta().then(poll).catch(() => {});
    timer = setInterval(poll, POLL_MS);
  } else if (chart) {
    chart.destroy();
    chart = null;
  }
}

export const state = ui;   // for checks in the browser console (window.view3d.tags)
