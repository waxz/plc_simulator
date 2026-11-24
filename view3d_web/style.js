// style.js - the 3D view's style: colours, sizes and heights, per kind of
// model and per component (by id). Kept next to the layout, in
// <layout>.view3d.json (GET / POST "style", view3d.py); without a layout
// file, in this browser (localStorage).
//
//   import * as style from "./style.js";
//   await style.load();                 // the saved style over the defaults
//   style.S.belt.thickness              // a value
//   style.item("belt_2")                // a component's own values: {color, thickness, height, hidden}
//   style.openPanel(onChange)           // the Style panel; onChange() after every edit
//
// Only the drawing changes: where loads ride and the conveyor height (the
// layout's, heights.py) stay the simulation's.

export const DEFAULTS = {
  scene: { background: "#1f2329", floor: "#262b32", grid: "#30363e", gridMajor: "#3a414b", light: 1.8,
           ambient: 1.4, labelSize: 1.0 },
  belt: { surface: "#3a3f46", frame: "", thickness: 0.12, stands: true, standColor: "#9aa3ad", chevrons: true,
          chevronColor: "#50575f" },
  roller: { color: "" },
  parcel: { colorMode: "layout", color: "#d9a441", heightScale: 1.0 },
  sensor: { on: "#ff3b30", off: "#2f6b45", postHeight: 0.35, bannerHead: 0.75 },
  source: { color: "#4678c8", height: 0.04 },
  sink: { color: "#c8803c", height: 0.6 },
  chute: { color: "#5b6470", height: 0.55 },
  items: {},
};

// The panel's fields: [group, key, label, type, extra].
const FIELDS = [
  ["scene", "background", "Background", "color"],
  ["scene", "floor", "Floor", "color"],
  ["scene", "grid", "Grid", "color"],
  ["scene", "gridMajor", "Grid, every metre", "color"],
  ["scene", "light", "Sun", "number", { min: 0, max: 5, step: 0.1 }],
  ["scene", "ambient", "Ambient light", "number", { min: 0, max: 5, step: 0.1 }],
  ["scene", "labelSize", "Label size", "number", { min: 0.3, max: 4, step: 0.1 }],
  ["belt", "surface", "Belt surface", "color"],
  ["belt", "frame", "Belt frame (empty: its layout colour)", "color-or-layout"],
  ["belt", "thickness", "Belt thickness (m)", "number", { min: 0.01, max: 1, step: 0.01 }],
  ["belt", "stands", "Stands", "checkbox"],
  ["belt", "standColor", "Stand colour", "color"],
  ["belt", "chevrons", "Moving chevrons", "checkbox"],
  ["belt", "chevronColor", "Chevron colour", "color"],
  ["roller", "color", "Roller frame (empty: its layout colour)", "color-or-layout"],
  ["parcel", "colorMode", "Parcel colour", "select", { options: ["layout", "destination", "fixed"] }],
  ["parcel", "color", "Fixed parcel colour", "color"],
  ["parcel", "heightScale", "Parcel height x", "number", { min: 0.1, max: 5, step: 0.1 }],
  ["sensor", "off", "Sensor clear", "color"],
  ["sensor", "on", "Sensor blocked", "color"],
  ["sensor", "postHeight", "Sensor post height (m)", "number", { min: 0.05, max: 2, step: 0.05 }],
  ["sensor", "bannerHead", "Laser banner head above the belt (m)", "number", { min: 0.1, max: 3, step: 0.05 }],
  ["source", "color", "Source zone", "color"],
  ["source", "height", "Source zone height (m)", "number", { min: 0.01, max: 3, step: 0.01 }],
  ["sink", "color", "Sink zone", "color"],
  ["sink", "height", "Sink zone height (m)", "number", { min: 0.01, max: 3, step: 0.05 }],
  ["chute", "color", "Chute bin", "color"],
  ["chute", "height", "Chute wall height (m)", "number", { min: 0.05, max: 3, step: 0.05 }],
];

export let S = clone(DEFAULTS);
let storage = "server";            // "server" (next to the layout) or "browser" (no layout file)
let savedPath = "";

function clone(v) { return JSON.parse(JSON.stringify(v)); }

function merge(base, over) {
  for (const [k, v] of Object.entries(over || {})) {
    if (v && typeof v === "object" && !Array.isArray(v) && base[k] && typeof base[k] === "object") merge(base[k], v);
    else if (k in base || base === S.items || base.__free) base[k] = v;
  }
  return base;
}

export function hex(color) {               // "#rrggbb" -> 0xrrggbb
  return parseInt(String(color).replace("#", ""), 16);
}

export function item(id) {
  return S.items[id] || {};
}

export const DESTINATION_COLORS = ["#d9a441", "#3898d6", "#40b268", "#c8505a", "#9b6cd6", "#d6803a", "#3ab5b0",
                                   "#c9c04a"];

export function parcelColor(color, destination) {
  if (S.parcel.colorMode === "fixed") return S.parcel.color;
  if (S.parcel.colorMode === "destination") {
    return destination ? DESTINATION_COLORS[(destination - 1) % DESTINATION_COLORS.length] : "#7d8590";
  }
  return color;
}

export async function load() {
  S = clone(DEFAULTS);
  try {
    const res = await (await fetch("style", { cache: "no-store" })).json();
    storage = res.path ? "server" : "browser";
    savedPath = res.path || "";
    let saved = res.style;
    if (!res.path) {
      try { saved = JSON.parse(localStorage.getItem("view3d.style") || "null"); } catch { saved = null; }
    }
    merge(S, saved || {});
  } catch {
    storage = "browser";
  }
  return S;
}

export async function save() {
  if (storage === "browser") {
    try { localStorage.setItem("view3d.style", JSON.stringify(S)); } catch { /* private window */ }
    return "this browser";
  }
  const res = await fetch("style", { method: "POST", headers: { "Content-Type": "application/json" },
                                     body: JSON.stringify(S) });
  if (!res.ok) throw new Error(`the simulator answered ${res.status}`);
  return savedPath;
}

// -- the panel --------------------------------------------------------------------------------

let panel = null;
let changed = () => {};
let selected = null;           // {id, kind, label}

function field(group, key, label, type, extra = {}) {
  const row = document.createElement("label");
  row.className = "srow";
  const text = document.createElement("span");
  text.textContent = label;
  row.appendChild(text);
  const obj = group === "items" ? S.items[selected.id] : S[group];
  let input;
  if (type === "select") {
    input = document.createElement("select");
    for (const o of extra.options) input.add(new Option(o, o));
    input.value = obj[key];
  } else if (type === "checkbox") {
    input = document.createElement("input");
    input.type = "checkbox";
    input.checked = !!obj[key];
  } else if (type === "color" || type === "color-or-layout") {
    const wrap = document.createElement("span");
    input = document.createElement("input");
    input.type = "color";
    input.value = obj[key] || "#808080";
    wrap.appendChild(input);
    if (type === "color-or-layout") {
      const reset = document.createElement("button");
      reset.textContent = obj[key] ? "layout" : "(layout)";
      reset.title = "Use each component's colour from the layout";
      reset.onclick = (e) => { e.preventDefault(); obj[key] = ""; changed(); render(); };
      wrap.appendChild(reset);
    }
    row.appendChild(wrap);
    input.oninput = () => { obj[key] = input.value; changed(); };
    return row;
  } else {
    input = document.createElement("input");
    input.type = "number";
    Object.assign(input, extra);
    input.value = obj[key] ?? "";
  }
  input.onchange = input.oninput = () => {
    if (type === "checkbox") obj[key] = input.checked;
    else if (type === "number") {
      if (input.value === "" && group === "items") delete obj[key];
      else if (Number.isFinite(parseFloat(input.value))) obj[key] = parseFloat(input.value);
    } else obj[key] = input.value;
    changed();
  };
  row.appendChild(input);
  return row;
}

function heading(text) {
  const h = document.createElement("div");
  h.className = "shead";
  h.textContent = text;
  return h;
}

function render() {
  panel.innerHTML = "";
  const top = document.createElement("div");
  top.className = "stop";
  const title = document.createElement("b");
  title.textContent = "3D style";
  const close = document.createElement("button");
  close.textContent = "x";
  close.onclick = () => { panel.style.display = "none"; };
  top.append(title, close);
  panel.appendChild(top);
  if (selected) {
    panel.appendChild(heading(`Selected: ${selected.label}`));
    S.items[selected.id] = S.items[selected.id] || {};
    panel.appendChild(field("items", "color", "Colour", "color"));
    if (["belt", "source", "sink", "chute"].includes(selected.kind)) {
      panel.appendChild(field("items", selected.kind === "belt" ? "thickness" : "height",
                              selected.kind === "belt" ? "Thickness (m, empty: as all)" : "Height (m, empty: as all)",
                              "number", { min: 0.01, max: 3, step: 0.01 }));
    }
    panel.appendChild(field("items", "hidden", "Hidden", "checkbox"));
    const clear = document.createElement("button");
    clear.textContent = `Back to the defaults for ${selected.id}`;
    clear.onclick = () => { delete S.items[selected.id]; changed(); render(); };
    panel.appendChild(clear);
  } else {
    const hint = document.createElement("div");
    hint.className = "shint";
    hint.textContent = "Double-click (or Alt+click) a component to style it alone.";
    panel.appendChild(hint);
  }
  let group = null;
  for (const [g, key, label, type, extra] of FIELDS) {
    if (g !== group) {
      group = g;
      panel.appendChild(heading(g[0].toUpperCase() + g.slice(1)));
    }
    panel.appendChild(field(g, key, label, type, extra));
  }
  const bar = document.createElement("div");
  bar.className = "sbar";
  const saveBtn = document.createElement("button");
  saveBtn.textContent = "Save";
  const status = document.createElement("span");
  status.className = "shint";
  saveBtn.onclick = async () => {
    try { status.textContent = `saved: ${await save()}`; } catch (err) { status.textContent = `not saved: ${err}`; }
  };
  const resetBtn = document.createElement("button");
  resetBtn.textContent = "Defaults";
  resetBtn.title = "Every value back to its default (Save to keep it)";
  resetBtn.onclick = () => { S = clone(DEFAULTS); selected = null; changed(); render(); };
  bar.append(saveBtn, resetBtn, status);
  panel.appendChild(bar);
  const where = document.createElement("div");
  where.className = "shint";
  where.textContent = storage === "server" ? `Saved next to the layout: ${savedPath}` : "Saved in this browser";
  panel.appendChild(where);
}

export function openPanel(onChange) {
  changed = onChange;
  if (!panel) {
    panel = document.getElementById("style");
  }
  panel.style.display = panel.style.display === "block" ? "none" : "block";
  if (panel.style.display === "block") render();
}

export function select(target, onChange) {
  selected = target;
  changed = onChange || changed;
  panel = panel || document.getElementById("style");
  panel.style.display = "block";
  render();
}
