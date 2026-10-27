// app.js - Channel Registry editor + live metrics dashboard.
//
// No build step - this runs on a local Flask server that may sit on a
// plant-floor network with no internet access, so everything here is
// either stdlib-only Python or vanilla JS/CSS, plus one vendored library:
// Chart.js (web_static/vendor/chart.umd.js, loaded by index.html before
// this file - see vendor/README.md), used for the Live Data chart at the
// bottom (see updateChart()).
//
// Every validation message shown here comes straight from POST /api/validate,
// which calls the exact same Python functions (verify_component_list,
// parse_mapping_rows, channel_registry.validate_entries) the CLI tools and
// MQTTSyncEngine itself use - this file never decides what's valid, only
// displays what the server already decided.

const state = {
  registryPath: "",
  channels: [],       // [{name, kind, direction, topic, interval_ms, frame_type, mapping_csv, rows: [{byte_type, path}]}]
  selected: -1,
  catalog: { components: [] },
  results: {},         // name -> {ok, errors, payload_size}
  validateTimer: null,
  dirty: false,
};

function markDirty() {
  state.dirty = true;
  document.getElementById("dirty-badge").classList.remove("hidden");
}

function clearDirty() {
  state.dirty = false;
  document.getElementById("dirty-badge").classList.add("hidden");
}

window.addEventListener("beforeunload", (e) => {
  if (!state.dirty) return;
  e.preventDefault();
  e.returnValue = "";
});

function showToast(message, kind) {
  const toast = document.getElementById("toast");
  toast.textContent = message;
  toast.className = "toast " + (kind || "");
  clearTimeout(showToast._timer);
  showToast._timer = setTimeout(() => toast.classList.add("hidden"), 3500);
}

// =========================================================== tabs

document.querySelectorAll(".tab-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab-btn").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".tab-panel").forEach((p) => p.classList.remove("active"));
    btn.classList.add("active");
    document.getElementById("tab-" + btn.dataset.tab).classList.add("active");
    if (btn.dataset.tab === "live") startLiveTab();
  });
});

// =========================================================== bootstrap

async function init() {
  const [config, registry] = await Promise.all([
    fetchJson("/api/config"),
    fetchJson("/api/registry"),
  ]);
  if (config && !config.error) state.catalog = config;
  if (registry && !registry.error) {
    state.registryPath = registry.path;
    state.channels = registry.channels.map(normalizeChannel);
    document.getElementById("registry-path").textContent = registry.path;
  }
  renderChannelList();
  if (state.channels.length) selectChannel(0);
  scheduleValidate();
}

function normalizeChannel(c) {
  return {
    name: c.name || "",
    kind: c.kind || "io_mapping",
    direction: c.direction || "in",
    topic: c.topic || "",
    interval_ms: c.interval_ms || 100,
    frame_type: c.frame_type || "",
    mapping_csv: c.mapping_csv || "",
    // null = "authored outside this UI, leave its CSV alone until the
    // picker is actually used"; an array (even empty) = "this channel's
    // mapping is entirely generated from these ids" (see validate_entries
    // in channel_registry.py - component_ids and hand-authored rows are
    // mutually exclusive on the wire, and the server decides ByteType +
    // padding for every id in this list, never this file).
    component_ids: (c.component_ids !== undefined && c.component_ids !== null) ? c.component_ids.slice() : null,
    rows: (c.rows || []).map((r) => ({ byte_type: r.byte_type, path: r.path })),
  };
}

async function fetchJson(url, opts) {
  try {
    const res = await fetch(url, opts);
    const body = await res.json();
    if (!res.ok) body.__status = res.status;
    return body;
  } catch (e) {
    return { error: String(e) };
  }
}

// =========================================================== channel list

function renderChannelList() {
  const ul = document.getElementById("channel-list");
  ul.innerHTML = "";
  state.channels.forEach((ch, i) => {
    const li = document.createElement("li");
    const result = state.results[ch.name];
    li.className = (i === state.selected ? "selected " : "") + (result && !result.ok ? "invalid" : "");
    const sizeBadge = result && result.ok ? `<span class="size-badge">${result.payload_size}B</span>` : "";
    li.innerHTML = `<span class="dot"></span><span class="name">${escapeHtml(ch.name || "(unnamed)")}</span>${sizeBadge}`;
    li.addEventListener("click", () => selectChannel(i));
    ul.appendChild(li);
  });
}

document.getElementById("new-channel-btn").addEventListener("click", () => {
  // A brand-new channel has no legacy CSV to protect, so it starts ready
  // for the component picker (an empty list, not null).
  state.channels.push(normalizeChannel({ name: `channel_${state.channels.length + 1}`, component_ids: [] }));
  markDirty();
  renderChannelList();
  selectChannel(state.channels.length - 1);
});

document.getElementById("delete-btn").addEventListener("click", () => {
  if (state.selected < 0) return;
  const ch = state.channels[state.selected];
  if (!confirm(`Delete channel "${ch.name || "(unnamed)"}"? This only takes effect once you Save.`)) return;
  state.channels.splice(state.selected, 1);
  state.selected = -1;
  markDirty();
  renderChannelList();
  showEditor(false);
  scheduleValidate();
});

function selectChannel(i) {
  state.selected = i;
  renderChannelList();
  const ch = state.channels[i];
  showEditor(true);
  document.getElementById("f-name").value = ch.name;
  document.getElementById("f-kind").value = ch.kind;
  document.getElementById("f-direction").value = ch.direction;
  document.getElementById("f-topic").value = ch.topic;
  document.getElementById("f-frame-type").value = ch.frame_type;
  document.getElementById("f-interval").value = ch.interval_ms;
  document.getElementById("f-csv").value = ch.mapping_csv;
  renderForKind();
  renderChannelStatus();
}

function showEditor(show) {
  document.getElementById("editor-empty").classList.toggle("hidden", show);
  document.getElementById("editor-body").classList.toggle("hidden", !show);
}

function currentChannel() {
  return state.selected >= 0 ? state.channels[state.selected] : null;
}

// Which editing UI applies depends on kind: io_mapping channels pick
// components and the server generates everything else; metric channels
// have no config-backed component to pick from, so they're hand-authored.
function isComponentDriven(ch) {
  return ch.kind === "io_mapping" && ch.component_ids !== null;
}

function renderForKind() {
  const ch = currentChannel();
  if (!ch) return;
  const usesPicker = ch.kind === "io_mapping";
  document.getElementById("component-picker").classList.toggle("hidden", !usesPicker);
  document.getElementById("metric-row-add").classList.toggle("hidden", usesPicker);
  document.getElementById("metric-hint").classList.toggle("hidden", usesPicker);
  if (usesPicker) {
    populateComponentPicker();
    renderComponentChips();
  }
  updateRowsTableHeader(usesPicker);
  renderRows();
}

function updateRowsTableHeader(usesPicker) {
  const thead = document.getElementById("rows-thead");
  thead.innerHTML = usesPicker
    ? `<tr>
        <th style="width:36px">No.</th>
        <th style="width:90px">Address</th>
        <th style="width:110px">Data Type</th>
        <th>Symbol</th>
        <th style="width:40px"></th>
      </tr>`
    : `<tr>
        <th style="width:36px">No.</th>
        <th style="width:90px">Address</th>
        <th style="width:130px">Type</th>
        <th style="width:35%">Group</th>
        <th>Name</th>
        <th style="width:40px"></th>
      </tr>`;
}

// Every field input writes straight back into state and re-validates -
// this is the "real-time" part: no explicit "check" step, feedback follows
// typing (debounced) automatically.
["f-name", "f-kind", "f-direction", "f-topic", "f-frame-type", "f-interval", "f-csv"].forEach((id) => {
  document.getElementById(id).addEventListener("input", onFieldChange);
  document.getElementById(id).addEventListener("change", onFieldChange);
});

function onFieldChange() {
  const ch = currentChannel();
  if (!ch) return;
  const kindChanged = ch.kind !== document.getElementById("f-kind").value;
  ch.name = document.getElementById("f-name").value.trim();
  ch.kind = document.getElementById("f-kind").value;
  ch.direction = document.getElementById("f-direction").value;
  ch.topic = document.getElementById("f-topic").value.trim();
  ch.frame_type = document.getElementById("f-frame-type").value.trim();
  ch.interval_ms = parseInt(document.getElementById("f-interval").value, 10) || 100;
  ch.mapping_csv = document.getElementById("f-csv").value.trim();
  markDirty();
  renderChannelList();
  if (kindChanged) renderForKind();
  scheduleValidate();
}

// =========================================================== rows table
//
// Address column: Siemens/TIA-Portal-style %I/%Q byte.bit notation, purely
// a display computed from the row order + widths (create_pack_format_string
// in mqtt_message_binding.py packs bits the same way) - it needs no round
// trip to the server, so it updates the instant a row changes.

const BYTE_TYPE_WIDTH = { BYTE: 1, UINT16: 2, UINT32: 4, UINT64: 8, FLOAT: 4 };
const BYTE_TYPE_LETTER = { BYTE: "B", UINT16: "W", UINT32: "D", UINT64: "X", FLOAT: "D" };

function computeAddresses(rows, direction) {
  const prefix = direction === "out" ? "Q" : "I";
  let byteOffset = 0;
  let bitIndex = 0;
  return rows.map((row) => {
    const type = (row.byte_type || "").toUpperCase();
    if (type === "BIT") {
      const addr = `%${prefix}${byteOffset}.${bitIndex}`;
      bitIndex += 1;
      if (bitIndex === 8) { bitIndex = 0; byteOffset += 1; }
      return addr;
    }
    if (bitIndex > 0) { byteOffset += 1; bitIndex = 0; } // a non-bit field flushes the partial byte
    const width = BYTE_TYPE_WIDTH[type];
    if (!width) return "-";
    const addr = width === 1 ? `%${prefix}B${byteOffset}` : `%${prefix}${BYTE_TYPE_LETTER[type]}${byteOffset}`;
    byteOffset += width;
    return addr;
  });
}

function refreshAddressColumn() {
  const ch = currentChannel();
  if (!ch) return;
  const addresses = computeAddresses(ch.rows, ch.direction);
  document.querySelectorAll("#rows-tbody tr").forEach((tr, i) => {
    const cell = tr.querySelector(".col-address");
    if (cell) cell.textContent = addresses[i];
  });
}

// PLC-style register widths a metric field can pack as - value is the real
// ByteType name (what's actually written to the CSV/wire); label is the
// friendlier size term an engineer coming from ladder-logic tooling
// already knows. PAD stays available (see parse_mapping_rows) since a
// hand-authored metric row still sometimes needs manual byte alignment.
const METRIC_TYPE_OPTIONS = [
  { value: "BIT", label: "BIT" },
  { value: "BYTE", label: "BYTE" },
  { value: "UINT16", label: "WORD (UINT16)" },
  { value: "UINT32", label: "DWORD (UINT32)" },
  { value: "UINT64", label: "QWORD (UINT64)" },
  { value: "FLOAT", label: "FLOAT" },
  { value: "PAD", label: "PAD (auto)" },
];

// A metric field's path is "group/name" (MetricsMessage's own parse_path
// convention) - group is exactly what the Live Data tab's tree (and the
// Qt panel it replaces) branches on, so splitting it into its own column
// makes that grouping something you set, not something you have to
// remember to type as "group/" every time.
function splitGroupName(path) {
  const idx = (path || "").indexOf("/");
  if (idx === -1) return { group: "", name: path || "" };
  return { group: path.slice(0, idx), name: path.slice(idx + 1) };
}

function renderRows() {
  const ch = currentChannel();
  const tbody = document.getElementById("rows-tbody");
  tbody.innerHTML = "";

  if (!ch) return;
  const addresses = computeAddresses(ch.rows, ch.direction);

  if (isComponentDriven(ch)) {
    // Fully server-computed - display only, nothing here is editable.
    ch.rows.forEach((row, i) => {
      const tr = document.createElement("tr");
      if (row.path && row.path.startsWith("padding_")) tr.className = "row-padding";
      tr.innerHTML = `
        <td class="row-num">${i + 1}</td>
        <td class="row-readonly col-address">${addresses[i]}</td>
        <td class="row-readonly">${escapeHtml(row.byte_type)}</td>
        <td class="row-readonly">${escapeHtml(row.path)}</td>
        <td></td>
      `;
      tbody.appendChild(tr);
    });
    return;
  }

  // Metric (or a legacy hand-authored io_mapping channel): free-form,
  // editable rows - there's no config-backed component to derive these
  // from, so type/group/name are entered directly.
  ensureMetricGroupDatalist(ch);
  const errorRows = errorRowIndexesFor(ch.name);
  ch.rows.forEach((row, i) => {
    const { group, name } = splitGroupName(row.path);
    const tr = document.createElement("tr");
    if (errorRows.has(i)) tr.className = "row-error";
    const typeOptions = METRIC_TYPE_OPTIONS.map(
      (opt) => `<option value="${opt.value}" ${opt.value === (row.byte_type || "").toUpperCase() ? "selected" : ""}>${opt.label}</option>`
    ).join("");
    // A byte_type that isn't one of the curated options (e.g. PADDING, or
    // a lowercase/legacy value from a hand-edited CSV) still needs a spot
    // in the dropdown, or picking it here would silently change the row.
    const knownValues = METRIC_TYPE_OPTIONS.map((o) => o.value);
    const currentUpper = (row.byte_type || "").toUpperCase();
    const extraOption = row.byte_type && !knownValues.includes(currentUpper)
      ? `<option value="${escapeHtml(row.byte_type)}" selected>${escapeHtml(row.byte_type)}</option>` : "";

    tr.innerHTML = `
      <td class="row-num">${i + 1}</td>
      <td class="col-address">${addresses[i]}</td>
      <td><select class="row-type">${extraOption}${typeOptions}</select></td>
      <td><input class="row-group" value="${escapeHtml(group)}" list="metric-group-options" placeholder="e.g. perf"></td>
      <td><input class="row-name" value="${escapeHtml(name)}" placeholder="e.g. scan_time_ms"></td>
      <td><button class="row-remove-btn" title="Remove row">&times;</button></td>
    `;
    tr.querySelector(".row-type").addEventListener("change", (e) => {
      row.byte_type = e.target.value;
      markDirty();
      const newType = e.target.value.toUpperCase();
      if (newType !== "BIT" && newType !== "PAD" && computeBitPending(ch.rows.slice(0, i)) !== 0) {
        // This row just became a non-BIT field directly after an unclosed
        // run of bits before it - insert the padding it needs, the same
        // auto-pad "+ Add row" already does for a brand new row.
        ch.rows.splice(i, 0, { byte_type: "PAD", path: "auto" });
        renderRows(); // row count changed - a full rebuild, not just the address column
      } else {
        refreshAddressColumn(); // recompute addresses without rebuilding inputs (would drop focus mid-keystroke)
      }
      scheduleValidate();
    });
    tr.querySelector(".row-group").addEventListener("input", (e) => {
      row.path = `${e.target.value.trim()}/${splitGroupName(row.path).name}`;
      markDirty();
      scheduleValidate();
    });
    tr.querySelector(".row-name").addEventListener("input", (e) => {
      row.path = `${splitGroupName(row.path).group}/${e.target.value.trim()}`;
      markDirty();
      scheduleValidate();
    });
    tr.querySelector(".row-remove-btn").addEventListener("click", () => {
      ch.rows.splice(i, 1);
      markDirty();
      renderRows();
      scheduleValidate();
    });
    tbody.appendChild(tr);
  });
}

// Mirrors parse_mapping_rows()'s own bit-packing rule (mqtt_message_
// binding.py): a run of BIT rows shares one byte, and any non-BIT row (or
// an explicit PAD) always starts the next one fresh. Used by the row-type
// change handler to auto-pad the moment a row actually stops continuing a
// bit run - the same way component-driven io_mapping channels never
// require a person to think about padding by hand. "+PAD row" stays
// available for the one thing auto-padding can't infer: forcing two
// back-to-back BIT rows that are semantically different groups onto
// separate bytes.
function computeBitPending(rows) {
  let pending = 0;
  for (const row of rows) {
    const type = (row.byte_type || "").toUpperCase();
    if (type === "BIT") { pending = (pending + 1) % 8; continue; }
    pending = 0; // PAD and every other type both close out the current byte
  }
  return pending;
}

function ensureMetricGroupDatalist(ch) {
  let dl = document.getElementById("metric-group-options");
  if (!dl) {
    dl = document.createElement("datalist");
    dl.id = "metric-group-options";
    document.body.appendChild(dl);
  }
  const groups = [...new Set(ch.rows.map((r) => splitGroupName(r.path).group).filter(Boolean))];
  dl.innerHTML = groups.map((g) => `<option value="${escapeHtml(g)}">`).join("");
}

document.getElementById("add-row-btn").addEventListener("click", () => {
  const ch = currentChannel();
  if (!ch) return;
  // Continue an in-progress run of BIT rows rather than eagerly padding
  // and closing it out - padding on every click would make it impossible
  // to add several consecutive bits (the very next click would insert
  // padding before you'd typed a second one). If the last row is BIT,
  // the new one defaults to BIT too; changing ITS type away from BIT
  // afterward is what actually triggers auto-pad (see the row-type
  // change handler above) - that's the one place the decision "this bit
  // run is done" is actually made.
  const lastType = ch.rows.length ? (ch.rows[ch.rows.length - 1].byte_type || "").toUpperCase() : "";
  const defaultType = lastType === "BIT" ? "BIT" : "FLOAT";
  ch.rows.push({ byte_type: defaultType, path: "" });
  markDirty();
  renderRows();
  scheduleValidate();
});

document.getElementById("qa-pad-btn").addEventListener("click", () => {
  const ch = currentChannel();
  if (!ch) return;
  ch.rows.push({ byte_type: "PAD", path: "auto" });
  markDirty();
  renderRows();
  scheduleValidate();
});

// -------- component picker (io_mapping only) - add an id, the server
// decides its ByteType and any padding via each component's own
// packFormat (config_parser.py) + custom_component_mapping().

function populateComponentPicker() {
  const ch = currentChannel();
  const sel = document.getElementById("qa-component");
  const already = new Set(ch.component_ids || []);
  sel.innerHTML = state.catalog.components
    .filter((c) => !already.has(c.id))
    .map((c) => `<option value="${escapeHtml(c.id)}">${escapeHtml(c.id)} (${c.type}${c.pack_format ? ", " + c.pack_format : ""})</option>`)
    .join("");
}

function renderComponentChips() {
  const ch = currentChannel();
  const list = document.getElementById("component-chip-list");
  const ids = ch.component_ids || [];
  list.innerHTML = ids.map((id, i) => {
    const comp = state.catalog.components.find((c) => c.id === id);
    const packTag = comp && comp.pack_format ? `<span class="pack-tag">${comp.pack_format}</span>` : "";
    return `
      <li class="chip" data-index="${i}">
        <button class="reorder-btn" data-dir="-1" ${i === 0 ? "disabled" : ""} title="Move earlier">&uarr;</button>
        <button class="reorder-btn" data-dir="1" ${i === ids.length - 1 ? "disabled" : ""} title="Move later">&darr;</button>
        <span>${escapeHtml(id)}</span>${packTag}
        <button class="chip-remove-btn" title="Remove">&times;</button>
      </li>`;
  }).join("");

  list.querySelectorAll(".chip").forEach((li) => {
    const i = parseInt(li.dataset.index, 10);
    li.querySelector(".chip-remove-btn").addEventListener("click", () => {
      ch.component_ids.splice(i, 1);
      markDirty();
      populateComponentPicker();
      renderComponentChips();
      scheduleValidate();
    });
    li.querySelectorAll(".reorder-btn").forEach((btn) => {
      btn.addEventListener("click", () => {
        const j = i + parseInt(btn.dataset.dir, 10);
        if (j < 0 || j >= ch.component_ids.length) return;
        [ch.component_ids[i], ch.component_ids[j]] = [ch.component_ids[j], ch.component_ids[i]];
        markDirty();
        renderComponentChips();
        scheduleValidate();
      });
    });
  });
}

document.getElementById("qa-add-btn").addEventListener("click", () => {
  const ch = currentChannel();
  const id = document.getElementById("qa-component").value;
  if (!ch || !id) return;
  if (ch.component_ids === null) ch.component_ids = []; // first use converts a legacy channel
  ch.component_ids.push(id);
  markDirty();
  populateComponentPicker();
  renderComponentChips();
  scheduleValidate();
});

// =========================================================== validation

function scheduleValidate() {
  clearTimeout(state.validateTimer);
  state.validateTimer = setTimeout(runValidate, 300); // debounce - not every keystroke
}

async function runValidate() {
  const statusEl = document.getElementById("channel-status");
  if (currentChannel() && statusEl) statusEl.innerHTML = '<span class="validating">Checking...</span>';

  const body = { channels: state.channels.map((c) => ({ ...c })) };
  const res = await fetchJson("/api/validate", {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
  });
  if (res.error) return;
  state.results = {};
  res.results.forEach((r) => { state.results[r.name] = r; });

  // Component-driven channels never author rows by hand - every validate
  // call replaces the preview with whatever the server just computed from
  // component_ids, so the table always shows the real, current layout
  // (including padding). Hand-authored (metric) rows are never touched
  // here - overwriting mid-edit would clobber a "PAD" row's shorthand
  // with its expanded form while someone is still typing.
  state.channels.forEach((ch) => {
    const result = state.results[ch.name];
    if (isComponentDriven(ch) && result && result.rows) ch.rows = result.rows;
  });

  renderChannelList();
  renderChannelStatus();
  refreshRowsSoft(); // re-highlight rows with the fresh error set
}

// Called on every debounced validate tick (every keystroke, effectively) -
// a full renderRows() would tear down and rebuild every <input>, which
// drops focus and the cursor position out from under whatever field
// someone is mid-typing into. Component-driven rows are always safe to
// fully rebuild (nothing in them is ever focused/typed into); hand-
// authored (metric) rows only get their address column and error
// highlighting refreshed in place, never their inputs.
function refreshRowsSoft() {
  const ch = currentChannel();
  if (!ch) return;
  if (isComponentDriven(ch)) {
    renderRows();
    return;
  }
  refreshAddressColumn();
  const errorRows = errorRowIndexesFor(ch.name);
  document.querySelectorAll("#rows-tbody tr").forEach((tr, i) => {
    tr.classList.toggle("row-error", errorRows.has(i));
  });
}

function errorRowIndexesFor(name) {
  const indexes = new Set();
  const result = name ? state.results[name] : null;
  if (!result) return indexes;
  for (const err of result.errors) {
    const m = /^.*?:(\d+): /.exec(err);
    if (m) indexes.add(parseInt(m[1], 10) - 1);
  }
  return indexes;
}

const UNKNOWN_FRAME_TYPE_RE = /^unknown frame_type '([^']+)'/;

function renderChannelStatus() {
  const el = document.getElementById("channel-status");
  el.innerHTML = "";
  const ch = currentChannel();
  if (!ch) return;
  const result = state.results[ch.name];
  if (!result) return;

  if (result.ok) {
    el.innerHTML = `<span class="ok">Valid - ${result.payload_size} payload bytes</span>`;
    return;
  }

  result.errors.forEach((err) => {
    const line = document.createElement("span");
    line.className = "err-line";
    const match = UNKNOWN_FRAME_TYPE_RE.exec(err);
    if (match) {
      // The error text itself says to run generate_frame_type.py - offer
      // to do exactly that from here instead, via the same function.
      line.textContent = `unknown frame_type '${match[1]}' `;
      const btn = document.createElement("button");
      btn.className = "btn-small";
      btn.textContent = "Generate FrameType";
      btn.addEventListener("click", () => generateFrameType(match[1], btn));
      line.appendChild(btn);
    } else {
      line.textContent = err;
    }
    el.appendChild(line);
  });
}

async function generateFrameType(name, btn) {
  btn.disabled = true;
  const res = await fetchJson("/api/frame-types/generate", {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ names: [name] }),
  });
  btn.disabled = false;
  if (res.error) {
    showToast(res.error, "err");
    return;
  }
  showToast(res.message, Object.keys(res.added).length ? "ok" : "");
  // Re-validating won't turn this green: the server that answers /api/validate
  // still has the pre-restart FrameType in memory. scheduleValidate() is
  // called anyway so anything ELSE that changed while this was in flight
  // still gets checked - the toast is what actually tells the person what
  // to do next (restart the server).
  scheduleValidate();
}

// =========================================================== save

document.getElementById("save-btn").addEventListener("click", async () => {
  const btn = document.getElementById("save-btn");
  btn.disabled = true;
  try {
    const res = await fetchJson("/api/registry", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ channels: state.channels }),
    });
    if (res.error) {
      showToast(res.error, "err");
      return;
    }
    res.results.forEach((r) => { state.results[r.name] = r; });
    state.channels.forEach((ch) => {
      const result = state.results[ch.name];
      if (isComponentDriven(ch) && result && result.rows) ch.rows = result.rows;
    });
    renderChannelList();
    renderChannelStatus();
    renderRows();
    if (res.saved) {
      clearDirty();
      showToast("Saved.", "ok");
    } else {
      showToast("Not saved - fix the errors shown for each channel first.", "err");
    }
  } finally {
    btn.disabled = false;
  }
});

// =========================================================== live data tab

const live = {
  started: false,
  connected: null,
  selected: new Set(),   // "group|name"
  buffers: {},            // "group|name" -> [{ts, value}]
  lastTs: {},              // "group|name" -> last ts fetched
  filterText: "",
  // From/To/"Follow latest" - the same pattern data_visualization_panel.py's
  // Qt plot tab used: following keeps the window's WIDTH fixed and slides
  // it forward as new data arrives; typing into either field (like
  // scrubbing a video timeline) freezes the view at exactly that range
  // until "Follow latest" is re-checked. range.from/to are epoch seconds,
  // null until the first real data point sets them.
  range: { from: null, to: null, follow: true },
  treeTimer: null,
  tableTimer: null,
  seriesTimer: null,
};

const RANGE_WINDOW_DEFAULT_S = 60;

function startLiveTab() {
  if (live.started) return;
  live.started = true;
  pollTree();
  pollLatestTable();
  live.treeTimer = setInterval(pollTree, 2000);
  live.tableTimer = setInterval(pollLatestTable, 1000);
  live.seriesTimer = setInterval(pollSeries, 500);
  document.getElementById("metrics-filter").addEventListener("input", (e) => {
    live.filterText = e.target.value.trim().toLowerCase();
    applyMetricsFilter();
  });

  const fromInput = document.getElementById("range-from");
  const toInput = document.getElementById("range-to");
  const followInput = document.getElementById("range-follow");
  const onManualRangeEdit = () => {
    // A manual edit takes over from auto-following, same as editing the
    // Qt panel's From/To fields does - and since a value earlier than
    // anything currently buffered needs data the incremental since=...
    // polling never fetched, force one full backfill for every checked
    // series (bounded by whatever the store itself still retains).
    followInput.checked = false;
    live.range.follow = false;
    live.range.from = fromInput.value ? Date.parse(fromInput.value) / 1000 : live.range.from;
    live.range.to = toInput.value ? Date.parse(toInput.value) / 1000 : live.range.to;
    for (const key of live.selected) { live.lastTs[key] = 0; live.buffers[key] = []; }
  };
  fromInput.addEventListener("change", onManualRangeEdit);
  toInput.addEventListener("change", onManualRangeEdit);
  followInput.addEventListener("change", (e) => { live.range.follow = e.target.checked; });
}

// datetime-local inputs display and parse in the browser's LOCAL time zone,
// with no zone marker of their own - Date's own toISOString() is UTC, so
// using it here would silently show a UTC-labeled clock value as if it
// were local (e.g. a 7-hour-off reading on a UTC-7 machine). This formats
// from the Date object's local getters instead, matching what
// onManualRangeEdit's Date.parse(...) (which also assumes local for a
// zone-less string) reads back - the two stay inverses of each other.
function formatLocalDatetimeInput(ts) {
  const d = new Date(ts * 1000);
  const pad = (n) => String(n).padStart(2, "0");
  return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}T${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}

function setRangeInputs(fromTs, toTs) {
  // Pushed in programmatically while following - setting .value directly
  // (not simulating typing) never fires the inputs' own "change" listener,
  // so this can't loop back into onManualRangeEdit().
  document.getElementById("range-from").value = formatLocalDatetimeInput(fromTs);
  document.getElementById("range-to").value = formatLocalDatetimeInput(toTs);
}

async function pollTree() {
  const tree = await fetchJson("/api/metrics/tree");
  if (tree.error) {
    setLiveConnected(false);
    return;
  }
  setLiveConnected(true);
  const el = document.getElementById("metrics-tree");
  el.innerHTML = "";
  Object.keys(tree).sort().forEach((group) => {
    const groupDiv = document.createElement("div");
    groupDiv.className = "tree-group";
    groupDiv.dataset.group = group.toLowerCase();
    groupDiv.innerHTML = `<div class="tree-group-name">${escapeHtml(group)}</div>`;
    Object.keys(tree[group]).sort().forEach((name) => {
      const key = group + "|" + name;
      const kind = tree[group][name];
      const row = document.createElement("label");
      row.className = "tree-metric";
      row.dataset.text = `${group} ${name}`.toLowerCase();
      row.innerHTML = `<input type="checkbox" ${live.selected.has(key) ? "checked" : ""}>
        <span>${escapeHtml(name)}</span><span class="kind-tag">${kind}</span>`;
      row.querySelector("input").addEventListener("change", (e) => {
        if (e.target.checked) { live.selected.add(key); live.lastTs[key] = 0; live.buffers[key] = []; }
        else { live.selected.delete(key); delete live.buffers[key]; }
      });
      groupDiv.appendChild(row);
    });
    el.appendChild(groupDiv);
  });
  applyMetricsFilter();
}

// Purely a display filter over the tree already fetched - it never
// touches live.selected, so hiding a metric doesn't un-plot it.
function applyMetricsFilter() {
  const needle = live.filterText;
  document.querySelectorAll("#metrics-tree .tree-metric").forEach((row) => {
    row.classList.toggle("filtered-out", !!needle && !row.dataset.text.includes(needle));
  });
  document.querySelectorAll("#metrics-tree .tree-group").forEach((groupDiv) => {
    const anyVisible = groupDiv.querySelector(".tree-metric:not(.filtered-out)");
    groupDiv.classList.toggle("filtered-out", !!needle && !anyVisible && !groupDiv.dataset.group.includes(needle));
  });
}

async function pollLatestTable() {
  const rows = await fetchJson("/api/metrics/latest");
  if (rows.error) { setLiveConnected(false); return; }
  setLiveConnected(true);
  const tbody = document.querySelector("#latest-table tbody");
  tbody.innerHTML = rows.map((r) => `
    <tr>
      <td>${escapeHtml(r.group)}</td><td>${escapeHtml(r.name)}</td><td>${r.kind}</td>
      <td>${formatValue(r.value)}</td><td>${new Date(r.ts * 1000).toLocaleTimeString()}</td>
    </tr>`).join("");
}

async function pollSeries() {
  for (const key of live.selected) {
    const [group, name] = key.split("|");
    const since = live.lastTs[key] || 0;
    const points = await fetchJson(`/api/metrics/series?group=${encodeURIComponent(group)}&name=${encodeURIComponent(name)}&since=${since}`);
    if (!Array.isArray(points) || !points.length) continue;
    const buf = live.buffers[key] || (live.buffers[key] = []);
    for (const p of points) buf.push(p);
    live.lastTs[key] = points[points.length - 1].ts;
    const MAX_POINTS = 800;
    if (buf.length > MAX_POINTS) buf.splice(0, buf.length - MAX_POINTS);
  }
  updateChart();
}

function setLiveConnected(ok) {
  if (live.connected === ok) return;
  live.connected = ok;
  document.getElementById("live-disconnected").classList.toggle("hidden", ok);
  document.getElementById("live-body").classList.toggle("hidden", !ok);
}

function formatValue(v) {
  if (Array.isArray(v)) return "[" + v.map((x) => (typeof x === "number" ? x.toFixed(3) : x)).join(", ") + "]";
  if (typeof v === "number") return Number.isInteger(v) ? String(v) : v.toFixed(4);
  return String(v);
}

// =========================================================== chart (Chart.js)
//
// Chart.js is vendored locally (web_static/vendor/chart.umd.js, MIT - see
// vendor/README.md) and loaded by index.html before this file, so `Chart`
// below is a real global, not a stub. One Chart instance is created lazily
// (getChart()) and mutated in place on every poll tick - new datasets and
// scales assigned, then chart.update() - rather than destroyed/recreated,
// which is what keeps the built-in hover tooltip and legend responsive
// across ticks instead of flickering.

const CHART_PALETTE = ["#4c8dff", "#3fb872", "#f0655c", "#eab308", "#a855f7", "#06b6d4", "#f97316", "#ec4899"];
let chartInstance = null;

// Chart.js only draws what's in chart.data.datasets - with none selected
// yet there's nothing for it to draw at all, so this plugin paints the
// same "check a metric" placeholder the old hand-rolled canvas used to.
const NO_DATA_PLUGIN = {
  id: "noDataMessage",
  afterDraw(chart) {
    if (chart.data.datasets.length) return;
    const { ctx, chartArea } = chart;
    ctx.save();
    ctx.fillStyle = "#5a6b7d";
    ctx.font = "13px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(
      "Check a metric on the left to plot it",
      (chartArea.left + chartArea.right) / 2,
      (chartArea.top + chartArea.bottom) / 2
    );
    ctx.restore();
  },
};

function getChart() {
  if (chartInstance) return chartInstance;
  const canvas = document.getElementById("chart");
  chartInstance = new Chart(canvas.getContext("2d"), {
    type: "line",
    data: { datasets: [] },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      // "index"/intersect:false is Chart.js's native equivalent of the old
      // hand-rolled hover tooltip: hovering anywhere over a timestamp
      // shows every plotted series' value at that point, not just the one
      // line directly under the cursor.
      interaction: { mode: "index", intersect: false },
      plugins: {
        legend: { position: "top", labels: { boxWidth: 12, font: { size: 11 } } },
        tooltip: {
          callbacks: {
            title: (items) => (items.length ? new Date(items[0].parsed.x * 1000).toLocaleTimeString() : ""),
            label: (item) => `${item.dataset.label} = ${formatValue(item.parsed.y)}`,
          },
        },
      },
      scales: {
        x: {
          type: "linear",
          ticks: { callback: (v) => new Date(v * 1000).toLocaleTimeString(), maxTicksLimit: 8 },
        },
      },
    },
    plugins: [NO_DATA_PLUGIN],
  });
  return chartInstance;
}

function updateChart() {
  const chart = getChart();

  // Vector metrics split into one line per component, mirroring the Qt
  // panel's own [i]-indexed series-splitting behavior.
  const series = {};
  for (const key of live.selected) {
    const buf = live.buffers[key] || [];
    if (!buf.length) continue;
    const isVector = Array.isArray(buf[buf.length - 1].value);
    if (isVector) {
      const count = buf[buf.length - 1].value.length;
      for (let i = 0; i < count; i++) {
        series[`${key}[${i}]`] = buf
          .filter((p) => Array.isArray(p.value) && p.value.length > i)
          .map((p) => ({ x: p.ts, y: p.value[i] }));
      }
    } else {
      const pts = buf.filter((p) => typeof p.value === "number").map((p) => ({ x: p.ts, y: p.value }));
      if (pts.length) series[key] = pts;
    }
  }
  const labels = Object.keys(series);

  // "Follow latest" slides a fixed-width window forward to the newest
  // point seen across every plotted series - the same behavior as the Qt
  // panel's own Follow-latest checkbox (data_visualization_panel.py).
  // Typing into From/To (see onManualRangeEdit in startLiveTab) sets
  // live.range.follow = false, which skips this and freezes the view at
  // exactly what was typed.
  if (live.range.follow) {
    const allTs = labels.flatMap((l) => series[l].map((p) => p.x));
    const toTs = allTs.length ? Math.max(...allTs) : Date.now() / 1000;
    const fromTs = toTs - RANGE_WINDOW_DEFAULT_S;
    live.range.to = toTs;
    live.range.from = fromTs;
    setRangeInputs(fromTs, toTs);
  }
  const { from, to } = live.range;

  // Only the portion of each buffer inside the current range is charted -
  // the buffers themselves keep up to MAX_POINTS regardless (pollSeries()),
  // so scrubbing From/To backward doesn't need a re-fetch as long as the
  // point is still in the buffer.
  if (from != null && to != null) {
    labels.forEach((label) => { series[label] = series[label].filter((p) => p.x >= from && p.x <= to); });
  }
  const visibleLabels = labels.filter((l) => series[l].length);

  // Each series gets its own y-axis, auto-scaled to that series' own
  // min/max - Chart.js's native replacement for the old manual per-series
  // percentage normalization: a running counter like plc_mock's
  // sample_count and a millisecond-scale metric like scan_time_ms no
  // longer fight over one shared axis (that fight was "the y axis keeps
  // increasing" - the counter dragging a shared axis upward every poll
  // while everything else flattened near zero). Only the first axis is
  // actually drawn; the rest exist purely to scale their own line and stay
  // hidden so the plot area isn't crowded with duplicate tick columns.
  const scales = {
    x: {
      type: "linear",
      min: from ?? undefined,
      max: to ?? undefined,
      ticks: { callback: (v) => new Date(v * 1000).toLocaleTimeString(), maxTicksLimit: 8 },
    },
  };
  const datasets = visibleLabels.map((label, i) => {
    const axisId = `y${i}`;
    scales[axisId] = {
      type: "linear",
      display: i === 0,
      position: i % 2 === 0 ? "left" : "right",
      grid: { drawOnChartArea: i === 0 },
    };
    return {
      label,
      data: series[label],
      borderColor: CHART_PALETTE[i % CHART_PALETTE.length],
      backgroundColor: CHART_PALETTE[i % CHART_PALETTE.length],
      yAxisID: axisId,
      borderWidth: 1.8,
      pointRadius: 0,
      tension: 0,
    };
  });

  chart.options.scales = scales;
  chart.data.datasets = datasets;
  chart.update();
}

// =========================================================== utils

function escapeHtml(s) {
  return String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}

init();
