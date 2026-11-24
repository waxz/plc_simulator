// delta.js - the page's side of stream_delta.py: a live stream sent as what
// changed. The first message on a connection is the whole state; each next
// one ({"Δ": ...}) holds only what changed, next to the keys always sent
// (t, w, layout). Generic: any dict of name -> value, any list of rows keyed
// by their first element, any other value replaced whole.
//
//   let state = null;
//   source.onmessage = (e) => { state = applyDelta(state, JSON.parse(e.data)); use(state); };
//   (a new connection: state = null - its first message is whole again)

export const ALWAYS = ["t", "w", "layout"];

export function applyDelta(state, msg) {
  const d = msg["Δ"];
  if (!d) return msg;                                  // a whole state (a key frame)
  const out = { ...(state || {}) };
  for (const k of ALWAYS) if (k in msg) out[k] = msg[k];
  Object.assign(out, d.s || {});
  for (const [key, entry] of Object.entries(d.d || {})) {
    const value = { ...(out[key] || {}) };
    Object.assign(value, entry.u || {});
    for (const name of entry.x || []) delete value[name];
    out[key] = value;
  }
  for (const [key, rows] of Object.entries(d.r || {})) {
    const prev = out[key] || [];
    const byId = new Map(prev.map((r) => [r[0], r.slice()]));
    for (const row of rows.n || []) byId.set(row[0], row.slice());
    for (const change of rows.c || []) {
      const row = byId.get(change[0]);
      if (!row) continue;
      for (let i = 1; i < change.length; i += 2) row[change[i]] = change[i + 1];
    }
    for (const gone of rows.x || []) byId.delete(gone);
    let order = rows.o;
    if (!order) {
      const known = new Set(prev.map((r) => r[0]));
      order = prev.map((r) => r[0]).filter((k) => byId.has(k)).concat([...byId.keys()].filter((k) => !known.has(k)));
    }
    out[key] = order.filter((k) => byId.has(k)).map((k) => byId.get(k));
  }
  for (const key of d.x || []) delete out[key];
  return out;
}
