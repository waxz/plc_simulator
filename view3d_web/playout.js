// playout.js - a playout (jitter) buffer for the live stream (viewer.js).
//
// Over a slow or bursty link (a tunnel, Wi-Fi) the ~30 frames a second do not
// arrive evenly: several come at once, then none for a while. Drawn as they
// come, the parcels jump. So every frame is kept first, with the time the
// server sent it (`w`, view3d.py, its own clock in seconds), and played back
// on the server's time line a little behind: `delay` seconds, just enough to
// cover how unevenly frames arrive (the spread of their delays, measured over
// the last seconds), so a frame is always in hand. Between two frames the
// moving parts are interpolated (`between`).
//
// The clock: the server's time now is estimated as the local time plus the
// largest (server time - arrival time) seen lately - the frame that came
// quickest. Playback never goes back; it holds on the last frame when the
// buffer runs dry, and catches up slowly when the needed delay shrinks.

export const MIN_DELAY_S = 0.08;     // locally: about two frames
export const MAX_DELAY_S = 3.0;
const WINDOW_S = 10.0;               // how long arrival delays are remembered
const MARGIN = 1.5;                  // delay = MARGIN x the arrival spread + PAD_S
const PAD_S = 0.05;
const SHRINK_PER_S = 0.05;           // the delay comes down this much a second (playback 5% faster)

export class Playout {
  constructor() {
    this.reset();
  }

  reset() {
    this.frames = [];                // {w, state}, by w
    this.samples = [];               // {at, off}: off = server time - arrival time
    this.delay = MIN_DELAY_S;
    this.play = -Infinity;           // the server time being shown
    this.lastAt = null;
  }

  // A frame arrived (`now`: the local time in seconds, performance.now() / 1000).
  push(state, now) {
    const w = state.w;
    if (typeof w !== "number") return;
    const last = this.frames[this.frames.length - 1];
    if (last && w <= last.w) {
      if (w < last.w - 1.0) this.reset();         // the server's clock went back: a new simulator
      else return;
    }
    this.frames.push({ w, state });
    // a page not drawing (a throttled background tab) does not take frames: keep only what playback could need
    while (this.frames.length > 2 && this.frames[0].w < w - MAX_DELAY_S - 1.0) this.frames.shift();
    this.samples.push({ at: now, off: w - now });
    while (this.samples.length && this.samples[0].at < now - WINDOW_S) this.samples.shift();
  }

  // How far behind the newest frame playback runs, in s, and the server time now.
  target() {
    let hi = -Infinity, lo = Infinity;
    for (const s of this.samples) { hi = Math.max(hi, s.off); lo = Math.min(lo, s.off); }
    const need = Math.min(MAX_DELAY_S, Math.max(MIN_DELAY_S, MARGIN * (hi - lo) + PAD_S));
    return { need, offset: hi };
  }

  // The two frames around playback time and how far between them (0..1), for a
  // render at local time `now`; null before the first frame.
  sample(now) {
    if (!this.frames.length) return null;
    const { need, offset } = this.target();
    const dt = this.lastAt === null ? 0 : Math.max(0, now - this.lastAt);
    this.lastAt = now;
    // grow at once (a stall would follow otherwise), shrink slowly (no visible jump)
    this.delay = need > this.delay ? need : Math.max(need, this.delay - SHRINK_PER_S * dt);
    this.play = Math.max(this.play, now + offset - this.delay);
    const frames = this.frames;
    // drop what playback is past (keeping the frame it is on)
    let i = 0;
    while (i + 1 < frames.length && frames[i + 1].w <= this.play) i++;
    if (i > 0) frames.splice(0, i);
    const a = frames[0];
    if (this.play < a.w) return { a, b: null, f: 0 };          // the first frame: shown until the next
    const b = frames[1] || null;
    if (!b) return { a, b: null, f: 0 };                       // ran dry: hold the last frame
    return { a, b, f: Math.min(1, (this.play - a.w) / (b.w - a.w)) };
  }

  // Seconds of frames held ahead of playback.
  get buffered() {
    return this.frames.length ? Math.max(0, this.frames[this.frames.length - 1].w - this.play) : 0;
  }
}

// A row's value between two frames: positions straight, angles the short way round.
export function lerp(a, b, f) {
  return a + (b - a) * f;
}

export function lerpDeg(a, b, f) {
  const d = ((((b - a) % 360) + 540) % 360) - 180;
  return a + d * f;
}

// Rows by id (the first column), for frame b's matching row.
export function byId(rows) {
  const m = new Map();
  for (const row of rows || []) m.set(row[0], row);
  return m;
}
