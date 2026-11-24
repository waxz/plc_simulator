// playout_test.mjs - the 3D page's playout buffer (playout.js), run by test_playout.py
// (node view3d_web/playout_test.mjs): frames sent evenly at 30/s and arriving in
// bursts, as over a tunnel, are played back evenly; it never goes back.
import assert from "node:assert/strict";
import { Playout, lerpDeg, byId } from "./playout.js";

function run(arrivalOf, seconds = 20) {
  const p = new Playout();
  const sent = [];
  for (let w = 100; w < 100 + seconds; w += 1 / 30) sent.push(w);
  const arrivals = sent.map((w, i) => ({ w, at: arrivalOf(w, i) })).sort((a, b) => a.at - b.at);
  let k = 0, last = -Infinity, holds = 0, steps = [];
  for (let now = 0; now < seconds + 5; now += 1 / 60) {            // a 60 Hz screen
    while (k < arrivals.length && arrivals[k].at <= now) p.push({ w: arrivals[k++].w }, now);
    const s = p.sample(now);
    if (!s) continue;
    const shown = s.b ? s.a.w + (s.b.w - s.a.w) * s.f : s.a.w;
    assert.ok(shown >= last - 1e-9, "playback went back");
    if (now > 5 && now < seconds - 1) {                            // settled: not before the first second
      if (shown - last < 1e-9) holds++;
      steps.push(shown - last);
    }
    last = shown;
  }
  return { p, holds, steps };
}

// locally: frames arrive 2 ms after they are sent - a short buffer, no holds
{
  const { p, holds } = run((w) => w - 100 + 0.002);
  assert.ok(p.delay <= 0.1, `local delay ${p.delay}`);
  assert.equal(holds, 0);
}

// a tunnel: 80 ms latency, and frames held back and delivered in bursts every 250 ms
{
  const { p, holds, steps } = run((w) => Math.ceil((w - 100 + 0.08) / 0.25) * 0.25);
  assert.ok(p.delay > 0.25 && p.delay < 0.6, `tunnel delay ${p.delay}`);
  assert.equal(holds, 0, "ran dry");
  const sorted = steps.slice().sort((a, b) => a - b);
  const median = sorted[sorted.length >> 1];
  assert.ok(Math.abs(median - 1 / 60) < 0.002, `even steps: median ${median}`);
  assert.ok(sorted[sorted.length - 1] < 2 / 60, `largest step ${sorted[sorted.length - 1]}`);
}

// random jitter up to 300 ms
{
  let seed = 7;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  const { holds } = run((w) => w - 100 + 0.05 + 0.3 * rnd());
  assert.ok(holds < 5, `holds ${holds}`);
}

// a new simulator (its clock starts again): the buffer starts again
{
  const p = new Playout();
  p.push({ w: 500 }, 0);
  p.push({ w: 500.1 }, 0.1);
  p.push({ w: 3 }, 0.2);
  assert.deepEqual(p.frames.map((f) => f.w), [3]);
}

// a page that does not draw for a while keeps a few seconds, not every frame
{
  const p = new Playout();
  for (let i = 0; i < 3000; i++) p.push({ w: i / 30 }, i / 30);
  assert.ok(p.frames.length <= 31 * (3 + 1) + 2, `held ${p.frames.length}`);
  const s = p.sample(100);
  assert.ok(s.a.w > 99, "plays from now when drawing resumes");
}

assert.equal(lerpDeg(350, 10, 0.5), 360);
assert.equal(lerpDeg(10, 350, 0.5), 0);
assert.equal(byId([["a", 1], ["b", 2]]).get("b")[1], 2);
console.log("playout ok");
