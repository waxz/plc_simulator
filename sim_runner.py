"""
sim_runner.py - run physics and the PLC exchange on their own thread,
decoupled from rendering.

Without this, both advance once per GUI frame: a slow frame (lots to draw)
slows the simulation down, a 10 ms exchange channel actually goes out every
~16.7 ms, and the PLC only ever sees state that changes at frame rate.
SimulationRunner ticks every `tick_s` (default 2 ms) of wall time: it
advances physics and then lets the exchange publish whatever channels are
due. The GUI only renders - at its own, lower rate - and tells the runner
whether to play and how fast.

Speed:
  `speed` = 1.0   real time: physics advances by the elapsed wall time
  `speed` = k     k times real time (the elapsed wall time x k)
  `speed` = None  as fast as possible: whole physics steps back to back in
                  short slices (SLICE_S), releasing the physics lock between
                  slices so the GUI can still draw and the PLC exchange can
                  still read/write the I/O image.
"""
import collections
import threading
import time
from typing import Optional

import stall_watch

SLICE_S = 0.008   # max mode: wall time per burst of physics steps, holding the lock
STALL_S = 0.25    # a tick (physics + exchange) longer than this, or a gap between ticks: a stall
WINDOW_S = 5.0    # health figures cover the last WINDOW_S seconds


class SimulationRunner:
    def __init__(self, physics_engine, exchange=None, tick_s: float = 0.002, speed: Optional[float] = 1.0):
        self.physics = physics_engine
        self.exchange = exchange
        self.tick_s = tick_s
        self.playing = True
        self.speed = speed          # None = as fast as possible
        self._running = False
        self._thread: Optional[threading.Thread] = None
        # Health counters: ticks run, and the worst lateness of a tick vs. its
        # schedule (s) - a large value means the host can't keep up.
        self.ticks = 0
        self.max_lateness_s = 0.0
        # The debug panel's figures (health()): recent ticks as (wall end, tick s,
        # physics s, sim time), and the stalls (wall start, length s).
        self._recent = collections.deque()
        self.stalls = collections.deque(maxlen=20)
        self.stall_count = 0
        self.longest_stall_s = 0.0
        self._watch: Optional[stall_watch.StallWatch] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="SimulationRunner", daemon=True)
        self._thread.start()
        # Stuck for 2 s (not ticking at all): every thread's stack to sim_stall.log.
        self._watch = stall_watch.StallWatch("simulation", lambda: self.ticks)
        self._watch.start()

    def stop(self) -> None:
        self._running = False
        if self._watch is not None:
            self._watch.stop()
            self._watch = None
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None

    @property
    def running(self) -> bool:
        return self._running

    def health(self) -> dict:
        """Recent timing, for the window's debug panel (WINDOW_S seconds)."""
        now = time.perf_counter()
        recent = [r for r in list(self._recent) if now - r[0] <= WINDOW_S]
        out = {"ticks": self.ticks, "tick_hz": 0.0, "tick_ms_avg": 0.0, "tick_ms_max": 0.0,
               "physics_ms_max": 0.0, "ratio": 0.0, "stall_count": self.stall_count,
               "longest_stall_s": self.longest_stall_s, "last_stall": None,
               "dropped_s": getattr(self.physics, "dropped_time", 0.0)}
        if len(recent) >= 2:
            span = recent[-1][0] - recent[0][0]
            out["tick_hz"] = (len(recent) - 1) / span if span > 0 else 0.0
            out["tick_ms_avg"] = 1000.0 * sum(r[1] for r in recent) / len(recent)
            out["tick_ms_max"] = 1000.0 * max(r[1] for r in recent)
            out["physics_ms_max"] = 1000.0 * max(r[2] for r in recent)
            out["ratio"] = (recent[-1][3] - recent[0][3]) / span if span > 0 else 0.0
        if self.stalls:
            start, length = self.stalls[-1]
            out["last_stall"] = {"ago_s": now - start - length, "length_s": length}
        return out

    def _note_stall(self, start: float, length: float, what: str) -> None:
        self.stalls.append((start, length))
        self.stall_count += 1
        self.longest_stall_s = max(self.longest_stall_s, length)
        print(f"[STALL] {time.strftime('%H:%M:%S')} the simulation loop {what} {length:.2f} s "
              f"(sim time {getattr(self.physics, 'sim_time', 0.0):.1f} s; stacks of a stall over "
              f"{stall_watch.DUMP_AFTER_S:g} s: {stall_watch.LOG})", flush=True)

    def _run(self) -> None:
        last = last_end = time.perf_counter()
        next_tick = last + self.tick_s
        while self._running:
            now = time.perf_counter()
            dt = now - last
            last = now
            if now - last_end > STALL_S:            # this thread didn't run (host, GIL, a debugger)
                self._note_stall(last_end, now - last_end, "did not run for")
            if self.playing and self.speed is None:
                self.physics.step_for(SLICE_S)
            elif self.playing:
                self.physics.update(dt * self.speed)
            t_physics = time.perf_counter() - now
            if self.exchange is not None:
                self.exchange.sync()
            self.ticks += 1
            end = last_end = time.perf_counter()
            if end - now > STALL_S:                 # a tick that took too long (physics or the exchange)
                self._note_stall(now, end - now, f"spent (physics {t_physics:.2f} s) in one tick:")
            self._recent.append((end, end - now, t_physics, getattr(self.physics, "sim_time", 0.0)))
            while self._recent and end - self._recent[0][0] > WINDOW_S:
                self._recent.popleft()

            if self.playing and self.speed is None:
                # Flat out: no schedule to keep - just let the other threads
                # (GUI, exchange clients) take the lock for a moment.
                time.sleep(0.0005)
                next_tick = time.perf_counter() + self.tick_s
                continue
            lateness = time.perf_counter() - next_tick
            if lateness > self.max_lateness_s:
                self.max_lateness_s = lateness
            next_tick += self.tick_s
            if lateness > 0.1:
                # Far behind (debugger, host stall): resynchronize instead of
                # running a burst of back-to-back catch-up ticks.
                next_tick = time.perf_counter() + self.tick_s
            delay = next_tick - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
