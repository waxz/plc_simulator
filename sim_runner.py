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
import threading
import time
from typing import Optional

SLICE_S = 0.008   # max mode: wall time per burst of physics steps, holding the lock


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

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="SimulationRunner", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None

    @property
    def running(self) -> bool:
        return self._running

    def _run(self) -> None:
        last = time.perf_counter()
        next_tick = last + self.tick_s
        while self._running:
            now = time.perf_counter()
            dt = now - last
            last = now
            if self.playing and self.speed is None:
                self.physics.step_for(SLICE_S)
            elif self.playing:
                self.physics.update(dt * self.speed)
            if self.exchange is not None:
                self.exchange.sync()
            self.ticks += 1

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
