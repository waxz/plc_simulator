"""
sim_runner.py - run physics and the PLC exchange on their own thread,
decoupled from rendering.

Without this, both advance once per GUI frame (~60 Hz): a 10 ms exchange
channel actually goes out every ~16.7 ms, and every frame the PLC receives
reflects state that only changes at frame rate. SimulationRunner ticks every
`tick_s` (default 2 ms) of wall time: it advances physics by the real elapsed
time (PhysicsEngine.update() runs whole physicsDt steps) and then lets the
exchange publish whatever channels are due. The GUI only renders, and tells
the runner whether the simulation is playing.
"""
import threading
import time
from typing import Optional


class SimulationRunner:
    def __init__(self, physics_engine, exchange=None, tick_s: float = 0.002):
        self.physics = physics_engine
        self.exchange = exchange
        self.tick_s = tick_s
        self.playing = True
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
            if self.playing:
                self.physics.update(dt)
            if self.exchange is not None:
                self.exchange.sync()
            self.ticks += 1

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
