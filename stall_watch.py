"""
stall_watch.py - find out why the simulator froze.

A StallWatch watches a loop through a counter the loop advances (the
simulation runner's ticks, the window's frames). When the counter has not
moved for DUMP_AFTER_S, the loop is stuck: the watch writes every thread's
stack to LOG (sim_stall.log, next to this file), once per stall, with the
time, so the line the loop was stuck in is on record. It prints one line
when the loop runs again, with how long it was stuck.

    watch = StallWatch("simulation", lambda: runner.ticks)
    watch.start()
    ...
    watch.stop()
"""
import os
import sys
import threading
import time
import traceback
from typing import Callable, Optional

DUMP_AFTER_S = 2.0
POLL_S = 0.25
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_stall.log")
_log_lock = threading.Lock()


def dump_stacks(title: str) -> str:
    """Every thread's stack, as text."""
    names = {t.ident: t.name for t in threading.enumerate()}
    lines = [title]
    for ident, frame in sys._current_frames().items():
        lines.append(f"\n--- thread {names.get(ident, '?')} ({ident}) ---")
        lines.extend(line.rstrip("\n") for line in traceback.format_stack(frame))
    return "\n".join(lines) + "\n"


def write_log(text: str) -> None:
    with _log_lock:
        try:
            with open(LOG, "a", encoding="utf-8") as f:
                f.write(text + "\n")
        except OSError:
            pass


class StallWatch:
    def __init__(self, name: str, counter: Callable[[], int], after_s: float = DUMP_AFTER_S):
        self.name = name
        self.counter = counter
        self.after_s = after_s
        self.dumps = 0                       # stalls written to LOG
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name=f"StallWatch-{self.name}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        last, since, dumped = self.counter(), time.monotonic(), False
        while not self._stop.wait(POLL_S):
            value = self.counter()
            now = time.monotonic()
            if value != last:
                if dumped:
                    print(f"[STALL] {time.strftime('%H:%M:%S')} the {self.name} loop runs again after "
                          f"{now - since:.1f} s", flush=True)
                    write_log(f"{time.strftime('%Y-%m-%d %H:%M:%S')} the {self.name} loop ran again after "
                              f"{now - since:.1f} s\n")
                last, since, dumped = value, now, False
            elif not dumped and now - since >= self.after_s:
                dumped = True
                self.dumps += 1
                write_log(dump_stacks(f"==== {time.strftime('%Y-%m-%d %H:%M:%S')} (pid {os.getpid()}): the "
                                      f"{self.name} loop has not run for {now - since:.1f} s; every thread:"))
                print(f"[STALL] {time.strftime('%H:%M:%S')} the {self.name} loop has not run for "
                      f"{now - since:.1f} s: stacks written to {LOG}", flush=True)
