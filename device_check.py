"""
device_check.py - does every conveyor device do what its PLC command says?

A PLC (plc_mock.PlcImage: the same I/O frames plc_mock.py sends over TCP)
commands each device of a test layout, one at a time, and the result is
checked against the device's definition (config_parser.py) by watching a
parcel and the device's feedback in the PLC's input frame:

  belt "vfd"        run / reverse / speed (m/s)
  belt "speed"      signed speed: + along the belt, - against it
  belt "frequency"  run / reverse / Hz through its drive train (50 Hz = its design speed)
  roller conveyor   the same commands on rollers
  curved belt       a load follows the arc, counter-clockwise
  pusher diverter   enable: pushed off at commandAngle (+90 left, -90 right); off: the belt carries it on
  wheel diverter    enable, commandSpeed, commandAngle: the wheels carry the load at commandSpeed the
                    way they point (commandAngle, at most 60 deg; 0 straight on, wheelAngle drawn by
                    the window); enable off: they stop, the load stands on them
  turntable         targetIndex: turns to angles[i] (90, 180) with its load; its belt then runs
                    the turned way; index / inPosition / turning / occupied in the feedback

Every command goes PLC copy of the layout -> output frame bytes ->
simulator, every feedback simulator -> input frame bytes -> PLC copy, so
a wrong offset or bit in the frames fails here too.

    python device_check.py                      # every check, a table
    python device_check.py --write device_check.json   # the test layout, to open in the simulator
"""
import argparse
import copy
import math
import sys
from typing import Callable, List, Optional, Tuple

import io_signals
from config_parser import Belt, Diverter, DriveConfig, SimulatorConfig, Turntable
from physics import PhysicsEngine
from plc_mock import PlcImage

SCAN_S = 0.01            # a PLC scan / exchange interval
LANE = 3.0               # m between the test lanes (y)


def _belt(id, x, y, length, rotation=0.0, **kw) -> Belt:
    base = dict(id=id, type="linear", x=x, y=y, length=length, beltWidth=0.6, rotation=rotation, speed=1.0,
                directionAngle=0.0, color="#64748b", commandFormat="vfd")
    base.update(kw)
    return Belt(**base)


def layout() -> SimulatorConfig:
    """One lane per device (every belt runs +x unless said), nothing else."""
    belts = [
        _belt("belt_vfd", 0.0, 0 * LANE, 6.0),
        _belt("belt_speed", 0.0, 1 * LANE, 6.0, commandFormat="speed"),
        _belt("belt_hz", 0.0, 2 * LANE, 6.0, commandFormat="frequency", speed=1.2,
              drive=DriveConfig(drumDiameter=0.1)),
        _belt("roller", 0.0, 3 * LANE, 6.0, surface="roller", color="#9ca3af"),
        Belt(id="curve", type="curved", x=0.0, y=4 * LANE + 2.0, length=0.0, beltWidth=0.6, rotation=0.0,
             radius=1.5, startAngle=270.0, endAngle=0.0, speed=1.0, directionAngle=90.0, color="#64748b",
             commandFormat="vfd"),                                    # directionAngle > 0: counter-clockwise
        _belt("main_pusher_r", 0.0, 6 * LANE, 6.0),
        _belt("branch_r", 0.0, 6 * LANE - 0.3 - 0.75, 1.5, rotation=-90.0),     # takes what is pushed off
        _belt("main_pusher_l", 0.0, 7 * LANE, 6.0),
        _belt("branch_l", 0.0, 7 * LANE + 0.3 + 0.75, 1.5, rotation=90.0),
        _belt("main_wheels", 0.0, 8 * LANE, 6.0),
        _belt("table_belt", 0.0, 10 * LANE, 1.2),
    ]
    diverters = [
        Diverter(id="pusher_r", x=0.0, y=6 * LANE, width=0.6, height=0.6, divertAngle=-90.0, speed=1.0),
        Diverter(id="pusher_l", x=0.0, y=7 * LANE, width=0.6, height=0.6, divertAngle=90.0, speed=1.0),
        Diverter(id="wheels", x=0.0, y=8 * LANE, width=1.0, height=0.6, diverterType="wheels",
                 divertAngle=-30.0, speed=1.0),
    ]
    tables = [Turntable(id="table", x=0.0, y=10 * LANE, radius=0.8, beltId="table_belt",
                        angles=[0.0, 90.0, 180.0], turnSpeed=180.0)]
    return SimulatorConfig(belts=belts, diverters=diverters, turntables=tables, physicsDt=0.004,
                           collisionEnabled=True)


class Rig:
    """The simulator (its own copy of the layout) and a PLC (another copy),
    talking only through the I/O frames' bytes."""

    def __init__(self, config: SimulatorConfig):
        self.sim = copy.deepcopy(config)
        self.plc = copy.deepcopy(config)
        io_signals.at_rest(self.sim)               # as the simulator starts: nothing moves until commanded
        io_signals.at_rest(self.plc)
        self.engine = PhysicsEngine(self.sim)
        self.wire_plc = PlcImage(self.plc)         # the PLC's frames (plc_mock.py)
        self.wire_sim = PlcImage(self.sim)         # the simulator's own (the same channels, io_topics)
        self.frames = 0

    def run(self, seconds: float, each: Optional[Callable[["Rig"], None]] = None) -> None:
        end = self.engine.sim_time + seconds - 1e-9
        while self.engine.sim_time < end:
            payload = self.wire_plc.send(self.plc)                              # PLC -> simulator
            self.wire_sim.output_msg.deserialize(payload, self.sim, skip_manual=True)
            self.engine.run_for(SCAN_S)
            feedback = self.wire_sim.input_msg.serialize(self.sim)              # simulator -> PLC
            self.wire_plc.receive(feedback, self.plc)
            self.frames += 1
            if each is not None:
                each(self)

    def plc_of(self, kind: str, id: str):
        return next(c for c in getattr(self.plc, kind) if c.id == id)

    def sim_of(self, kind: str, id: str):
        return next(c for c in getattr(self.sim, kind) if c.id == id)

    def parcel(self, x, y, w=0.4, h=0.3):
        return self.engine.add_parcel(x, y, w, h)


def _pos(p) -> Tuple[float, float]:
    return float(p.body.position.x), float(p.body.position.y)


def _heading(a, b) -> float:
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def _angle_off(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


class Results:
    def __init__(self):
        self.rows: List[Tuple[str, str, bool, str]] = []

    def check(self, device: str, what: str, ok: bool, detail: str) -> bool:
        self.rows.append((device, what, bool(ok), detail))
        return ok

    @property
    def failed(self):
        return [r for r in self.rows if not r[2]]


# -- belts ------------------------------------------------------------------------------------

def _belt_motion(cfg, belt_id, y, set_cmd, seconds=1.5):
    """(speed m/s, heading deg) of a load on the belt over `seconds` after a
    settling second, the belt commanded by set_cmd(plc belt)."""
    rig = Rig(cfg)
    p = rig.parcel(0.0, y)
    set_cmd(rig.plc_of("belts", belt_id))
    rig.run(1.0)                                      # the drive ramps, the load picks up its speed
    a = _pos(p)
    rig.run(seconds)
    b = _pos(p)
    return math.dist(a, b) / seconds, _heading(a, b), rig


def check_belts(cfg, res: Results) -> None:
    def vfd(run, reverse, speed):
        def f(b):
            b.run, b.reverse, b.speed = run, reverse, speed
        return f

    v, _h, _ = _belt_motion(cfg, "belt_vfd", 0.0, vfd(False, False, 1.0))
    res.check("belt vfd", "run off: stands", v < 0.02, f"{v:.3f} m/s")
    v, h, _ = _belt_motion(cfg, "belt_vfd", 0.0, vfd(True, False, 1.0))
    res.check("belt vfd", "run, 1.0 m/s: forward at 1.0", abs(v - 1.0) < 0.05 and _angle_off(h, 0) < 3,
              f"{v:.3f} m/s at {h:.1f} deg")
    v, h, _ = _belt_motion(cfg, "belt_vfd", 0.0, vfd(True, False, 0.5))
    res.check("belt vfd", "run, 0.5 m/s: forward at 0.5", abs(v - 0.5) < 0.03 and _angle_off(h, 0) < 3,
              f"{v:.3f} m/s at {h:.1f} deg")
    v, h, _ = _belt_motion(cfg, "belt_vfd", 0.0, vfd(True, True, 1.0))
    res.check("belt vfd", "run + reverse: backward at 1.0", abs(v - 1.0) < 0.05 and _angle_off(h, 180) < 3,
              f"{v:.3f} m/s at {h:.1f} deg")

    def signed(speed):
        def f(b):
            b.speed = speed
        return f
    v, h, _ = _belt_motion(cfg, "belt_speed", LANE, signed(0.8))
    res.check("belt speed", "+0.8: forward at 0.8", abs(v - 0.8) < 0.04 and _angle_off(h, 0) < 3,
              f"{v:.3f} m/s at {h:.1f} deg")
    v, h, _ = _belt_motion(cfg, "belt_speed", LANE, signed(-0.8))
    res.check("belt speed", "-0.8: backward at 0.8", abs(v - 0.8) < 0.04 and _angle_off(h, 180) < 3,
              f"{v:.3f} m/s at {h:.1f} deg")

    def hz(run, reverse, f_hz):
        def f(b):
            b.run, b.reverse, b.frequency = run, reverse, f_hz
        return f
    design = next(b for b in cfg.belts if b.id == "belt_hz").speed
    v, h, _ = _belt_motion(cfg, "belt_hz", 2 * LANE, hz(True, False, 50.0))
    res.check("belt frequency", f"50 Hz: forward at its design {design} m/s",
              abs(v - design) < 0.06 and _angle_off(h, 0) < 3, f"{v:.3f} m/s at {h:.1f} deg")
    v, h, _ = _belt_motion(cfg, "belt_hz", 2 * LANE, hz(True, False, 25.0))
    res.check("belt frequency", f"25 Hz: half, {design / 2:.2f} m/s", abs(v - design / 2) < 0.04,
              f"{v:.3f} m/s at {h:.1f} deg")
    v, h, _ = _belt_motion(cfg, "belt_hz", 2 * LANE, hz(True, True, 50.0))
    res.check("belt frequency", "50 Hz + reverse: backward", abs(v - design) < 0.06 and _angle_off(h, 180) < 3,
              f"{v:.3f} m/s at {h:.1f} deg")
    v, _h, _ = _belt_motion(cfg, "belt_hz", 2 * LANE, hz(False, False, 50.0))
    res.check("belt frequency", "run off at 50 Hz: stands", v < 0.02, f"{v:.3f} m/s")

    v, h, _ = _belt_motion(cfg, "roller", 3 * LANE, vfd(True, False, 0.6))
    res.check("roller conveyor", "run, 0.6 m/s: forward at 0.6", abs(v - 0.6) < 0.04 and _angle_off(h, 0) < 3,
              f"{v:.3f} m/s at {h:.1f} deg")
    v, h, _ = _belt_motion(cfg, "roller", 3 * LANE, vfd(True, True, 0.6))
    res.check("roller conveyor", "run + reverse: backward", abs(v - 0.6) < 0.04 and _angle_off(h, 180) < 3,
              f"{v:.3f} m/s at {h:.1f} deg")

    # the curve (270 -> 0 deg): directionAngle > 0 runs it counter-clockwise, else clockwise
    cx, cy = 0.0, 4 * LANE + 2.0
    for direction, start, lo, hi, name in ((90.0, 285.0, 300.0, 359.0, "counter-clockwise (directionAngle 90)"),
                                           (-90.0, 345.0, 271.0, 330.0, "clockwise (directionAngle -90)")):
        layout_ = copy.deepcopy(cfg)
        next(b for b in layout_.belts if b.id == "curve").directionAngle = direction
        rig = Rig(layout_)
        a0 = math.radians(start)
        p = rig.parcel(cx + 1.5 * math.cos(a0), cy + 1.5 * math.sin(a0), 0.3, 0.3)
        b = rig.plc_of("belts", "curve")
        b.run, b.speed = True, 1.0
        rig.run(0.6)
        x, y = _pos(p)
        angle = math.degrees(math.atan2(y - cy, x - cx)) % 360.0
        radius = math.hypot(x - cx, y - cy)
        res.check("curved belt", f"a load follows the arc {name}",
                  lo < angle < hi and abs(radius - 1.5) < 0.2, f"{start:g} -> {angle:.0f} deg, r {radius:.2f} m")


# -- diverters --------------------------------------------------------------------------------

def _divert_run(cfg, belt_id, div_id, y, divert, angle=None, speed=None, seconds=3.0, enable=None):
    """A load from 1.5 m upstream through the diverter, the main belt at 1 m/s.
    Returns (heading of its move over the diverter's zone .. 1 m after, rig, max wheel angle)."""
    rig = Rig(cfg)
    p = rig.parcel(-1.5, y, 0.35, 0.3)
    belt = rig.plc_of("belts", belt_id)
    belt.run, belt.speed = True, 1.0
    for b in rig.plc.belts:
        if b.id.startswith("branch"):
            b.run, b.speed = True, 1.0
    d = rig.plc_of("diverters", div_id)
    sim_d = rig.sim_of("diverters", div_id)
    if sim_d.diverterType == "wheels":       # wheels: always enabled, divert = steer to the branch
        d.enable = True if enable is None else enable
        d.commandAngle = (sim_d.divertAngle if divert else 0.0) if angle is None else angle
    else:                                    # a pusher: divert = enable, at its divertAngle
        d.enable = divert
        d.commandAngle = sim_d.divertAngle if angle is None else angle
    d.commandSpeed = d.speed if speed is None else speed
    track, wheel, in_zone = [], [0.0], []

    def each(r):
        track.append(_pos(p))
        if abs(sim_d.wheelAngle) > abs(wheel[0]):
            wheel[0] = sim_d.wheelAngle
        v = p.body.velocity
        if sim_d.present and sim_d.active and v.length > 0.2:      # how the diverter moves it
            in_zone.append(math.degrees(math.atan2(v.y, v.x)))
    rig.run(seconds, each)
    start, end = track[0], track[-1]
    heading = sorted(in_zone)[len(in_zone) // 2] if in_zone else _heading(start, end)   # the median in the zone
    return heading, rig, wheel[0], start, end


def check_diverters(cfg, res: Results) -> None:
    for div_id, belt_id, y, side, name in (("pusher_r", "main_pusher_r", 6 * LANE, -90.0, "right (-90)"),
                                           ("pusher_l", "main_pusher_l", 7 * LANE, 90.0, "left (+90)")):
        h, rig, _w, a, b = _divert_run(cfg, belt_id, div_id, y, True)
        fb = rig.plc_of("diverters", div_id)
        off = b[1] - y
        res.check("pusher diverter", f"divert: pushed at {side:+g} deg to the travel ({name})",
                  _angle_off(h, side) < 6, f"heading {h:.1f} deg in the zone")
        res.check("pusher diverter", f"divert: off onto its branch ({name})",
                  math.copysign(1, off) == math.copysign(1, side) and abs(off) > 0.6,
                  f"{off:+.2f} m across; PLC feedback active {fb.active}")
        res.check("pusher diverter", f"divert ({name}): feedback diverted = 1", fb.diverted == 1,
                  f"diverted {fb.diverted}")
        h, rig, _w, a, b = _divert_run(cfg, belt_id, div_id, y, False)
        res.check("pusher diverter", f"no divert ({name}): straight on", abs(b[1] - y) < 0.05 and b[0] > 1.0,
                  f"at ({b[0]:.2f}, {b[1] - y:+.2f})")

    y = 8 * LANE
    h, rig, w, a, b = _divert_run(cfg, "main_wheels", "wheels", y, True)
    res.check("wheel diverter", "commandAngle -30 (its divertAngle): off to the right",
              _angle_off(h, -30.0) < 6 and b[1] < y - 0.3, f"heading {h:.1f} deg, {b[1] - y:+.2f} m across")
    res.check("wheel diverter", "the wheels point -30 (wheelAngle, as drawn)", abs(w + 30.0) < 0.5,
              f"wheelAngle {w:.1f}")
    fb = rig.plc_of("diverters", "wheels")
    res.check("wheel diverter", "feedback: diverted 1", fb.diverted == 1, f"diverted {fb.diverted}")
    h, rig, w, a, b = _divert_run(cfg, "main_wheels", "wheels", y, True, angle=30.0)
    res.check("wheel diverter", "commandAngle +30: off to the left",
              _angle_off(h, 30.0) < 6 and b[1] > y + 0.3, f"heading {h:.1f} deg, wheelAngle {w:.1f}")
    h, rig, w, a, b = _divert_run(cfg, "main_wheels", "wheels", y, True, angle=-90.0)
    res.check("wheel diverter", "commandAngle -90: the wheels stop at -60",
              abs(w + 60.0) < 0.5 and _angle_off(h, -60.0) < 8, f"heading {h:.1f} deg, wheelAngle {w:.1f}")
    h, rig, w, a, b = _divert_run(cfg, "main_wheels", "wheels", y, False)
    res.check("wheel diverter", "commandAngle 0: carried straight through",
              abs(b[1] - y) < 0.05 and b[0] > 1.0 and abs(w) < 0.5, f"at ({b[0]:.2f}, {b[1] - y:+.2f})")
    h, rig, w, a, b = _divert_run(cfg, "main_wheels", "wheels", y, False, enable=False)
    sim_d = rig.sim_of("diverters", "wheels")
    res.check("wheel diverter", "enable off: the wheels stop, the load stands on them",
              sim_d.present and abs(b[0] - a[0]) > 0.5 and b[0] < 0.6, f"at ({b[0]:.2f}, {b[1] - y:+.2f})")
    rig = Rig(cfg)
    p = rig.parcel(-1.5, y, 0.35, 0.3)
    belt = rig.plc_of("belts", "main_wheels")
    belt.run, belt.speed = True, 1.0
    d = rig.plc_of("diverters", "wheels")
    d.enable, d.commandSpeed, d.commandAngle = True, 1.6, rig.sim_of("diverters", "wheels").divertAngle
    speeds = []
    sim_d = rig.sim_of("diverters", "wheels")
    rig.run(2.0, lambda r: speeds.append(p.body.velocity.length) if sim_d.present else None)
    top = max(speeds) if speeds else 0.0
    res.check("wheel diverter", "commandSpeed 1.6: the load leaves the wheels at 1.6 m/s", abs(top - 1.6) < 0.1,
              f"top speed in the zone {top:.2f} m/s")
    h, rig, w, a, b = _divert_run(cfg, "main_wheels", "wheels", y, False)
    res.check("wheel diverter", "no divert: straight on, wheels straight",
              abs(b[1] - y) < 0.05 and abs(w) < 0.5, f"{b[1] - y:+.2f} m across, wheelAngle {w:.1f}")


# -- turntable --------------------------------------------------------------------------------

def check_turntable(cfg, res: Results) -> None:
    y = 10 * LANE
    for index, angle, heading in ((1, 90.0, 90.0), (2, 180.0, 180.0)):
        rig = Rig(cfg)
        p = rig.parcel(0.0, y, 0.4, 0.3)
        t = rig.plc_of("turntables", "table")
        rig.run(0.3)
        occupied = t.occupied
        t.targetIndex = index
        rig.run(0.1)
        turning = t.turning
        rig.run(angle / 180.0 + 0.6)                  # turnSpeed 180 deg/s
        sim_t = rig.sim_of("turntables", "table")
        res.check("turntable", f"targetIndex {index}: turns to {angle:g} deg",
                  t.index == index and t.inPosition and not t.turning and abs(sim_t.angle - angle) < 0.01,
                  f"index {t.index}, inPosition {t.inPosition}, turning was {turning}, angle {sim_t.angle:.1f}")
        res.check("turntable", f"to {angle:g}: occupied with its load on board", occupied and t.occupied,
                  f"occupied {occupied} -> {t.occupied}")
        a = _pos(p)
        res.check("turntable", f"to {angle:g}: the load turned with it, stays on the table",
                  math.dist(a, (0.0, y)) < 0.05, f"load at ({a[0]:.2f}, {a[1] - y:+.2f})")
        belt = rig.plc_of("belts", "table_belt")
        belt.run, belt.speed = True, 0.5
        rig.run(1.0)
        b = _pos(p)
        h = _heading(a, b)
        res.check("turntable", f"at {angle:g}: its belt runs the load out at {heading:g} deg",
                  _angle_off(h, heading) < 5 and math.dist(a, b) > 0.3, f"heading {h:.1f} deg")
        belt.run = False
        rig.run(1.5)
        t.targetIndex = 0
        rig.run(angle / 180.0 + 0.6)
        res.check("turntable", f"back from {angle:g} to 0", t.index == 0 and t.inPosition and
                  abs(rig.sim_of("turntables", "table").angle) < 0.01, f"index {t.index}")


def run_all(cfg: Optional[SimulatorConfig] = None) -> Results:
    cfg = cfg or layout()
    res = Results()
    check_belts(cfg, res)
    check_diverters(cfg, res)
    check_turntable(cfg, res)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--write", metavar="JSON", help="write the test layout here (open it in the simulator)")
    args = ap.parse_args(argv)
    cfg = layout()
    if args.write:
        with open(args.write, "w", encoding="utf-8") as f:
            f.write(cfg.model_dump_json(indent=2))
        print(f"wrote {args.write}")
    res = run_all(cfg)
    width = max(len(r[0]) + len(r[1]) for r in res.rows) + 3
    for device, what, ok, detail in res.rows:
        print(f"{'ok  ' if ok else 'FAIL'}  {(device + ': ' + what):<{width}} {detail}")
    print(f"\n{len(res.rows) - len(res.failed)} of {len(res.rows)} checks pass")
    return 1 if res.failed else 0


if __name__ == "__main__":
    sys.exit(main())
