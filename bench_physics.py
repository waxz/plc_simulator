"""
bench_physics.py - physics cost per step vs parcel count, without a window.

A synthetic plant with station_1.json's physics settings (belts driving
parcels, the same drive model on every belt, a laser banner and an IR sensor
per belt, sinks): parallel 50 m belts, parcels about 1 m apart, ~40 per
belt. Prints ms per physics step and the real-time factor it allows.

    python bench_physics.py                 # 50, 100, 200 parcels
    python bench_physics.py 100 400 --dt 0.008
    python bench_physics.py --profile 200   # cProfile of 300 steps
"""
import cProfile
import pstats
import sys
import time

from config_parser import Belt, Sensor, Sink, SimulatorConfig, load_config
from physics import PhysicsEngine

DRIVE = load_config("station_1.json").belts[0].drive   # the plant's drive model (VFD ramp, maxLoad)


def plant(n_parcels: int, dt: float):
    belts_n = max(2, (n_parcels + 39) // 40)          # ~40 parcels per 50 m belt
    belts, sensors, sinks = [], [], []
    for i in range(belts_n):
        y = i * 2.5
        belts.append(Belt(id=f"belt_{i}", type="linear", x=25.0, y=y, length=50.0, beltWidth=1.2,
                          rotation=0.0, speed=1.0, directionAngle=0.0, color="#888888",
                          drive=DRIVE.model_copy() if DRIVE else None))
        sensors.append(Sensor(id=f"banner_{i}", x=30.0, y=y, width=0.1, height=1.2, rotation=0.0,
                              sensorType="laser_banner", laserCount=32, beamDistance=0.035, label="", color="#f00"))
        sensors.append(Sensor(id=f"ir_{i}", x=40.0, y=y, width=0.1, height=1.2, rotation=0.0,
                              sensorType="ir", label="", color="#f00"))
        sinks.append(Sink(id=f"sink_{i}", x=51.0, y=y, width=1.0, height=1.5, label=""))
    cfg = SimulatorConfig(belts=belts, sensors=sensors, sinks=sinks)
    cfg.physicsDt = dt
    cfg.useSurfaceVelocity = True      # as station_1.json: belts drive the parcels
    cfg.kineticFriction = True
    e = PhysicsEngine(cfg)
    for k in range(n_parcels):
        i, j = k % belts_n, k // belts_n
        e.add_parcel(2.0 + j * 1.1, i * 2.5, 0.5, 0.4)
    return e


def measure(n: int, dt: float, seconds: float = 3.0):
    e = plant(n, dt)
    for _ in range(25):
        e.step(dt)                                   # warm up
    t0, steps = time.perf_counter(), 0
    while time.perf_counter() - t0 < seconds:
        e.step(dt)
        steps += 1
    per_step = (time.perf_counter() - t0) / steps
    return per_step, len(e.parcels)


if __name__ == "__main__":
    dt = float(sys.argv[sys.argv.index("--dt") + 1]) if "--dt" in sys.argv else 0.004
    if "--profile" in sys.argv:
        n = int(sys.argv[sys.argv.index("--profile") + 1])
        e = plant(n, dt)
        pr = cProfile.Profile()
        pr.enable()
        for _ in range(300):
            e.step(dt)
        pr.disable()
        pstats.Stats(pr).sort_stats("tottime").print_stats(18)
        sys.exit()
    counts = [int(a) for a in sys.argv[1:] if a.isdigit()] or [50, 100, 200]
    print(f"physicsDt {dt * 1000:g} ms ({1 / dt:.0f} steps per simulated second)")
    for n in counts:
        per_step, left = measure(n, dt)
        print(f"{n:4d} parcels: {per_step * 1000:6.2f} ms/step -> {dt / per_step:5.2f}x real time "
              f"({left} still on belts)")
