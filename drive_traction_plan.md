# Design Plan — Belt Drive Model, Friction Traction, Fixed-Step Timing

Status: **Phases 0–4 implemented (simulator side), 2026-10-27.** Open items: unchecked boxes in §5.

### Implementation notes (deviations from the design below)

- The fixed-step loop lives in `PhysicsEngine.update()` / `step()` / `run_for()`, not a separate `stepper.py`.
- `KinematicTraction` was not moved into `traction.py`. The legacy code stays in
  `PhysicsEngine.apply_conveyor_kinematics()` and friction parcels skip it; this avoided
  churn in code the 18 existing tests guard.
- `physicsDt` defaults to 4 ms, not 2 ms. On the 48-belt `test_config.json` layout this
  measures 2.0× real time (friction) and 2.35× (kinematic). Belt geometry is now cached
  (`surface.py`), which is what made friction mode fast enough (it was 0.26× before caching).
- Added after the design: `sim_runner.SimulationRunner` (`main.py --sim-thread`, always on
  with `--headless`) runs physics + exchange on their own 2 ms tick. Exchange channels
  were capped at the GUI frame rate before that, and at ~64 Hz on Windows by
  `time.monotonic()`'s 15.6 ms resolution (now `perf_counter()`).
- Belt feedback is its own frame type, `FrameType.BELT_FEEDBACK` (6), and is **opt-in**
  (`mqttSettings.beltFeedback`). The PLC's `CommunicationProgram` (ax_dev
  `tcp_client.st`) deserializes every received frame into its IO_MAPPING struct without
  checking `ExtractedFrameType`, so an extra frame type would corrupt it until P4.5 is done.
- `clock/simNanoseconds` (UINT64, simulated time) is available next to the wall-clock
  `clock/nanoseconds`.
- `--headless` never actually ran physics before (nothing set `playing`); fixed.
- Tests: `test_drive.py`, `test_friction_traction.py`, `test_tcp_exchange.py` (350 total, all passing).

## 1. Goal

Make belt **motor speed control** realistic enough that PLC logic developed against
this simulator (conveyor sync / gap control, laser-banner dimensioning, parcel
tracking in `ax_dev`) behaves the same on real hardware:

- A PLC speed command drives a **motor model** (ramp + lag), not the belt surface directly.
- Parcels are moved by **bounded friction**, so start/stop, slip, pushing, and
  belt-to-belt handoff come from physics instead of tuning constants.
- Physics, sensors, and drives run on a **fixed time step**, decoupled from rendering.
- The simulator reports **actual belt speed and encoder position** back to the PLC.

### Non-goals (this plan)

- New belt types (merge, diverter, roller zones, incline). The design must make
  them easy (§3.3), but they are a follow-up.
- 3D physics, or changing physics library: pymunk stays.
- React frontend (`../src/`) feature work, beyond keeping its config schema compatible.

## 2. Current State (measured 2026-10-27)

| Area | Today | Problem |
|---|---|---|
| Belt speed | PLC writes `belt_N/speed` → used directly as surface speed | Instant speed steps; no ramp/inertia; no feedback to PLC |
| Traction | `dynamic_velocity_modifier` **overwrites** `body.velocity` each step | Infinite grip; mass/μ ignored; no slip |
| Belt selection | Center point only, one "best" belt | Straddling parcels jump between belt behaviours |
| Handoff | Smootherstep blend over `transition_momentum_duration` (0.6 s) | Handoff point set by a tuning constant (~0.3 m overshoot at 1 m/s) |
| Timing | `dt` = `clock.tick(60)`, 4 sub-steps, sensors once per frame | 33 mm sensor resolution at 2 m/s; variable dt |
| Curved belt | `ω = speed/radius`, rotation capped by remaining arc angle | Works well (r error 0.5 %, exit angle 89.6°) |
| Queueing | pymunk collisions | Works; parcels stay 0.40 m apart |

These results come from a headless experiment run against `PhysicsEngine`. The
existing tests in `test_physics.py` (18) all pass.

## 3. Design

### 3.1 Module layout

```
physics.py            PhysicsEngine (orchestration only, slimmer)
drive.py        NEW   BeltDrive: setpoint → ramp → lag → actual speed, encoder position
surface.py      NEW   belt surface-velocity functions per belt type + point-on-belt tests
traction.py     NEW   TractionModel interface; KinematicTraction (legacy), FrictionTraction
stepper.py      NEW   FixedStepper: accumulator, max-steps guard, per-step hooks
```

`PhysicsEngine.update(dt)` keeps its signature (both GUIs and tests call it) but
becomes: `stepper.advance(dt)`. Each fixed step does:

1. `drive.update(h)` for every belt
2. `handle_sources(h)`
3. `traction.pre_step(h)`: set carrier velocities from belt surfaces
4. `space.step(h)`
5. `handle_sinks()`
6. `evaluate_sensors()` (now every step)

### 3.2 Belt drive (`drive.py`)

The setpoint is the existing wire field `belt.speed` (PLC → sim), so the meaning on
the wire is unchanged.

```
ref    += clamp(setpoint - ref, -decel·h, +accel·h)   # VFD ramp (decel when |sp| < |ref|)
actual += (ref - actual) · (1 - exp(-h / tau))        # mechanical/drive lag
actual  = clamp(actual, -speedMax, speedMax)
position_m += actual · h
encoder     = (encoder + round(actual·h·countsPerM)) mod 2^32   # UDINT wrap, like a real counter
```

New optional `Belt.drive` config (pydantic `DriveConfig`). If it is absent, the
drive is **ideal** (`actual = setpoint`), which gives today's behaviour exactly.

| Field | Unit | Default |
|---|---|---|
| `accel` | m/s² | 2.0 |
| `decel` | m/s² | 2.0 |
| `tau` | s | 0.05 (0 = no lag) |
| `speedMax` | m/s | 5.0 |
| `countsPerM` | counts/m | 10000 |

Runtime outputs live on the `Belt` model as `actualSpeed: float` and
`encoderCount: int`, declared with `Field(exclude=True)`. This lets the existing
`id/field` binding (`find_component_by_id` + getattr/setattr) publish them with no
new plumbing, and they are never saved into layout files.

### 3.3 Belt surfaces (`surface.py`)

Each belt type exposes:

```python
contains(px, py) -> bool
surface_velocity(px, py, actual_speed) -> (vx, vy)
```

- linear: `actual · dir(rotation + directionAngle)`
- curved: `ω × r`, with `ω = ±actual / radius`. The current
  `sign(directionAngle)` direction rule is kept as-is and documented.

New belt types later (merge, roller zone, diverter) only add a surface class.

### 3.4 Friction traction (`traction.py`)

This uses pymunk's standard top-down friction technique. Each parcel gets:

- a **kinematic carrier body** that sits on the parcel center every step
- `PivotJoint(carrier, parcel)` with `max_bias = 0` and `max_force = μ·m·g`, which
  gives linear Coulomb-like friction
- `GearJoint(carrier, parcel)` with `max_bias = 0` and `max_force = μ·m·g·r_eff`,
  which gives rotational friction (`r_eff` ≈ ⅓ of the parcel's half-diagonal)

In `pre_step`, for each parcel:

1. Sample K contact points: center, 4 corners, 4 edge midpoints (K = 9).
2. For each point, find the belt under it (the topmost/last belt if belts overlap)
   and its surface velocity `v_i`. Points that are off every belt use the
   **floor**: `v = 0`, with `μ_floor`.
3. Fit a rigid motion to the samples:
   `v_c = mean(v_i)` and `ω = mean(cross(r_i, v_i − v_c) / |r_i|²)`.
   This is exact for both straight and curved belts. Where the parcel straddles
   two belts, the mismatch produces a torque, which is real skew behaviour.
4. Set `carrier.velocity = v_c` and `carrier.angular_velocity = ω`.
5. Scale `max_force` by the μ-weighted fraction of points on belt vs floor.

Effects:

- Acceleration is capped at `μ·g`: a 0.3 μ gives about 2.9 m/s².
- A stop is a real stopping distance, `v²/(2μg)`.
- Parcels pushed against a stopped belt slide when the force balance says they
  should. **This is a deliberate behaviour change:** the accumulation result will
  differ from today's.

Config:

- `SimulatorConfig.tractionModel: Literal["kinematic", "friction"] = "kinematic"`
- `Belt.friction` (μ, default 0.5) and `SimulatorConfig.floorFriction` (default 0.6)
- `Source.minMass` / `Source.maxMass` (kg, default 1.0)

`KinematicTraction` is the existing code moved behind the interface, unchanged
(velocity override + smootherstep blend), so the 18 existing tests keep guarding it.

**Alternative considered:** apply explicit Coulomb forces ourselves. It is simpler,
but it is stiff and chatters at low relative speed unless dt is tiny. The joint
approach lets pymunk's solver handle the stick/slip transition. Fallback only.

### 3.5 Fixed-step timing (`stepper.py`)

- `SimulatorConfig.physicsDt` defaults to 0.002 s. `simulationSteps` is kept for
  the kinematic model and ignored by the stepper.
- Accumulator with `maxStepsPerAdvance` (default 50). Overflow is dropped and
  counted (`stepper.dropped_time`), so the existing Sim/Real ratio indicator shows
  it honestly.
- `PhysicsEngine.run_for(seconds)` gives deterministic headless stepping for tests.
- Sensors are evaluated every step. A rising-edge latch per IR sensor
  (`sensor.edgeCount`, excluded from save) lets the PLC detect pulses shorter than
  the exchange interval.
- The PLC exchange still runs on the GUI/main loop: it reads the latest state
  under the lock. `clock/nanoseconds` is derived from `sim_time`, so PLC-side
  `dt_ns` matches physics time.

### 3.6 PLC I/O changes

| Direction | Item | Type | Notes |
|---|---|---|---|
| PLC → sim | `belt_N/speed` | FLOAT | unchanged; now means *setpoint* |
| PLC → sim | `belt_N/directionAngle` | FLOAT | unchanged |
| sim → PLC | `belt_N/actualSpeed` | FLOAT | new |
| sim → PLC | `belt_N/encoderCount` | UINT32 | new; wraps like a real encoder |

The mapping builders (`custom_component_mapping`) currently emit the same belt
fields for both directions. They need a **direction-aware belt mapping**: inbound
publishes speed + directionAngle, outbound publishes actualSpeed + encoderCount.

The ST side (`ax_dev/src/project/site_1/types.st` → regenerate
`types-generated.st`) needs a matching input struct. That is a separate change in
the `ax_dev` repo.

## 4. Acceptance Criteria

All of these are checked headlessly with `run_for()`, and are deterministic
(sources seeded).

| # | Scenario | Pass if |
|---|---|---|
| A1 | Drive step 0→2 m/s, accel 2, tau 0 | actual reaches 2.0 at t = 1.00 s ± 1 step; slope 2.0 ± 1 % |
| A2 | Drive with tau 0.1, setpoint step, accel ∞ | 63 % of the step at t = 0.10 s ± 2 % |
| A3 | Ideal drive (no `drive` config) | actual == setpoint every step (back-compat) |
| A4 | Encoder | counts == ∫v·dt·countsPerM ± 1; wraps at 2³² |
| B1 | Parcel dropped at rest on a 1 m/s belt, μ 0.3 | initial accel = μg ± 5 %; reaches 1 m/s at t ≈ 0.34 s |
| B2 | Parcel at 1 m/s onto a stopped belt | stopping distance v²/(2μg) ± 5 % |
| B3 | Parcel 50 % on 1 m/s and 50 % on 0 m/s | carrier v_c ≈ 0.5 m/s; yaw torque sign matches the side |
| B4 | Curved r = 2 m, 90° | radius error < 1 %; exit heading within 2° of tangent |
| B5 | Queue of 5 against a stopped belt | no overlap (< 1 mm); front slide matches force balance ± 10 % |
| B6 | Belt speed ramp while carrying a parcel, μ high | parcel tracks actualSpeed within 1 % after settling |
| C1 | Sensors per step at 2 ms | IR edge time error ≤ 1 step; no missed pulse for a parcel ≥ 1 step long |
| C2 | Laser banner + recorded actualSpeed into `tools/laser_banner_scanner_sim.py` algorithm | length/width within that tool's documented tolerances |
| D1 | Kinematic mode | all 18 existing `test_physics.py` tests pass unchanged |
| D2 | Performance, friction mode, 100 parcels, 2 ms step | Sim/Real ratio ≥ 1.0 on dev machine (record the number) |

## 5. Todo List

### Phase 0 — Prep

- [x] P0.1 Commit or stash the current uncommitted changes in `python_sim` (10 modified files) so this work starts from a clean base
- [ ] P0.2 Add `pytest` (dev dependency via `uv`) or keep `unittest`; decide and document in README
- [x] P0.3 Add a headless `run_for(seconds)` helper and a seeded RNG for sources; turn the 2026-10-27 experiment into `test_baseline_behavior.py` (it records today's numbers)

### Phase 1 — Fixed-step timing

- [x] P1.1 `stepper.py`: accumulator, `physicsDt`, `maxStepsPerAdvance`, `dropped_time`
- [x] P1.2 `PhysicsEngine.update(dt)` delegates to the stepper; sensors evaluated per step
- [x] P1.3 IR rising-edge latch (`edgeCount`, excluded from save)
- [ ] P1.4 Surface `dropped_time` in both GUIs' timing overlay and in the `[TIMING]` console line
- [x] P1.5 Tests C1 + D1 green

### Phase 2 — Belt drive

- [x] P2.1 `DriveConfig` + optional `Belt.drive`; runtime `actualSpeed` / `encoderCount` fields with `exclude=True`
- [x] P2.2 `drive.py` `BeltDrive.update(h)` (ramp, lag, clamp, encoder wrap)
- [x] P2.3 Physics reads `actualSpeed` (not `speed`) for surfaces, in **both** traction models
- [ ] P2.4 GUIs: show setpoint vs actual on belts; edit drive params in the Qt properties panel
- [x] P2.5 Tests A1–A4

### Phase 3 — Surfaces and friction traction

- [x] P3.1 `surface.py`: extract `contains` / `surface_velocity` for linear + curved from `physics.py`
- [ ] P3.2 `traction.py`: `TractionModel` interface; move existing logic into `KinematicTraction` (no behaviour change, D1 stays green)
- [x] P3.3 `FrictionTraction`: carrier body, Pivot/Gear joints, created and removed with the parcel (sources/sinks/clear)
- [x] P3.4 9-point sampling + rigid-motion fit + floor fraction
- [x] P3.5 Config: `tractionModel`, `Belt.friction`, `floorFriction`, `Source.minMass/maxMass`
- [x] P3.6 Tests B1–B6
- [x] P3.7 Performance check D2; if slow, add a spatial index (pymunk `point_query` against static sensor shapes per belt) for point-on-belt lookups

### Phase 4 — PLC I/O

- [x] P4.1 Direction-aware belt mapping in `mqtt_message_binding.py` (inbound speed/directionAngle; outbound actualSpeed/encoderCount)
- [ ] P4.2 Channel registry + web editor: allow the new belt fields; validator updated
- [x] P4.3 `plc_mock.py`: send speed setpoints and read back actualSpeed/encoderCount; extend `test_plc_mock.py`
- [x] P4.4 Fix audit §1.3 (`TcpFramedServer.send()` blocking while holding the config lock): send outside the lock or per-client non-blocking queues
- [ ] P4.5 **ax_dev repo:** first make `CommunicationProgram` check `tcp_praser.ExtractedFrameType = FRAME_TYPE_IO_MAPPING` before deserializing; then add the belt feedback struct to `types.st`, regenerate `types-generated.st`, `apax build`

### Phase 5 — Validation and switch-over

- [ ] P5.1 Test C2: replay the laser banner + actualSpeed through the scanner algorithm
- [ ] P5.2 Run `test_config.json` / `sim_config.json` layouts in friction mode; tune default μ values for visually sane behaviour
- [ ] P5.3 Decide whether to make `tractionModel: "friction"` the default; keep `kinematic` available
- [ ] P5.4 Update README (architecture, new config fields, units) and React editor schema (`../src/`) so it round-trips the new optional fields

### Follow-ups (out of scope here)

- [ ] Belt types: merge, diverter/pusher (moving collider), roller zones (accumulation), incline (gravity component)
- [ ] One drive driving several belt segments (`driveId` grouping)
- [ ] Drive fault simulation (trip, overload when the parcel mass pushes the force sum over a limit)

## 6. Risks and Open Questions

1. **Encoder unit on the wire.** Recommendation: UINT32 counts with wrap (like real
   hardware) rather than FLOAT metres, which lose precision after long runs. Needs
   confirming against what the ST side wants.
2. **Behaviour change in accumulation** (§3.4). Existing layouts that relied on
   "infinite grip" queueing may need stops/zones. That's why the default stays
   `kinematic` until P5.3.
3. **Joint stability** at 2 ms with very light parcels or high μ: mitigate with
   `space.iterations` and solver checks in B-tests. The explicit-force fallback is
   documented in §3.4.
4. **CPU cost.** Moving from 60 Hz × 4 sub-steps (240 Hz) to 500 Hz with 9-point
   sampling is about 2× the steps and 9× the point tests. D2 measures it; P3.7 is
   the mitigation.
5. **React editor** (`../src/`) shares the config schema. All new fields are
   optional with defaults, so old files still load, but the editor will drop unknown
   fields on save until P5.4.
