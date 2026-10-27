# CAD Editor Prototype — API Coverage & Reimplementation Risk Report

Scope: evaluates `cad_editor_prototype.py` (PySide6/Qt) against the existing
production surface it's meant to eventually replace — `config_parser.py`'s
data model, `editor_ui.py`/`visualizer.py`'s editing features, and the
runtime layer (`physics.py`, `mqtt_data_exchange.py`). Written to answer two
questions: *how much of the real app does the prototype actually cover?*
and *is it safe to reimplement the remaining pieces (physics in
particular) the same way the GUI was reimplemented?*

## 0. Progress since this report

Following §4's recommendation, `physics.py`/`mqtt_data_exchange.py` were
wired in unchanged rather than reimplemented: `main.py --gui {pygame,qt}`
now selects the backend, and `--gui qt` runs `cad_editor_prototype.run()`
with the same `PhysicsEngine`/`MQTTSyncEngine` instances the pygame path
uses. `build_items_from_config()` loads a real `SimulatorConfig` into the
Qt scene (belts/sensors/sources/sinks - buttons/LEDs are reported and
skipped, still not a supported kind), a `ParcelItem` visualizes live
`physics_engine.parcels` each tick, and a Play/Pause toolbar action drives
`physics_engine.update(dt)` (not bound to Space, since `CanvasView` already
uses Space for pan). This closes the *loading* half of §3.1's persistence
gap when run via `main.py`; the standalone `cad_editor_prototype.py` demo
(no config passed) still starts from hardcoded seed belts, and **saving**
edits back to JSON is still not implemented anywhere in the Qt UI.

## 1. Verdict, up front

**Coverage**: the prototype covers component *authoring* (placing, editing,
aligning, saving as reusable templates) for 4 of 6 component kinds, and
none of the runtime behavior — no simulation, no MQTT, no file I/O. That's
expected and fine for a GUI-engine evaluation; see §2 for the detail.

**Reimplementation**: do **not** reimplement `physics.py` (or
`mqtt_data_exchange.py`). Both are already fully decoupled from pygame —
zero pygame imports in either file — so they can be called from a Qt
`QTimer` loop completely unchanged. Rewriting the physics engine to run
"natively" in Qt would mean re-deriving pymunk's collision response, the
belt-momentum-handoff state machine, and curved-belt angular kinematics
from scratch, with real risk of silently drifting from the tuned behavior
already in production. The GUI was worth reimplementing because pygame_gui
has no scene graph, no alignment guides, no docking — Qt gives you those
for free. Pymunk's physics has no equivalent gap: it doesn't care what
draws it. See §4 for the full argument.

## 2. Data-model field coverage

Cross-referenced against `config_parser.py`'s pydantic models.

### Belt
| Field | Covered? | Notes |
|---|---|---|
| id, x, y, rotation, length, beltWidth, speed, directionAngle, color | Yes | |
| type (linear/curved), shape (rectangle/quadrilateral/arc) | Yes | Arc maps to a real `type="curved"` belt, not a polygon approximation |
| radius, startAngle, endAngle | Yes | Curved belts only |
| vertexs | Yes | Freehand/triangle/quadrilateral custom shapes |
| transition_momentum_duration | Yes | |
| direction (int 1/-1) | **No** | Also never exposed in the pygame editor - not a regression |
| speedBinding, directionAngleBinding, directionBinding | **No** | MQTT/PLC tag bindings - no MQTT UI exists in the prototype at all (see §2.6) |

### Sensor
| Field | Covered? | Notes |
|---|---|---|
| id, x, y, width, height, rotation, label, color, sensorType | Yes | |
| laserCount, beamDistance | Yes | Laser banner only, shown conditionally |
| isActive, sensor_states, detectedWord | **No** | Live/detection state - meaningless without a physics loop |
| stateBinding, wordBinding | **No** | MQTT bindings |

### Source
| Field | Covered? | Notes |
|---|---|---|
| id, x, y, rotation, interval, minWidth/maxWidth, minHeight/maxHeight, label | Yes | |
| colorScheme | **No** | Multi-color parcel randomization - cosmetic, low priority |
| triggerBinding, intervalBinding | **No** | MQTT bindings |

### Sink
| Field | Covered? | Notes |
|---|---|---|
| id, x, y, width, height, label | Yes | 100% - it's a small model |

### ControlButton, LedIndicator
**Not implemented at all** - no `ButtonItem`/`LedItem` class exists, no
library templates. These two exist in the schema purely as MQTT/PLC I/O
widgets (a button the operator presses, an LED the PLC lights up);
`physics.py` never reads or writes them (confirmed - no `ControlButton`/
`LedIndicator` references anywhere in `physics.py`), so their absence
**does not block reusing physics.py at all** - it's a pure editor-
completeness gap, independent of the physics question below.

### MqttSettings / TopicConfig / TopicComponent
**Not implemented.** This mirrors the earlier decision to remove the
pygame editor's MQTT panel entirely rather than port it forward (it was
found to be disconnected from the runtime `MQTTSyncEngine` - editing topics
there never affected the live simulation). If MQTT authoring is wanted
back, the same "it's already decoupled from pygame, don't rewrite it"
argument applies here too - see §4.3.

### SimulatorConfig top-level settings
`simulationSteps`, `kineticFriction`, `collisionEnabled`, `gridSnap`,
`gridSize`, `useSurfaceVelocity`, `simulatorBackend` - **none exposed**.
These are physics-engine tuning knobs; there's nowhere to put them until
the prototype actually runs a physics engine (§4).

## 3. Editor feature coverage (vs. `editor_ui.py` / `visualizer.py`)

| Feature | Pygame | Prototype | Notes |
|---|---|---|---|
| Add belt (linear rect) | Yes | Yes | |
| Add belt (curved arc) | Yes | Yes | Now a real `type="curved"` belt |
| Add belt (custom quad, 4-vertex) | Yes (drag-handle edit) | Yes (Shape Designer only) | **Gap**: pygame lets you drag a placed quad's corner handles directly on the canvas; the prototype's vertices are fixed once placed - editing means re-opening the Shape Designer for a new template, not nudging the existing shape's points |
| Add belt (custom N-gon, N≠4) | **No** | Yes | Prototype exceeds pygame here |
| Add sensor (ir/laser_banner) | Yes | Yes | |
| Add source/sink | Yes | Yes | Added this session |
| Add button/LED | Yes | **No** | See §2 |
| Select/move/rotate | Yes | Yes | |
| Smart alignment guides | **No** | Yes | Prototype exceeds pygame - key-point capture, rotation-aware, zoom-independent tolerance |
| Rubber-band multi-select | **No** | Yes | Free from `QGraphicsView` |
| Component Manager (list/select/duplicate/delete) | Yes | Yes | |
| Copy/paste (Ctrl+C/V) | **No** (Duplicate button only) | Yes | Prototype exceeds pygame |
| Placement dialog (type exact X/Y before placing) | Yes | **No** | Prototype only supports drag-drop placement; exact position must be typed into the properties panel *after* placing, not before |
| Save/Load config JSON | Yes | **No** | See §2.7 below - the single largest functional gap |
| Grid-based snapping | No (dead field) | No (smart guides instead) | `gridSnap` is declared in the schema but never read anywhere in pygame's code - drag positions were never actually snapped to a grid there either, only `gridSize` affects the *drawn* grid spacing. The prototype's key-point smart guides are a different (arguably better) mechanism, not a gap |
| Play/Pause simulation | Yes | **No** | No physics loop exists (§4) |
| MQTT settings/debug panel | Removed (found broken) | **No** (never existed) | Intentional in both |

### 3.1 The persistence gap

The prototype has **zero file I/O** - no `load_config()` call, no
`model_dump_json()`, no `open()` anywhere in the file. Every session starts
from the two hardcoded seed belts and ends when the window closes. This is
the most consequential gap for real use (more than physics, arguably):
without it, the prototype can't open `test_config.json`, can't save a
session, and can't hand a config to `main.py`/`plc_mock.py` for a real run.
It's also the cheapest gap to close - `config_parser.load_config()`/
`SimulatorConfig.model_dump_json()` already exist and are pygame-agnostic;
wiring them in is glue code (build a `SimulatorConfig` from `items_list`
grouped by `.kind`, or populate `items_list` from a loaded one), not a
reimplementation.

## 4. Should physics (and MQTT) be reimplemented too?

### 4.1 Why the GUI reimplementation was justified

`editor_ui.py`/`visualizer.py` were rewritten because pygame + pygame_gui
provide no scene graph, no rotation-aware hit-testing, no docking panels,
and no alignment-guide primitives - every one of those had to be hand-built
in the original code (and inconsistently: six copies of the same belt-
polygon math, a single overloaded drag-state flag, absolute-pixel widget
layout). Qt's `QGraphicsScene`/`QGraphicsItem`/`QDockWidget` provide all of
that natively. The reimplementation traded a large amount of custom
geometry/layout code for framework features - a good trade.

### 4.2 Why physics is a different situation

```
$ grep "^import\|^from" physics.py
from config_parser import Belt
from typing import Dict
from pymunk.vec2d import Vec2d
import math
import random
import pymunk
from typing import List, Tuple, Optional
from config_parser import SimulatorConfig
from math_utils import point_in_polygon, point_in_arc, get_rect_vertices, get_belt_polygon
```

No pygame. `PhysicsEngine.__init__(self, config: SimulatorConfig)` and
`.update(dt)` are the entire public surface; it reads/mutates the belt and
sensor model objects directly and holds pymunk's own body/shape objects
internally. Nothing about it assumes a pygame render loop - `main.py`'s
`while running: ... physics_engine.update(dt); renderer.render(physics_engine)`
is just a caller, not a dependency. A Qt version of that loop looks like:

```python
timer = QTimer()
timer.timeout.connect(lambda: physics_engine.update(dt))
timer.start(16)  # ~60fps
```

with a `ParcelItem(QGraphicsPolygonItem)` reading `physics_engine.parcels`
each tick to draw them - new code, but additive, not a rewrite of anything
that exists.

Contrast that with what "reimplementing physics" in Qt would actually
require re-deriving from scratch:
- Pymunk's rigid-body collision response (zero-restitution parcel-parcel
  contact, the custom `on_collision` pre-solve handlers).
- `apply_conveyor_kinematics`'s two distinct algorithms (surface-velocity
  vs. manual-impulse mode) - already flagged in the earlier code-analysis
  report as intricate enough to warrant splitting into named strategies,
  not clear enough to safely re-derive independently.
- The belt-to-belt momentum handoff state machine (`transition_momentum_v`/
  `_time`/`_duration`, `target_w_remaining` for curved-belt angular
  momentum) - several sessions of tuning encoded as small numeric
  constants and edge-case branches (see `physics.py:369-556`).
- Curved-belt angular kinematics (`_get_belt_velocity_at`'s tangent-
  velocity-proportional-to-radius calculation, the angular-sector-hit test
  before starting rotation).

None of this is exposed as a reusable primitive the way "here's a scene
graph" is - it's bespoke simulation logic. Reimplementing it risks two
parallel physics engines (pymunk-based `physics.py` for headless/PLC runs,
a hypothetical Qt-native one for the editor's live preview) that can drift
apart in exactly the subtle ways that are hardest to test for (a curved
belt that's 2% slower in one engine, a momentum handoff that resolves one
frame later in the other).

### 4.3 The same argument applies to MQTT

```
$ grep "^import\|^from" mqtt_data_exchange.py plc_mock.py | grep pygame
(no output)
```

`mqtt_data_exchange.py`/`plc_mock.py` are equally pygame-free. If MQTT
authoring comes back as a requirement, it's the same call: wire the
existing `MQTTSyncEngine` into a `QTimer`, don't rewrite the paho-mqtt
client/binding logic in a Qt-flavored way.

### 4.4 Recommendation

Treat `physics.py` and `mqtt_data_exchange.py` as **integration targets,
not reimplementation targets**. The Qt prototype's job is to be a better
*editor*; the simulation/exchange layer's job is to be correct and match
production, which it already is. The only new code needed to connect them:

1. A `SimulatorConfig` assembly step - group `items_list` by `.kind` into
   `belts=[...]`, `sensors=[...]`, etc. (trivial, since every `PlacedItem`
   already wraps the exact pydantic model `PhysicsEngine` expects).
2. A `QTimer`-driven update loop and a `ParcelItem` to visualize
   `physics_engine.parcels` + live `sensor_states` - the Qt equivalent of
   `visualizer.py`'s `draw_parcels`/`update_sensor_state`.
3. Load/save wired to `config_parser.load_config`/`model_dump_json` (§3.1) -
   needed regardless of the physics decision, and a prerequisite for it
   (there's no meaningful "run the simulation" without being able to load
   a real config file).

## 5. Suggested next-step ordering

1. **Load/save JSON** (§3.1) - closes the biggest gap, cheapest to build,
   and everything else benefits from being testable against real config
   files instead of hardcoded seed data.
2. **Wire `physics.py` via `QTimer`** + add `ParcelItem` - turns the editor
   into an editor-and-simulator, matching pygame's combined role, using
   the existing engine unchanged.
3. **Button/LED component kinds** - closes the remaining authoring gap;
   same pattern as Source/Sink (`PlacedItem` subclass + template + field
   spec), no new architecture needed.
4. **Bindings UI** (`speedBinding` etc.) and **MQTT re-integration** - only
   worth doing once there's a clear owner for what the "real" runtime
   topic/binding story should be (the original pygame panel was removed
   specifically because that story was inconsistent - see the earlier
   code-analysis report's §4.3).
