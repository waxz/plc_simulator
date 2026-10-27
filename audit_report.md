# Code Audit — Bugs, Memory Leaks, Bottlenecks

Scope: full `python_sim` codebase, read-only audit (3 parallel passes: the Qt CAD editor, the physics/networking core, and the remaining pygame/config/utility files). Each finding lists file:line, mechanism, severity, and confidence.

## How to read this

- **Severity**: High = data corruption / silent failure / user-visible freeze. Medium = real bug with a narrow trigger, or a bottleneck that scales badly. Low = cosmetic or requires an unusual sequence.
- **Confidence**: High = directly confirmed by reading the code path end-to-end. Medium = plausible mechanism, not reproduced live. Low = needs a specific sequence to trigger.

---

## 1. Bugs

### 1.1 `MQTTSyncEngine._lock` doesn't cover the physics/render loop's access to `config` — **High / High**
`mqtt_data_exchange.py:115,206-222,256-258` only locks around `ComponentMessage.serialize()`/`deserialize()`. But `main.py` calls `physics_engine.update(dt)` (main thread) and `mqtt.sync()` back-to-back with **no lock around the physics call**. `physics.py`'s `apply_conveyor_kinematics()`/`evaluate_sensors()` read/write `belt.speed`, `sensor.sensor_states`, etc. directly on the same `config` object a background TCP recv thread is concurrently `setattr()`-ing via `deserialize()`. The lock only serializes MQTT-side accesses *against each other* — not against the physics loop. A belt's speed/angle pair, or a sensor's multi-bit state string, can be read mid-update, torn between old and new values. Also covers structural risk: deleting a component from `config.belts`/`config.sensors` in the editor while a recv thread's `find_component_by_id` is mid-iteration is an unguarded concurrent list mutation.

### 1.2 A failed publish permanently un-throttles that channel — **Medium-High / High**
`mqtt_data_exchange.py:204-222`:
```python
with self._lock:
    try:
        payload = ch.message.serialize(self.config)
    except Exception as exc:
        print(f"[MQTTSync] Error serializing message: {exc}")
        continue          # skips ch.mark() below
    ...
ch.mark()
```
If `serialize()` raises once (e.g. a mapped component got deleted at runtime), `ch.mark()` never runs, so `is_due()` stays `True` forever — the channel retries and fails on *every frame* thereafter instead of respecting its configured interval.

### 1.3 `TcpFramedServer.send()` blocks the main/GUI thread sequentially, while holding the config lock — **High / High**
`tcp_framed_transport.py:234-248` loops over connected clients and calls blocking `sendall()` one at a time (up to `RECV_TIMEOUT_S`=2.0s each). Called from `MQTTSyncEngine.sync()` (`mqtt_data_exchange.py:221`), itself inside `with self._lock:`, itself called every frame from the main thread. **One stalled client stalls the entire app** (rendering, physics, and — because the lock is held — every other client's inbound deserialize) for up to 2s; N slow clients compound to N×2s sequentially. This is the same class of bug the earlier `settimeout()` fix addressed, just reintroduced at the broadcast layer.

### 1.4 Fast button clicks can be lost entirely (pygame editor) — **High / High**
`visualizer.py:147-176`. `process_events()` drains all queued pygame events in one pass. A `MOUSEBUTTONDOWN` sets `sensor_states = True`; if the matching `MOUSEBUTTONUP` lands in the *same* drain (any fast click, or a frame hitch that lets events queue up), `sensor_states` is set back to `False` before `mqtt.sync()` ever runs — the press is never observed on the wire. There's a regression test for the paused-gate variant of this bug (`ButtonPressPlayGateTest`) but nothing covers this same-frame-drain variant, and there's no edge-latch ("was pressed this frame, not yet synced").

### 1.5 Sensors sampled once per frame, after all physics substeps — **Medium / Medium-High**
`physics.py:798-812`: `update()` runs 4 substeps (spawn/kinematics/step/sinks) then calls `evaluate_sensors()` **once**, after all of them. A parcel spawned, crossing a sensor, and consumed by a sink within the same frame's earlier substeps never appears in `self.parcels` when sensors are finally evaluated — invisible to sensors, never reaches `sensor_states`, never reaches the wire. Fast parcels that fully traverse a narrow sensor within one frame have the same problem.

### 1.6 Receive-side exceptions kill the whole client connection; publish-side exceptions don't — **Medium / Medium**
`tcp_framed_transport.py:324-344`: `_recv_loop` only catches `(ConnectionError, OSError, ValueError)` around the *entire* loop. `find_component_by_id` (`mqtt_message_binding.py:45`) raises a bare `ValueError` for a component no longer in `config` — that propagates straight into `_recv_loop`'s catch, disconnecting that client entirely. Contrast with §1.2's publish path, which catches broadly and just skips a tick. Same failure mode, very different blast radius depending on direction.

### 1.7 `beginMacro`/`endMacro` aren't exception-safe (2 sites) — **Medium / Medium**
`cad_editor_prototype.py` — `PlacedItem.mouseReleaseEvent` (~934-944) and `CanvasView._paste_clipboard` (~2424-2437). No `try/finally` around the command-push loop. If any iteration raises (malformed model data mid-paste, a future validator), the macro is left open and every subsequent undo/redo action for the rest of the session silently folds into it.

### 1.8 Deferred `QTimer.singleShot(0, ...)` after a sensor-type combo change has no teardown guard — **Medium / Medium**
`cad_editor_prototype.py:2773`, callback `_rebuild_and_reload` (2777-2781). If the Properties dock closes or the app starts shutting down in that single-tick window, the callback runs against a possibly-destroyed `self.form`/`self.current` — the same failure class as the already-fixed `RuntimeError: libshiboken: Internal C++ object already deleted` (that fix only covers `scene.selectionChanged`, not this second deferred callback).

### 1.9 Placement dialog isn't registered as UI-blocking (pygame editor) — **Medium-High / High**
`editor_ui.py:372-380` vs `visualizer.py:120-121,213-217`. `is_ui_blocking_point()` checks the properties/toolbar/manager windows but never `self.placement_window` (opened by every "+ Linear Belt"/"+ Sensor" button). Any click while that dialog is open — including on its own buttons — also runs canvas hit-testing and can start a drag underneath it.

### 1.10 Component Manager delete doesn't prune dangling MQTT topic-component refs; Properties-panel delete does — **Medium / High**
`editor_ui.py:518-529` vs `531-545`. Two delete code paths in the same editor leave `config.mqttSettings.topics` in different states depending on which UI triggered the delete. Doesn't crash (the reader guards against the missing id) but is a real data-hygiene divergence.

### 1.11 Undo/redo replay of an Id edit bypasses uniqueness validation — **Low-Medium / Low-Medium**
`cad_editor_prototype.py:2783-2809`. `_apply_id` validates uniqueness before pushing the command; `_do_apply_id` (what `redo()`/`undo()` actually calls) doesn't re-check. A specific add/rename/undo sequence can produce two live items sharing an id.

### 1.12 `Optional[...] = None` fields render as the literal string `"None"` (pygame properties panel) — **Low-Medium / High**
`editor_ui.py:306,337-340`. `Sensor.laserCount`/`beamDistance` default to `None`; switching `sensorType` to `laser_banner` without those ever being set shows `"None"` in the text field. Doesn't crash (pygame_gui accepts any string), unlike the equivalent Qt-side bug that was fixed — just confusing UI unless the user then edits and resubmits.

### 1.13 Undo of a delete doesn't restore selection — **Low / Low-Medium**
`cad_editor_prototype.py:2051-2087`. `AddItemCommand`/`DeleteItemsCommand` only do scene/list/config bookkeeping, never re-select. After undoing a delete, the Properties panel stays blank even though the item is visually back.

---

## 2. Memory Leaks

### 2.1 `QUndoStack` has no `setUndoLimit()` — **Medium / High**
`cad_editor_prototype.py:2961`. Grows unbounded for the life of the session — every drag, every property keystroke, every add/paste/delete keeps a live reference to its `PlacedItem`. Since `QGraphicsScene.removeItem()` doesn't destroy the object, a "deleted" item's full geometry/model graph stays alive indefinitely purely because undo history references it.

### 2.2 Per-component-id caches in `PhysicsEngine` are never pruned on deletion — **Low / Medium**
`physics.py`: `self.source_timers` (128, 297-301) and `self._sensor_shapes_cache` (615-629), keyed by id, only grow. A long editing session that adds/removes many sources/sensors accumulates stale entries for the life of the process. Small per-entry footprint, but genuine unbounded-with-churn growth.

### 2.3 `TcpFramedServer.stop()` doesn't join per-client recv threads — **Low / Low-Medium**
`tcp_framed_transport.py:250-262`. Only `_accept_thread` gets joined; per-client threads are force-disconnected but not joined. Daemon threads, so it doesn't block process exit — but a `stop()` immediately followed by `start()` (e.g. a transport reload) could race with still-unwinding old threads.

### 2.4 Measure-tool markers accumulate without bound (by design, no cap) — **Low / High**
`cad_editor_prototype.py:275-350`. Only pruned by the explicit "Clear Measurements" action. A long measuring session silently accumulates hundreds of graphics items, which also then participate in every future full-scene traversal (compounding §3.1/§3.2 below). Intentional feature gap rather than a true bug, but worth a cap or a session reminder.

---

## 3. Performance Bottlenecks

### 3.1 Full O(N) scene rescan on every single drag pixel, no caching — **High / High**
`cad_editor_prototype.py:578-598`, `PlacedItem._sibling_key_points()`/`_sibling_edges()`, called from `itemChange()` on every `ItemPositionChange` (i.e. continuously while dragging). Each call does a full `scene().items()` traversal and recomputes every other item's keypoints/edges from scratch — no memoization across the two calls or across consecutive move events. O(N) geometry work per mouse-move, every frame, during any drag.

### 3.2 Live coordinate readout recomputes geometry for every item on every plain hover — **High / High**
`cad_editor_prototype.py:2356-2377`, `CanvasView.mouseMoveEvent`. `_all_points_and_edges()` iterates the entire `items_list` on **every** mouse move over the canvas (mouse tracking is always on) just to feed the coordinate label — independent of §3.1's drag-time cost. During an active drag, both scans run simultaneously.

### 3.3 `TcpFramedServer.send()`'s sequential broadcast compounds directly with §1.3 — **High / High**
Same code as §1.3, listed here for its throughput angle: even without a stalled client, broadcasting to N clients is N sequential blocking `sendall()` calls on the main thread, all while holding the config lock.

### 3.4 Every sensor/button/LED force-repaints every tick, regardless of change — **High / High**
`cad_editor_prototype.py:3225-3250`, `_refresh_sensor_visuals()`, called from `_tick()` at ~60fps. A laser-banner `SensorItem.paint()` draws up to `laserCount` (up to 128) individual ellipses per repaint — so several laser banners repaint hundreds of ellipses 60×/second even when nothing changed since the last tick.

### 3.5 `LedItem` doesn't reduce its keypoint/edge set the way curved belts do — **Medium-High / High**
`cad_editor_prototype.py:1286-1336`, `CIRCLE_SEGMENTS = 20`. `BeltItem` deliberately collapses a curved belt's many-segment ring to ~5 logical keypoints; `LedItem` has no equivalent override, so it contributes 41 keypoints + 20 edges to every other item's alignment scan during any drag (directly multiplying §3.1's cost per LED on the canvas).

### 3.6 `apply_conveyor_kinematics()` computes and discards belt velocities every parcel/substep when `useSurfaceVelocity` is on (the default) — **Medium / High**
`physics.py:420-426`. `belt_vels` is computed via a full per-belt loop with trig, then never referenced when the `useSurfaceVelocity` branch is taken — which `test_config.json` sets as the active default. Doubles the effective per-parcel belt-scan cost for no reason.

### 3.7 pygame editor: fresh `Surface` + `rotate()` allocated every frame for every component — **Medium-High / High**
`visualizer.py`: `draw_belt` (348-354), `draw_sensor` (401-408), `draw_button` (452-460), `draw_source` (478-486), `draw_sink` (493-496). None cache the constructed/rotated surface keyed on `(width, height, color, rotation)` — rebuilt from scratch at up to 60Hz for every component even when static.

### 3.8 pygame editor: text labels re-rasterized via `Font.render()` every frame — **Medium / High**
`visualizer.py:455,470,708,780-801`. Includes several genuinely static strings (button/LED labels, the "Controls: [Space] Play/Pause…" help line) re-rendered 30-60×/second for no reason. Per-parcel speed labels also scale with live parcel count.

### 3.9 `_snap_tolerance_scene_units()` re-derives the view transform on every call — **Low / Medium**
`cad_editor_prototype.py:600-606` and `2352-2354` (two copies). Looks up `scene().views()`/`transform().m11()` fresh every call rather than caching at drag-start; only needs to change if the user zooms mid-drag (rare).

---

## Cross-cutting observation

§1.1 + §1.3 + §1.2/§1.6 are related: the TCP/MQTT layer's locking and error-handling were built incrementally (config-write races were never fully closed off, and the publish/receive paths diverged in how forgiving they are of the same exception). These are worth fixing together rather than piecemeal, since a fix to one (e.g. widening the lock) changes the shape of the others (e.g. how long the lock is held, which feeds directly into §1.3/§3.3's stall duration).

---

## Improvement Plan (prioritized)

### Phase 1 — Correctness & stalls (do first)
1. **Widen `MQTTSyncEngine`'s locking** to cover physics's reads/writes of shared belt/sensor fields, or restructure so `physics_engine.update()` and `mqtt.sync()` never touch the same fields without a lock. Simplest safe option: take the lock around the whole `physics_engine.update(dt)` call too (same lock `mqtt.sync()` already uses), accepting slightly more serialization for real correctness. *(§1.1)*
2. **Stop holding the config lock during network sends.** Serialize the payload under the lock, release it, *then* broadcast. Removes the "one stalled client blocks physics + all other clients" failure mode. *(§1.3, §3.3)*
3. **Always call `ch.mark()`**, even on a caught serialize exception — move it out of the `try` or use `finally`. Prevents a permanently-retrying broken channel from spamming every frame. *(§1.2)*
4. **Add an edge-latch for button presses** in the pygame editor: instead of only setting `sensor_states` on down and clearing on up, track "pressed-and-not-yet-synced" so a same-frame down+up still produces one observable pulse before `mqtt.sync()` runs. *(§1.4)*
5. **Reconcile publish vs. receive exception handling** in the TCP layer — decide deliberately whether a bad frame should drop the connection or just that message, and make both directions consistent. *(§1.6)*

### Phase 2 — Performance (do once Phase 1 lands, since some interact)
6. **Cache sibling keypoints/edges per drag**, not per mouse-move pixel — compute once when a drag starts (or throttle to once per N ms), invalidate only when the scene's item set changes. *(§3.1, biggest single win)*
7. **Decouple the coordinate-readout scan from every hover event** — cache the points/edges list, rebuild only on add/remove/resize, not on every mouse move. *(§3.2)*
8. **Add a dirty-flag to sensor/button/LED repaint** — only call `.update()` when `sensor_states`/`isOn` actually changed since last tick, not unconditionally every tick. *(§3.4)*
9. **Give `LedItem` a reduced keypoint/edge override**, matching the existing `BeltItem` curved-belt pattern. *(§3.5)*
10. **Drop the discarded `belt_vels` computation** in `apply_conveyor_kinematics()` when `useSurfaceVelocity` is on. *(§3.6)*
11. **Cache pygame surfaces and rendered text** keyed by their content/params, invalidate on change. Biggest win for larger layouts in the pygame editor. *(§3.7, §3.8)*

### Phase 3 — Hygiene & safety nets
12. Wrap the two `beginMacro`/`endMacro` sites in `try/finally`. *(§1.7)*
13. Guard the deferred sensor-type-change callback against a destroyed panel (check `self.current`/widget validity, or use a `QPointer`-style pattern). *(§1.8)*
14. Register the placement dialog in `is_ui_blocking_point()`. *(§1.9)*
15. Make Component Manager delete prune dangling topic-component refs, same as the Properties panel path. *(§1.10)*
16. Fix `Optional` field display to show a sensible placeholder instead of `"None"`. *(§1.12)*
17. Prune `PhysicsEngine`'s per-id caches when a source/sensor is deleted. *(§2.2)*

### Phase 4 — Polish
18. `self.undo_stack.setUndoLimit(...)` to a reasonable bound (e.g. 200). *(§2.1)*
19. Join per-client recv threads in `TcpFramedServer.stop()`. *(§2.3)*
20. Restore selection after undoing a delete. *(§1.13)*
21. Re-validate uniqueness inside `_do_apply_id` so undo/redo replay can't produce duplicate ids. *(§1.11)*
22. Cache the view-transform lookup for the drag duration. *(§3.9)*
23. Add a soft cap or periodic reminder for accumulated measure-tool markers. *(§2.4)*
