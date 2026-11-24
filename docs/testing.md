# Testing the simulator

How to run and read the simulator's tests and verification tools, which ones
to run after which change, and how they connect to the PLC side.

The PLC side (the SIMATIC AX project: its TCP library, the generated message
types, the demo program and its tests on PLCSIM Advanced or an S7-1500) has its
own test plan, `docs/testing.md` in the AX project.

## Contents

1. [Running the tests](#1-running-the-tests)
2. [The test modules](#2-the-test-modules)
3. [End-to-end tools](#3-end-to-end-tools)
4. [Manual tests](#4-manual-tests)
5. [Which tests after which change](#5-which-tests-after-which-change)
6. [Tests that depend on the layout files](#6-tests-that-depend-on-the-layout-files)
7. [Cross-repository checks](#7-cross-repository-checks)

---

## 1. Running the tests

The tests use the standard library's `unittest` (pytest is not installed).
They live in `tests/`. Run them with the project's virtual environment, from the
project folder (python_sim/, not tests/):

```powershell
# everything: about 980 tests, about 15 minutes
.venv\Scripts\python.exe -m unittest discover -p "test_*.py"

# one module, one class, one test (modules are tests.<file>)
.venv\Scripts\python.exe -m unittest tests.test_foxglove_bridge
.venv\Scripts\python.exe -m unittest tests.test_foxglove_bridge.BridgeTest
.venv\Scripts\python.exe -m unittest tests.test_foxglove_bridge.BridgeTest.test_records_field_list_once_then_values
```

No window, PLC or network beyond localhost is needed: the GUI tests run Qt and
pygame off screen, and the exchange tests open their own sockets on free ports.
The output contains the tests' own log lines (`[TCP] ...`, `[Bridge] ...`,
`[3D] ...`); the result is the last lines:

```
Ran 748 tests in 171.298s
OK
```

Some tests load this folder's layout files (`test_config.json`,
`machines_demo.json`, `logistics_demo.json`, ...). Edits to those files can fail
tests that are otherwise fine: see section 6.

## 2. The test modules

Number of test methods per module in brackets.

### PLC exchange: frames, contract, PLC programs

| Module | Covers |
|---|---|
| `test_tcp_framed_transport` (32) | the frame format (`TcpFrameCodec.pack`) and the stream parser: split frames, garbage, false markers, corrupt lengths |
| `test_io_packing` (13) | the I/O frame byte by byte; agreement between the simulator and the mock PLC |
| `test_component_message` (6), `test_mqtt_message_binding` (23), `test_mqtt_message_codec` (5) | the layout → byte mapping and its packer / unpacker |
| `test_compact_sensor_mapping` (8), `test_pack_format_auto_generation` (11) | a sensor / button / lamp packed as a bit or as a register (`packFormat`) |
| `test_tcp_exchange` (12) | the simulator's exchange engine and a PLC client over real localhost sockets |
| `test_exchange_options` (3) | the TCP exchange's robustness options, with real sockets |
| `test_st_export` (7) | the SIMATIC AX package generated from a layout: offsets, sizes, fingerprints, golden vectors, `--check` |
| `test_device_kinds` (10) | the device kinds (`device_kinds.py`) and everything derived from them; the full logistics system; `tools/frame_map.py` |
| `test_io_signals` (24) | a component's PLC signals by name, the manual command source, everything at rest at start |
| `test_plc_logic` (9), `test_plc_lines` (8), `test_plc_mock` (28) | the mock PLC's programs and its testable units |
| `test_foxglove_bridge` (7) | the PLC → Foxglove bridge end to end: I/O images, line states, metrics, events, the layout check, JSON frames, records |
| `test_plc_records` (3) | the record format (field list once, then values only): round trip, wire bytes, mismatches |
| `test_generate_frame_type` (13), `test_channel_registry` (15), `test_validate_channel_registry` (4), `test_bootstrap_channel_csv` (10), `test_mqtt_data_exchange_codegen` (17), `test_mqtt_data_exchange` (9), `test_metric_layout` (6), `test_metrics_store` (36) | channel registry, frame types, metric layouts and their code generation |

### Simulation

| Module | Covers |
|---|---|
| `test_physics` (25), `test_friction_traction` (16), `test_drive` (13) | belt transitions, traction, the drive model (ramp, lag, encoder) |
| `test_devices` (24) | IR sensors, laser banners, buttons, lamps |
| `test_machines` (11), `test_logistics` (16), `test_robot6` (10), `test_equipment_geometry` (4) | pallets, stoppers, robots, elevators, scanners, diverters, transfers; the demo layouts under PLC control |
| `test_sim_runner` (3), `test_sim_process` (6), `test_main` (7) | the simulation thread / process and `main.py`'s wiring |
| `test_view3d` (13) | the 3D view and its data |

### Designer, CAD, UI, web

| Module | Covers |
|---|---|
| `test_layout_editor` (12), `test_catalogue` (3), `test_component_library` (10), `test_machines_editor` (3) | the layout editor and its library |
| `test_cad` (21), `test_sketch_tools` (29), `test_modify` (18), `test_modify_tools` (15), `test_divide_tool` (9), `test_path_tool` (18), `test_belt_path` (17), `test_belt_from_shapes` (27) | sketching, CAD operations, belts and rollers from sketches |
| `test_dxf_import` (19), `test_dxf_crop` (6), `test_dxf_workflow` (2), `test_background_place` (4), `test_delivery_workflow` (1) | DXF drawings and the design-to-control workflow |
| `test_ui_clicks` (17), `test_ui_style` (8), `test_ui_trees` (9) | both windows driven by clicks; their look |
| `test_web_app` (33) | the web UI's routes |

## 3. End-to-end tools

These run the real programs in their own processes and check the result. They
take longer than unit tests and are run on purpose, not by `discover`.

### 3.1 `tools/verify_exchange.py`: simulator + mock PLC over TCP

```powershell
.venv\Scripts\python.exe tools/verify_exchange.py logistics_full.json --seconds 30
```

Starts the simulator headless and `plc_mock.py --autostart` on a free port, lets
them run, then checks the exchange and the plant. Expected (about 45 s with
`--seconds 30`; counts vary):

```
[verify] results
  [ok] I/O layout handshake (LAYOUT_CHECK) - I/O layout check: OK - input 165 bytes 083EBF55, output 247 bytes A188665C on both sides
  [ok] PLC receives the simulator's input frames - 2516 frames
  [ok] PLC sends its output frames - 499 frames
  [ok] no malformed / wrong-size frames at the PLC - 0
  [ok] simulator applies the PLC's frames - rx_frames 541
  [ok] no rejected frames at the simulator - rx_errors 0, rx_size_errors 0
  [ok] the PLC has a client connection - clients 1
  [ok] nothing moves before the PLC - 0 belts moving
  [ok] belts run on the PLC's commands - 24 of 25 belts moving
  [ok] parcels sorted into chutes - {...}
  [ok] scanners read - {"dws_1": 13, "bcr_1": 17, "rfid_1": 6}
  [ok] transfers lift on the PLC's command
  [ok] guard doors locked by the PLC while running
  [ok] safety relays OK (no trip)
[verify] PASSED: 14 of 14 checks
```

The checks for chutes, scanners, transfers, doors and safety only appear when
the layout has them. Exit code 0 when all pass; on a failure it prints the last
lines of both processes.

### 3.2 `tools/frame_map.py`: the frames as a picture, with checks

```powershell
.venv\Scripts\python.exe tools/frame_map.py logistics_full.json --out map.html
```

Draws the input and output frames byte by byte (an HTML page) and checks each:
the mapped offsets match the packer's size, every field reads back at its
offset, and bytes from the PLC serializers decode in the simulator. Each check
prints `[ok]` or `[FAIL]`.

### 3.3 `st_export.py --check`: the PLC package is current

```powershell
.venv\Scripts\python.exe st_export.py logistics_full.json --out <AX>\src\generated --check
```

Exit code 1 and a list of stale files when the layout no longer matches the
generated ST. For the AX project's own generated file, use its
`tools/generate_st_serde.py --check` instead (AX `docs/testing.md`, OFF-01).

## 4. Manual tests

### 4.1 Simulator with the mock PLC

```powershell
.venv\Scripts\python.exe main.py machines_demo.json --exchange-config tcp_exchange_config.json
.venv\Scripts\python.exe plc_mock.py machines_demo.json --exchange-config tcp_exchange_config.json --autostart
```

Expected: nothing moves until the mock PLC connects; then belts run, machines
cycle, and the simulator's log shows `I/O layout check with node 0: OK`.
Stop the mock PLC: the plant stops (commands at rest). Start it again: it
resumes without restarting the simulator.

### 4.2 Bridge and Foxglove

```powershell
.venv\Scripts\python.exe plc_foxglove_bridge.py machines_demo.json --listen 0.0.0.0:5002
.venv\Scripts\python.exe plc_mock.py machines_demo.json --exchange-config tcp_exchange_config.json --autostart --bridge 127.0.0.1:5002
```

Then either open Foxglove (Open connection → Foxglove WebSocket →
`ws://localhost:8765`) or print what the bridge publishes without Foxglove:

```powershell
.venv\Scripts\python.exe foxglove_ws_probe.py ws://127.0.0.1:8765 --list
.venv\Scripts\python.exe foxglove_ws_probe.py ws://127.0.0.1:8765 --topic /plc/lines --seconds 5
```

Expected topics: `/plc/outputs`, `/plc/inputs`, `/plc/lines`, `/plc/events`,
`/bridge/stats`, `/plc/metrics/*`, and `/plc/json/<topic>` once a PLC sends
JSON frames or records. In `/bridge/stats`, `malformed` stays 0.

## 5. Which tests after which change

| Changed | Run |
|---|---|
| `tcp_framed_transport.py` | `test_tcp_framed_transport`, `test_tcp_exchange`, `test_exchange_options`, `test_foxglove_bridge`, then `tools/verify_exchange.py` |
| the byte mapping (`mqtt_message_binding.py`, `io_layout.py`, `packFormat`) | `test_io_packing`, `test_component_message`, `test_mqtt_message_binding`, `test_compact_sensor_mapping`, `test_pack_format_auto_generation`, `test_st_export`, then the AX project's OFF-02 |
| `device_kinds.py`, a new kind of device | `test_device_kinds`, `test_st_export`, `test_plc_logic`, `test_io_signals` |
| `st_export.py` | `test_st_export`, `test_device_kinds`, then the AX project's OFF-01 and OFF-02 |
| `plc_foxglove_bridge.py`, `plc_records.py` | `test_foxglove_bridge`, `test_plc_records`, then the AX project's OFF-03 |
| `plc_mock.py`, `plc_logic.py` | `test_plc_mock`, `test_plc_logic`, `test_plc_lines`, then `tools/verify_exchange.py` on `logistics_full.json` |
| physics, devices, machines | `test_physics`, `test_friction_traction`, `test_drive`, `test_devices`, `test_machines`, `test_logistics` |
| the editor, CAD, UI | the modules of section 2's last table |
| before a commit | everything (`discover`) |

## 6. Tests that depend on the layout files

These tests load the demo layouts from this folder, so unsaved or uncommitted
edits to those files change their results:

| Test | Layout | What an edit can break |
|---|---|---|
| `test_io_signals.AtRestTest.test_plc_driven_components_start_at_rest` | `test_config.json` | a belt not under PLC control starts moving |
| `test_ui_trees.SimulatorTreesTest.test_tool_bar_is_one_row_under_the_status_bar` | `test_config.json` | the layout now references a drawing |
| `test_machines.PlcControlTest.test_the_demo_layout_runs_under_plc_control` | `machines_demo.json` | moved machines cycle fewer times in the test's time |
| `test_logistics.TransferTest.test_the_demo_pallet_loop_runs_on_rollers_and_transfers` | `logistics_demo.json` | a moved transfer is no longer on the pallet loop |
| `test_device_kinds` (the full system), `tools/build_logistics_full.py` | read both demos from git `HEAD`, so not affected | |

On 2026-11-09 the full run gave 4 failures, exactly the four tests above, with
`test_config.json`, `machines_demo.json` and `logistics_demo.json` edited in the
working copy. The same tests passed (9 of 9 in their classes) on the committed
files. To check the committed state without touching the working copy, export
it and run there:

```powershell
git archive -o $env:TEMP\sim_head.tar HEAD
New-Item -ItemType Directory -Force $env:TEMP\sim_head
tar -xf $env:TEMP\sim_head.tar -C $env:TEMP\sim_head
cd $env:TEMP\sim_head
<python_sim>\.venv\Scripts\python.exe -m unittest tests.test_io_signals tests.test_ui_trees tests.test_machines tests.test_logistics
```

## 7. Cross-repository checks

The AX project tests its generated ST and its demo program against this
simulator, with the simulator's Python. From the AX project, with
`CONVEYOR_SIMULATOR` set to this folder:

| AX check | What it runs here |
|---|---|
| OFF-01 `tools/generate_st_serde.py --sim-config ... --check` | this folder's `st_export.py` (the exchange contract) |
| OFF-02 `tools/check_sim_serde.py` | the generated serializers against `ComponentMessage`, for any layouts given |
| OFF-03 `tools/sim_demo_twin.py` | `main.py` headless and `plc_foxglove_bridge.py`, against a model of the AX demo program |

A change to the frame format, the byte mapping, `st_export.py`, the bridge or
`plc_records.py` can break those checks without failing a test here: run them
after such a change (section 5).
